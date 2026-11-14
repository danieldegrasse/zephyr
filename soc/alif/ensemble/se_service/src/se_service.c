/*
 * Copyright (C) 2024  Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <alif_se_service.h>
#include <zephyr/sys/barrier.h>
#include <soc.h>
#include "se_protocol.h"
#include "se_service_ids.h"
#include "zephyr/toolchain.h"

LOG_MODULE_REGISTER(se_service, CONFIG_ALIF_SE_LOG_LEVEL);

#define CH_ID         0
#define MUTEX_TIMEOUT 15000
#define MBOX_TIMEOUT  1000
#define MAX_TRIES     100

static K_MUTEX_DEFINE(svc_mutex);
static K_SEM_DEFINE(rx_signalled, 0, 1);

static const struct device *mbox_dev;
static uint32_t se_toc_version;

/* SE ready state - used for lazy initialization */
static atomic_t se_ready = ATOMIC_INIT(0);

/*
 * Local run profile cache to reduce SE service calls.
 * The cached profile represents the last configuration sent to SE.
 * Used for read-modify-write operations and to skip redundant SE calls.
 */
static alif_run_profile_t cached_run_profile;
static bool run_profile_initialized;

/* Manufacturing data for older Ensemble Family revision <= REV_B2 */
typedef struct {
	uint8_t x_loc: 7;
	uint8_t y_loc: 7;
	uint8_t wfr_id: 5;
	uint8_t year: 6;
	uint8_t fab_id: 1;
	uint8_t week: 6;
	uint8_t lot_no: 8;
} mfg_data_v1_t;

/* Manufacturing data for Ensemble Family revision >= REV_B3 */
typedef struct {
	uint8_t year: 6;
	uint8_t zero_1: 2;
	uint8_t wfr_id: 5;
	uint8_t zero_2: 2;
	uint8_t fab_id: 1;
	uint8_t y_loc: 7;
	uint8_t zero_3: 1;
	uint8_t x_loc: 7;
	uint8_t zero_4: 1;
	uint8_t zero_5: 8;
	uint8_t zero_6: 8;
	uint8_t lot_no: 8;
	uint8_t week: 6;
	uint8_t zero_7: 2;
} mfg_data_v2_t;

typedef union {
	service_header_t service_header;
	get_rnd_svc_t get_rnd_svc_d;
	get_se_revision_t get_se_revision_svc_d;
	get_toc_number_svc_t get_toc_number_svc_d;
	get_toc_version_svc_t get_toc_version_svc_d;
	get_device_part_svc_t get_device_part_svc_d;
	get_device_revision_data_t get_device_revision_data_d;
	net_proc_boot_svc_t boot_svc_d;
	net_proc_shutdown_svc_t shutdown_svc_d;
	set_services_capabilities_t set_services_capabilities_d;
	aipm_get_run_profile_svc_t get_run_d;
	aipm_set_run_profile_svc_t set_run_d;
	aipm_set_off_profile_svc_t set_off_d;
	aipm_get_off_profile_svc_t get_off_d;
	control_cpu_svc_t cpu_reboot_d;
	se_sleep_svc_t se_sleep_d;
	update_stoc_svc_t update_stoc_svc_d;
	clk_set_clk_divider_svc_t set_clk_divider_d;
	process_toc_entry_svc_t process_toc_entry_svc_d;
	otp_data_t otp_svc_d;
	boot_cpu_svc_t boot_cpu_svc_d;
	power_setting_svc_t power_setting_svc_d;
	lp_cmp_configure_svc_t lp_cmp_configure_svc_d;
	clock_setting_svc_t clock_setting_svc_d;
} alif_se_service_all_svc_t;

/*
 * The SE communication buffer is shared with the Secure Enclave over MHUv2 and
 * kept coherent with explicit cache maintenance (flush before send, invalidate
 * after receive). It must therefore occupy whole cache lines of its own: if a
 * neighbouring variable shares a boundary cache line, that line can be written
 * back over the SE response, corrupting it (seen as all-zero data when e.g.
 * CONFIG_THREAD_MONITOR changes the .bss layout).
 */
#if defined(CONFIG_DCACHE_LINE_SIZE) && (CONFIG_DCACHE_LINE_SIZE > 0)
#define SE_SVC_BUF_ALIGN CONFIG_DCACHE_LINE_SIZE
#else
#define SE_SVC_BUF_ALIGN sizeof(uint32_t)
#endif

static union {
	alif_se_service_all_svc_t svc;
	uint8_t pad[ROUND_UP(sizeof(alif_se_service_all_svc_t), SE_SVC_BUF_ALIGN)];
} alif_se_service_all_svc_buf __aligned(SE_SVC_BUF_ALIGN);
#define alif_se_service_all_svc_d (alif_se_service_all_svc_buf.svc)

static uint32_t global_address;

/**
 * @brief Send data to SE through MHUv2.
 *
 * Interrupt/sem mode is used when the thread can yield
 * (implemented within the mbox driver)
 *
 * parameters,
 * @ptr     - placeholder for data to be sent.
 * @size    - size of data.
 *
 * returns,
 * 0      - success.
 * -EBUSY  - SE has not consumed previous message.
 */
static int send_msg_to_se(uint32_t *ptr, uint32_t size)
{
	int err;
	int service_id = ((service_header_t *)ptr)->hdr_service_id;
	mem_addr_t mptr = (mem_addr_t)ptr;
	struct mbox_msg msg;

	global_address = ensemble_tcm_to_global_addr(mptr);
	barrier_dmem_fence_full();
	sys_cache_data_flush_range(ptr, size);

	msg.data = &global_address;
	msg.size = sizeof(global_address);

	if (k_can_yield()) {
		/*
		 * Block SUSPEND_TO_RAM / SOFT_OFF while we hold svc_mutex and
		 * wait on the SE mailbox response. Otherwise the idle thread
		 * may run PM device-suspend callbacks
		 */
		pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
		pm_policy_state_lock_get(PM_STATE_SOFT_OFF, PM_ALL_SUBSTATES);
	}

	/*
	 * Perform transaction. MBOX driver will fall back to polling mode
	 * if we are in interrupt context.
	 */
	err = mbox_send(mbox_dev, CH_ID, &msg);
	if (err) {
		LOG_ERR("failed to send request for service ID %d (error: %d)\n", service_id, err);
		goto out;
	}

	err = k_sem_take(&rx_signalled, k_can_yield() ? K_MSEC(MBOX_TIMEOUT) : K_NO_WAIT);
	if (err) {
		LOG_ERR("failed to receive response for service ID %d (error: %d)\n", service_id,
			err);
		goto out;
	}

out:
	if (k_can_yield()) {
		pm_policy_state_lock_put(PM_STATE_SOFT_OFF, PM_ALL_SUBSTATES);
		pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
	}

	sys_cache_data_invd_range(ptr, size);
	return err;
}

/**
 * @brief Internal: Synchronize with SE (assumes svc_mutex is held)
 *
 * This internal function performs the actual SE synchronization work.
 * The caller must hold svc_mutex before calling this function.
 *
 * returns,
 * 0      - On success. SE is woken up to service SE service requests.
 * -EAGAIN - Timed out waiting for SE response.
 * -EBUSY  - SE communication channel is busy.
 */
static int alif_se_service_sync_locked(void)
{
	int err, i = 0;

	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.service_header.hdr_service_id = SERVICE_MAINTENANCE_HEARTBEAT_ID;

	while (i < MAX_TRIES) {
		err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.service_header,
				     sizeof(alif_se_service_all_svc_d.service_header));
		if (!err) {
			return 0;
		}
		/* SE service timed out. Increment count */
		++i;
	}

	LOG_ERR("Failed to synchronize with SE (error = %d)", err);
	return err;
}

int alif_se_service_sync(void)
{
	int ret;

	ret = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (ret) {
		LOG_ERR("Unable to lock mutex (error = %d)", ret);
		return ret;
	}

	ret = alif_se_service_sync_locked();

	k_mutex_unlock(&svc_mutex);
	return ret;
}

/**
 * @brief Ensure SE is ready to receive service calls
 *
 * This function ensures the Secure Enclave is awake and synchronized,
 * ready to process service requests. Uses lazy initialization - only
 * syncs on first call or after SE has been put to sleep.
 *
 * Safe to call from multiple threads - uses mutex for serialization.
 * The atomic flag provides fast-path optimization for subsequent calls.
 *
 * returns,
 * 0      - On success. SE is ready to receive service calls.
 * -EAGAIN - Timed out waiting for mutex or SE response.
 * -EBUSY  - SE communication channel is busy.
 */
static int alif_se_service_ensure_ready(void)
{
	int ret;

	/* Fast path: SE already ready (lock-free atomic read) */
	if (atomic_get(&se_ready)) {
		return 0;
	}

	/* Slow path: Need to synchronize with SE */
	int err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));

	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)", err);
		return err;
	}

	/* Double-check inside mutex - another thread may have already synced */
	if (atomic_get(&se_ready)) {
		k_mutex_unlock(&svc_mutex);
		return 0;
	}

	/* Perform SE sync while holding mutex */
	ret = alif_se_service_sync_locked();
	if (ret == 0) {
		atomic_set(&se_ready, 1);
		LOG_DBG("SE now ready to receive service calls");
	} else {
		LOG_ERR("Failed to sync with SE: %d", ret);
	}

	k_mutex_unlock(&svc_mutex);
	return ret;
}

int alif_se_service_heartbeat(void)
{
	int err;

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}
	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.service_header.hdr_service_id = SERVICE_MAINTENANCE_HEARTBEAT_ID;
	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.service_header,
			     sizeof(alif_se_service_all_svc_d.service_header));
	k_mutex_unlock(&svc_mutex);
	if (err) {
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}
	return 0;
}

int alif_se_service_update_stoc(uint8_t *img_addr, uint32_t img_size)
{
	int err, resp_err;
	mem_addr_t mptr = (mem_addr_t)img_addr;

	if (!img_addr || img_size == 0) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));

	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}

	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.update_stoc_svc_d.header.hdr_service_id = SERVICE_UPDATE_STOC;
	alif_se_service_all_svc_d.update_stoc_svc_d.send_image_address =
		ensemble_tcm_to_global_addr(mptr);
	alif_se_service_all_svc_d.update_stoc_svc_d.send_image_size = img_size;
	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.update_stoc_svc_d,
			     sizeof(alif_se_service_all_svc_d.update_stoc_svc_d));
	resp_err = alif_se_service_all_svc_d.update_stoc_svc_d.resp_error_code;

	k_mutex_unlock(&svc_mutex);

	if (err) {
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}
	if (resp_err) {
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}

	return 0;
}

int alif_se_service_get_rnd_num(uint8_t *buffer, uint16_t length)
{
	int err, resp_err = -1;

	if (!buffer) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}
	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.get_rnd_svc_d.header.hdr_service_id = SERVICE_CRYPTOCELL_GET_RND;
	alif_se_service_all_svc_d.get_rnd_svc_d.send_rnd_length = length;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.get_rnd_svc_d,
			     sizeof(alif_se_service_all_svc_d.get_rnd_svc_d));
	resp_err = alif_se_service_all_svc_d.get_rnd_svc_d.resp_error_code;

	if (err) {
		k_mutex_unlock(&svc_mutex);
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}
	if (resp_err) {
		k_mutex_unlock(&svc_mutex);
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}

	memcpy(buffer, (uint8_t *)alif_se_service_all_svc_d.get_rnd_svc_d.resp_rnd, length);
	k_mutex_unlock(&svc_mutex);

	return 0;
}

int alif_se_service_get_toc_number(uint32_t *ptoc)
{
	int err, resp_err = -1;

	if (!ptoc) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}
	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.get_toc_number_svc_d.header.hdr_service_id =
		SERVICE_SYSTEM_MGMT_GET_TOC_NUMBER;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.get_toc_number_svc_d,
			     sizeof(alif_se_service_all_svc_d.get_toc_number_svc_d));
	resp_err = alif_se_service_all_svc_d.get_toc_number_svc_d.resp_error_code;

	if (err) {
		k_mutex_unlock(&svc_mutex);
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}
	if (resp_err) {
		k_mutex_unlock(&svc_mutex);
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}

	*ptoc = alif_se_service_all_svc_d.get_toc_number_svc_d.resp_number_of_toc;
	k_mutex_unlock(&svc_mutex);

	return 0;
}

int alif_se_service_get_toc_version(uint32_t *pversion)
{
	int err, resp_err;

	if (!pversion) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}
	/* Check if the TOC version has already been read */
	if (se_toc_version) {
		*pversion = se_toc_version;
		return 0;
	}

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}
	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.get_toc_version_svc_d.header.hdr_service_id =
		SERVICE_SYSTEM_MGMT_GET_TOC_VERSION;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.get_toc_version_svc_d,
			     sizeof(alif_se_service_all_svc_d.get_toc_version_svc_d));
	resp_err = alif_se_service_all_svc_d.get_toc_version_svc_d.resp_error_code;

	if (err) {
		k_mutex_unlock(&svc_mutex);
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}
	if (resp_err) {
		k_mutex_unlock(&svc_mutex);
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}

	*pversion = alif_se_service_all_svc_d.get_toc_version_svc_d.resp_version;
	/* Save TOC version in static variable for caching */
	se_toc_version = alif_se_service_all_svc_d.get_toc_version_svc_d.resp_version;
	LOG_DBG("toc version: %x", se_toc_version);

	k_mutex_unlock(&svc_mutex);
	return 0;
}

int alif_se_service_get_se_revision(uint8_t *prev)
{
	int err, resp_err = -1;

	if (!prev) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}
	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.get_se_revision_svc_d.header.hdr_service_id =
		SERVICE_APPLICATION_FIRMWARE_VERSION_ID;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.get_se_revision_svc_d,
			     sizeof(alif_se_service_all_svc_d.get_se_revision_svc_d));
	resp_err = alif_se_service_all_svc_d.get_se_revision_svc_d.resp_error_code;

	if (err) {
		k_mutex_unlock(&svc_mutex);
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}
	if (resp_err) {
		k_mutex_unlock(&svc_mutex);
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}

	memcpy(prev, (uint8_t *)alif_se_service_all_svc_d.get_se_revision_svc_d.resp_se_revision,
	       alif_se_service_all_svc_d.get_se_revision_svc_d.resp_se_revision_length);
	k_mutex_unlock(&svc_mutex);

	return 0;
}

int alif_se_service_get_device_part_number(uint32_t *pdev_part)
{
	int err, resp_err = -1;

	if (!pdev_part) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}
	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.get_device_part_svc_d.header.hdr_service_id =
		SERVICE_SYSTEM_MGMT_GET_DEVICE_PART_NUMBER;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.get_device_part_svc_d,
			     sizeof(alif_se_service_all_svc_d.get_device_part_svc_d));
	resp_err = alif_se_service_all_svc_d.get_device_part_svc_d.resp_error_code;

	if (err) {
		k_mutex_unlock(&svc_mutex);
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}
	if (resp_err) {
		k_mutex_unlock(&svc_mutex);
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}

	*pdev_part = alif_se_service_all_svc_d.get_device_part_svc_d.resp_device_string;
	k_mutex_unlock(&svc_mutex);

	return 0;
}

int alif_se_service_system_get_device_data(alif_device_revision_data_t *pdev_data)
{
	int err, resp_err = -1;

	if (!pdev_data) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}
	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.get_device_revision_data_d.header.hdr_service_id =
		SERVICE_SYSTEM_MGMT_GET_DEVICE_REVISION_DATA;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.get_device_revision_data_d,
			     sizeof(alif_se_service_all_svc_d.get_device_revision_data_d));
	resp_err = alif_se_service_all_svc_d.get_device_revision_data_d.resp_error_code;
	if (err) {
		k_mutex_unlock(&svc_mutex);
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}
	if (resp_err) {
		k_mutex_unlock(&svc_mutex);
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}
	pdev_data->revision_id = alif_se_service_all_svc_d.get_device_revision_data_d.revision_id;
	memcpy((uint8_t *)pdev_data->SerialN,
	       (uint8_t *)alif_se_service_all_svc_d.get_device_revision_data_d.SerialN,
	       sizeof(pdev_data->SerialN));
	memcpy((uint8_t *)pdev_data->ALIF_PN,
	       (uint8_t *)alif_se_service_all_svc_d.get_device_revision_data_d.ALIF_PN,
	       sizeof(pdev_data->ALIF_PN));
	memcpy((uint8_t *)pdev_data->HBK0,
	       (uint8_t *)alif_se_service_all_svc_d.get_device_revision_data_d.HBK0,
	       sizeof(pdev_data->HBK0));
	memcpy((uint8_t *)pdev_data->DCU,
	       (uint8_t *)alif_se_service_all_svc_d.get_device_revision_data_d.DCU,
	       sizeof(pdev_data->DCU));
	memcpy((uint8_t *)pdev_data->config,
	       (uint8_t *)alif_se_service_all_svc_d.get_device_revision_data_d.config,
	       sizeof(pdev_data->config));
	memcpy((uint8_t *)pdev_data->HBK1,
	       (uint8_t *)alif_se_service_all_svc_d.get_device_revision_data_d.HBK1,
	       sizeof(pdev_data->HBK1));
	memcpy((uint8_t *)pdev_data->HBK_FW,
	       (uint8_t *)alif_se_service_all_svc_d.get_device_revision_data_d.HBK_FW,
	       sizeof(pdev_data->HBK_FW));
	memcpy((uint8_t *)pdev_data->MfgData,
	       (uint8_t *)alif_se_service_all_svc_d.get_device_revision_data_d.MfgData,
	       sizeof(pdev_data->MfgData));
	memcpy((uint8_t *)pdev_data->external_config,
	       (uint8_t *)alif_se_service_all_svc_d.get_device_revision_data_d.external_config,
	       sizeof(pdev_data->external_config));

	pdev_data->flags2 = alif_se_service_all_svc_d.get_device_revision_data_d.flags2;
	pdev_data->LCS = alif_se_service_all_svc_d.get_device_revision_data_d.LCS;

	k_mutex_unlock(&svc_mutex);
	return 0;
}

/**
 * @brief   Pack manufacturing data into 40 bits for EUI-64
 */
static void get_eui64_extension(mfg_data_v1_t *p_mfg_data, uint8_t *p_data_out)
{
	uint8_t seven_bits_1 = p_mfg_data->x_loc;
	uint8_t seven_bits_2 = p_mfg_data->y_loc;
	uint8_t six_bits_3 = (p_mfg_data->wfr_id << 1) | p_mfg_data->fab_id;
	uint8_t six_bits_4 = p_mfg_data->year;
	uint8_t six_bits_5 = p_mfg_data->week;
	uint8_t eight_bits = p_mfg_data->lot_no;

	/* x x x x x x x y */
	*p_data_out = (seven_bits_1 << 1) | ((seven_bits_2 & 0x40) >> 6);
	p_data_out++;
	/* y y y y y y wf wf */
	*p_data_out = ((seven_bits_2 & 0x3F) << 2) | ((six_bits_3 & 0x30) >> 4);
	p_data_out++;
	/* wf wf wf f yr yr yr yr */
	*p_data_out = ((six_bits_3 & 0x0F) << 4) | ((six_bits_4 & 0x3C) >> 2);
	p_data_out++;
	/* yr yr wk wk wk wk wk wk */
	*p_data_out = ((six_bits_4 & 0x03) << 6) | six_bits_5;
	p_data_out++;
	*p_data_out = eight_bits;
}

/**
 * @brief   Pack manufacturing data into 24 bits for EUI-48
 */
static void get_eui48_extension(mfg_data_v1_t *p_mfg_data, uint8_t *p_data_out)
{
	uint8_t six_bits_1 = p_mfg_data->x_loc & 0x3F;
	uint8_t six_bits_2 = p_mfg_data->y_loc & 0x3F;
	uint8_t six_bits_3 = (p_mfg_data->wfr_id << 1) | (p_mfg_data->lot_no & 0x1);
	uint8_t six_bits_4 = p_mfg_data->week;

	/* x x x x x x y y */
	*p_data_out = (six_bits_1 << 2) | ((six_bits_2 & 0x30) >> 4);
	p_data_out++;
	/* y y y y wf wf wf wf */
	*p_data_out = ((six_bits_2 & 0x0F) << 4) | ((six_bits_3 & 0x3C) >> 2);
	p_data_out++;
	/* wf lt wk wk wk wk wk wk */
	*p_data_out = ((six_bits_3 & 0x03) << 6) | six_bits_4;
}

static void alif_se_service_manufacture_data_parse(alif_device_revision_data_t *device_data,
						   mfg_data_v1_t *mfg_data)
{
	if (IS_ENABLED(CONFIG_SOC_FAMILY_ENSEMBLE) && device_data->revision_id < 0x0000b300) {
		mfg_data_v1_t *p_mfg_data = (mfg_data_v1_t *)device_data->MfgData;

		mfg_data->fab_id = p_mfg_data->fab_id;
		mfg_data->lot_no = p_mfg_data->lot_no;
		mfg_data->week = p_mfg_data->week;
		mfg_data->wfr_id = p_mfg_data->wfr_id;
		mfg_data->x_loc = p_mfg_data->x_loc;
		mfg_data->y_loc = p_mfg_data->y_loc;
		mfg_data->year = p_mfg_data->year;
	} else {
		/* Ensemble(rev>=B3) families use new Manufacture data model */
		mfg_data_v2_t *p_mfg_data = (mfg_data_v2_t *)device_data->MfgData;

		mfg_data->fab_id = p_mfg_data->fab_id;
		mfg_data->lot_no = p_mfg_data->lot_no;
		mfg_data->week = p_mfg_data->week;
		mfg_data->wfr_id = p_mfg_data->wfr_id;
		mfg_data->x_loc = p_mfg_data->x_loc;
		mfg_data->y_loc = p_mfg_data->y_loc;
		mfg_data->year = p_mfg_data->year;
	}
}

int alif_se_system_get_eui_extension(bool is_eui48, uint8_t *eui_extension)
{
	alif_device_revision_data_t device_data;
	mfg_data_v1_t p_mfg_dat;
	int ret;

	if (!eui_extension) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}

	ret = alif_se_service_system_get_device_data(&device_data);
	if (ret) {
		return ret;
	}

	alif_se_service_manufacture_data_parse(&device_data, &p_mfg_dat);

	/* Use Manufactured data  */
	if (is_eui48) {
		get_eui48_extension(&p_mfg_dat, eui_extension);
	} else {
		get_eui64_extension(&p_mfg_dat, eui_extension);
	}

	return ret;
}

int alif_se_service_boot_es0(uint8_t *nvds_buff, uint16_t nvds_size, uint32_t clock_select,
			     bool hpa_mode)
{
	int err, resp_err;
	mem_addr_t mptr = (mem_addr_t)nvds_buff;

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}
	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));

	alif_se_service_all_svc_d.boot_svc_d.header.hdr_service_id = SERVICE_EXTSYS0_BOOT_SET_ARGS;
	alif_se_service_all_svc_d.boot_svc_d.send_nvds_src_addr = ensemble_tcm_to_global_addr(mptr);
	alif_se_service_all_svc_d.boot_svc_d.send_nvds_dst_addr = 0x501D0000;
	alif_se_service_all_svc_d.boot_svc_d.send_nvds_copy_len = nvds_size;
	alif_se_service_all_svc_d.boot_svc_d.send_trng_dst_addr = 0x501D0200;
	alif_se_service_all_svc_d.boot_svc_d.send_trng_len = 64;
	alif_se_service_all_svc_d.boot_svc_d.send_internal_clock_select = clock_select;

	alif_se_service_all_svc_d.boot_svc_d.send_configuration =
		hpa_mode ? SERVICES_NET_PROC_BOOT_CONFIGURATION_HPA
			 : SERVICES_NET_PROC_BOOT_CONFIGURATION_NONE;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.boot_svc_d,
			     sizeof(alif_se_service_all_svc_d.boot_svc_d));

	resp_err = alif_se_service_all_svc_d.boot_svc_d.resp_error_code;

	k_mutex_unlock(&svc_mutex);
	if (err) {
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}
	if (resp_err) {
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}
	return 0;
}

int alif_se_service_shutdown_es0(void)
{
	int err, resp_err = -1;

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}
	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.shutdown_svc_d.header.hdr_service_id = SERVICE_EXTSYS0_SHUTDOWN;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.shutdown_svc_d,
			     sizeof(alif_se_service_all_svc_d.shutdown_svc_d));
	resp_err = alif_se_service_all_svc_d.shutdown_svc_d.resp_error_code;

	k_mutex_unlock(&svc_mutex);
	if (err) {
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}
	if (resp_err) {
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}
	return 0;
}

int alif_se_service_get_run_cfg(alif_run_profile_t *pp)
{
	int err, resp_err = -1;

	if (!pp) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}

	/* Always query SE firmware for current state */
	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.get_run_d.header.hdr_service_id = SERVICE_POWER_GET_RUN_REQ_ID;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.get_run_d,
			     sizeof(alif_se_service_all_svc_d.get_run_d));
	resp_err = alif_se_service_all_svc_d.get_run_d.resp_error_code;

	if (err) {
		LOG_ERR("%s failed with %d\n", __func__, err);
		k_mutex_unlock(&svc_mutex);
		return err;
	}
	if (resp_err) {
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		k_mutex_unlock(&svc_mutex);
		return resp_err;
	}

	pp->aon_clk_src = alif_se_service_all_svc_d.get_run_d.resp_aon_clk_src;
	pp->run_clk_src = alif_se_service_all_svc_d.get_run_d.resp_run_clk_src;
	pp->cpu_clk_freq = alif_se_service_all_svc_d.get_run_d.resp_cpu_clk_freq;
	pp->scaled_clk_freq = alif_se_service_all_svc_d.get_run_d.resp_scaled_clk_freq;
	pp->dcdc_mode = alif_se_service_all_svc_d.get_run_d.resp_dcdc_mode;
	pp->dcdc_voltage = alif_se_service_all_svc_d.get_run_d.resp_dcdc_voltage;
	pp->memory_blocks = alif_se_service_all_svc_d.get_run_d.resp_memory_blocks;
	pp->ip_clock_gating = alif_se_service_all_svc_d.get_run_d.resp_ip_clock_gating;
	pp->phy_pwr_gating = alif_se_service_all_svc_d.get_run_d.resp_phy_pwr_gating;
	pp->power_domains = alif_se_service_all_svc_d.get_run_d.resp_power_domains;
	pp->vdd_ioflex_3V3 = alif_se_service_all_svc_d.get_run_d.resp_vdd_ioflex_3V3;
	k_mutex_unlock(&svc_mutex);

	return 0;
}

int alif_se_service_get_last_set_run_cfg(alif_run_profile_t *pp)
{
	int ret = 0;

	if (!pp) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}

	ret = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (ret) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", ret);
		return ret;
	}

	/*
	 * Return cached profile if initialized.
	 * This returns what THIS core has set, not the current system state.
	 */
	if (run_profile_initialized) {
		memcpy(pp, &cached_run_profile, sizeof(alif_run_profile_t));
	} else {
		LOG_WRN("Profile cache not initialized - use alif_se_service_get_run_cfg()");
		ret = -ENODATA;
	}

	k_mutex_unlock(&svc_mutex);
	return ret;
}

/**
 * @brief Check if run profile has changed
 *
 * @param pp Requested profile
 * @return true if profile changed, false otherwise
 */
static bool alif_se_service_profile_changed(const alif_run_profile_t *pp)
{
	if (!run_profile_initialized) {
		return true;
	}

	return pp->aon_clk_src != cached_run_profile.aon_clk_src ||
	       pp->run_clk_src != cached_run_profile.run_clk_src ||
	       pp->cpu_clk_freq != cached_run_profile.cpu_clk_freq ||
	       pp->scaled_clk_freq != cached_run_profile.scaled_clk_freq ||
	       pp->dcdc_mode != cached_run_profile.dcdc_mode ||
	       pp->dcdc_voltage != cached_run_profile.dcdc_voltage ||
	       pp->memory_blocks != cached_run_profile.memory_blocks ||
	       pp->ip_clock_gating != cached_run_profile.ip_clock_gating ||
	       pp->phy_pwr_gating != cached_run_profile.phy_pwr_gating ||
	       pp->power_domains != cached_run_profile.power_domains ||
	       pp->vdd_ioflex_3V3 != cached_run_profile.vdd_ioflex_3V3;
}

int alif_se_service_set_run_cfg(alif_run_profile_t *pp)
{
	int err, resp_err = -1;

	if (!pp) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}

	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}

	/* Check if profile changed - skip SE call if unchanged */
	if (!alif_se_service_profile_changed(pp)) {
		k_mutex_unlock(&svc_mutex);
		return 0;
	}

	/* Profile changed - update SE */
	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));

	alif_se_service_all_svc_d.set_run_d.header.hdr_service_id = SERVICE_POWER_SET_RUN_REQ_ID;
	alif_se_service_all_svc_d.set_run_d.send_aon_clk_src = pp->aon_clk_src;
	alif_se_service_all_svc_d.set_run_d.send_run_clk_src = pp->run_clk_src;
	alif_se_service_all_svc_d.set_run_d.send_cpu_clk_freq = pp->cpu_clk_freq;
	alif_se_service_all_svc_d.set_run_d.send_scaled_clk_freq = pp->scaled_clk_freq;
	alif_se_service_all_svc_d.set_run_d.send_dcdc_mode = pp->dcdc_mode;
	alif_se_service_all_svc_d.set_run_d.send_dcdc_voltage = pp->dcdc_voltage;
	alif_se_service_all_svc_d.set_run_d.send_memory_blocks = pp->memory_blocks;
	alif_se_service_all_svc_d.set_run_d.send_ip_clock_gating = pp->ip_clock_gating;
	alif_se_service_all_svc_d.set_run_d.send_phy_pwr_gating = pp->phy_pwr_gating;
	alif_se_service_all_svc_d.set_run_d.send_power_domains = pp->power_domains;
	alif_se_service_all_svc_d.set_run_d.send_vdd_ioflex_3V3 = pp->vdd_ioflex_3V3;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.set_run_d,
			     sizeof(alif_se_service_all_svc_d.set_run_d));
	resp_err = alif_se_service_all_svc_d.set_run_d.resp_error_code;

	if (err) {
		LOG_ERR("%s failed with %d\n", __func__, err);
		k_mutex_unlock(&svc_mutex);
		return err;
	}
	if (resp_err) {
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		k_mutex_unlock(&svc_mutex);
		return resp_err;
	}

	/* Update cache on success */
	memcpy(&cached_run_profile, pp, sizeof(alif_run_profile_t));
	run_profile_initialized = true;

	k_mutex_unlock(&svc_mutex);
	return 0;
}

int alif_se_service_get_off_cfg(alif_off_profile_t *wp)
{
	int err, resp_err = -1;

	if (!wp) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}
	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.get_off_d.header.hdr_service_id = SERVICE_POWER_GET_OFF_REQ_ID;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.get_off_d,
			     sizeof(alif_se_service_all_svc_d.get_off_d));
	resp_err = alif_se_service_all_svc_d.get_off_d.resp_error_code;

	if (err) {
		LOG_ERR("%s failed with %d\n", __func__, err);
		k_mutex_unlock(&svc_mutex);
		return err;
	}
	if (resp_err) {
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		k_mutex_unlock(&svc_mutex);
		return resp_err;
	}

	wp->dcdc_voltage = alif_se_service_all_svc_d.get_off_d.resp_dcdc_voltage;
	wp->memory_blocks = alif_se_service_all_svc_d.get_off_d.resp_memory_blocks;
	wp->power_domains = alif_se_service_all_svc_d.get_off_d.resp_power_domains;
	wp->aon_clk_src = alif_se_service_all_svc_d.get_off_d.resp_aon_clk_src;
	wp->stby_clk_src = alif_se_service_all_svc_d.get_off_d.resp_stby_clk_src;
	wp->stby_clk_freq = alif_se_service_all_svc_d.get_off_d.resp_stby_clk_freq;
	wp->ip_clock_gating = alif_se_service_all_svc_d.get_off_d.resp_ip_clock_gating;
	wp->phy_pwr_gating = alif_se_service_all_svc_d.get_off_d.resp_phy_pwr_gating;
	wp->vdd_ioflex_3V3 = alif_se_service_all_svc_d.get_off_d.resp_vdd_ioflex_3V3;
	wp->vtor_address = alif_se_service_all_svc_d.get_off_d.resp_vtor_address;
	wp->vtor_address_ns = alif_se_service_all_svc_d.get_off_d.resp_vtor_address_ns;
	wp->wakeup_events = alif_se_service_all_svc_d.get_off_d.resp_wakeup_events;
	wp->ewic_cfg = alif_se_service_all_svc_d.get_off_d.resp_ewic_cfg;
	k_mutex_unlock(&svc_mutex);
	return 0;
}

int alif_se_service_set_off_cfg(alif_off_profile_t *wp)
{
	int err, resp_err = -1;

	if (!wp) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}
	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.set_off_d.header.hdr_service_id = SERVICE_POWER_SET_OFF_REQ_ID;
	alif_se_service_all_svc_d.set_off_d.send_dcdc_voltage = wp->dcdc_voltage;
	alif_se_service_all_svc_d.set_off_d.send_memory_blocks = wp->memory_blocks;
	alif_se_service_all_svc_d.set_off_d.send_power_domains = wp->power_domains;
	alif_se_service_all_svc_d.set_off_d.send_aon_clk_src = wp->aon_clk_src;
	alif_se_service_all_svc_d.set_off_d.send_stby_clk_src = wp->stby_clk_src;
	alif_se_service_all_svc_d.set_off_d.send_stby_clk_freq = wp->stby_clk_freq;
	alif_se_service_all_svc_d.set_off_d.send_ip_clock_gating = wp->ip_clock_gating;
	alif_se_service_all_svc_d.set_off_d.send_phy_pwr_gating = wp->phy_pwr_gating;
	alif_se_service_all_svc_d.set_off_d.send_vdd_ioflex_3V3 = wp->vdd_ioflex_3V3;
	alif_se_service_all_svc_d.set_off_d.send_vtor_address = wp->vtor_address;
	alif_se_service_all_svc_d.set_off_d.send_vtor_address_ns = wp->vtor_address_ns;
	alif_se_service_all_svc_d.set_off_d.send_wakeup_events = wp->wakeup_events;
	alif_se_service_all_svc_d.set_off_d.send_ewic_cfg = wp->ewic_cfg;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.set_off_d,
			     sizeof(alif_se_service_all_svc_d.set_off_d));
	resp_err = alif_se_service_all_svc_d.set_off_d.resp_error_code;

	k_mutex_unlock(&svc_mutex);
	if (err) {
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}
	if (resp_err) {
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}
	return 0;
}

int alif_se_service_se_sleep_req(uint32_t param)
{
	int err, resp_err = -1;

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}
	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));

	alif_se_service_all_svc_d.se_sleep_d.send_param = param;
	alif_se_service_all_svc_d.se_sleep_d.header.hdr_service_id = SERVICE_POWER_SE_SLEEP_REQ_ID;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.se_sleep_d,
			     sizeof(alif_se_service_all_svc_d.se_sleep_d));
	resp_err = alif_se_service_all_svc_d.se_sleep_d.resp_error_code;

	if (err) {
		k_mutex_unlock(&svc_mutex);
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}
	if (resp_err) {
		k_mutex_unlock(&svc_mutex);
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}

	/*
	 * SE is now asleep. Reset the ready flag so the next SE service call
	 * will re-sync (wake up) the SE before processing the request.
	 */
	atomic_set(&se_ready, 0);
	LOG_DBG("SE put to sleep - ready flag cleared");

	k_mutex_unlock(&svc_mutex);
	return 0;
}

int alif_se_service_system_set_services_debug(bool debug_enable)
{
	int err, resp_err = -1;

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}
	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));

	alif_se_service_all_svc_d.set_services_capabilities_d.header.hdr_service_id =
		SERVICE_POWER_SET_OFF_REQ_ID;
	alif_se_service_all_svc_d.set_services_capabilities_d.send_services_debug = debug_enable;
	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.set_services_capabilities_d,
			     sizeof(alif_se_service_all_svc_d.set_services_capabilities_d));
	resp_err = alif_se_service_all_svc_d.set_services_capabilities_d.resp_error_code;

	k_mutex_unlock(&svc_mutex);
	if (err) {
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}
	if (resp_err) {
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}

	return 0;
}

int alif_se_service_boot_reset_soc(void)
{
	int err, i = 0;

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}

	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.service_header.hdr_service_id = SERVICE_BOOT_RESET_SOC;

	while (i < MAX_TRIES) {
		err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.service_header,
				     sizeof(alif_se_service_all_svc_d.service_header));
		if (!err) {
			break;
		}
		/* SE service timed out. Increment count */
		++i;
	}
	k_mutex_unlock(&svc_mutex);
	if (i >= MAX_TRIES) {
		LOG_ERR("Failed to reset SoC with SE (error = %d)\n", err);
		return err;
	}
	return 0;
}

int alif_se_service_boot_reset_cpu(uint32_t cpu_id)
{
	int err, i = 0, resp_err = -1;

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}

	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.cpu_reboot_d.header.hdr_service_id = SERVICE_BOOT_RESET_CPU;
	alif_se_service_all_svc_d.cpu_reboot_d.send_cpu_id = cpu_id;

	while (i < MAX_TRIES) {
		err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.service_header,
				     sizeof(alif_se_service_all_svc_d.service_header));
		if (!err) {
			break;
		}
		/* SE service timed out. Increment count */
		++i;
	}
	resp_err = alif_se_service_all_svc_d.cpu_reboot_d.resp_error_code;
	k_mutex_unlock(&svc_mutex);
	if (i >= MAX_TRIES) {
		LOG_ERR("Failed to reset cpu with SE (error = %d)\n", err);
		return err;
	}
	if (resp_err) {
		LOG_ERR("received response error = %d\n", resp_err);
		return resp_err;
	}
	return 0;
}

int alif_se_service_clock_set_divider(alif_clock_divider_t divider, uint32_t value)
{
	int err, i = 0, resp_err = -1;

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}

	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.set_clk_divider_d.header.hdr_service_id =
		SERVICE_CLOCK_SET_DIVIDER;

	alif_se_service_all_svc_d.set_clk_divider_d.send_divider = divider;
	alif_se_service_all_svc_d.set_clk_divider_d.send_value = value;

	while (i < MAX_TRIES) {
		err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.service_header,
				     sizeof(alif_se_service_all_svc_d.service_header));
		if (!err) {
			break;
		}
		/* SE service timed out. Increment count */
		++i;
	}
	resp_err = alif_se_service_all_svc_d.set_clk_divider_d.resp_error_code;
	k_mutex_unlock(&svc_mutex);
	if (i >= MAX_TRIES) {
		LOG_ERR("Failed to set clock divider (error = %d)\n", err);
		return err;
	}
	if (resp_err) {
		LOG_ERR("received response error = %d\n", resp_err);
		return resp_err;
	}
	return 0;
}

int alif_se_service_clock_setting_get(alif_clock_setting_t setting, uint32_t *freq)
{
	int err, resp_err = -1;

	if (!freq) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}

	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}

	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.clock_setting_svc_d.header.hdr_service_id =
		SERVICE_CLOCK_SETTING_GET_REQ_ID;
	alif_se_service_all_svc_d.clock_setting_svc_d.send_setting_type = setting;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.clock_setting_svc_d,
			     sizeof(alif_se_service_all_svc_d.clock_setting_svc_d));
	resp_err = alif_se_service_all_svc_d.clock_setting_svc_d.resp_error_code;

	if (err) {
		LOG_ERR("%s failed with %d\n", __func__, err);
		k_mutex_unlock(&svc_mutex);
		return err;
	}
	if (resp_err) {
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		k_mutex_unlock(&svc_mutex);
		return resp_err;
	}

	*freq = alif_se_service_all_svc_d.clock_setting_svc_d.value;
	k_mutex_unlock(&svc_mutex);

	return 0;
}

int alif_se_service_process_toc_entry(const char *image_id)
{
	int err, resp_err = -1;

	if (!image_id) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}

	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));

	alif_se_service_all_svc_d.process_toc_entry_svc_d.header.hdr_service_id =
		SERVICE_BOOT_PROCESS_TOC_ENTRY;
	strncpy((char *)alif_se_service_all_svc_d.process_toc_entry_svc_d.send_entry_id, image_id,
		IMAGE_NAME_LENGTH);

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.process_toc_entry_svc_d,
			     sizeof(alif_se_service_all_svc_d.process_toc_entry_svc_d));

	resp_err = alif_se_service_all_svc_d.process_toc_entry_svc_d.resp_error_code;
	k_mutex_unlock(&svc_mutex);

	if (err) {
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}

	if (resp_err) {
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}

	return 0;
}

int alif_se_service_read_otp(uint32_t otp_offset, uint32_t *otp_word)
{
	int err, resp_err = -1;

	if (!otp_word) {
		LOG_ERR("Invalid argument\n");
		return -EINVAL;
	}

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));

	if (err) {
		LOG_ERR("Unable to lock mutex (err = %d)\n", err);
		return err;
	}

	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.otp_svc_d.header.hdr_service_id = SERVICE_SYSTEM_MGMT_READ_OTP;

	alif_se_service_all_svc_d.otp_svc_d.send_offset = otp_offset;
	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.otp_svc_d,
			     sizeof(alif_se_service_all_svc_d.otp_svc_d));

	resp_err = alif_se_service_all_svc_d.otp_svc_d.resp_error_code;
	k_mutex_unlock(&svc_mutex);

	if (err) {
		LOG_ERR("service_read_otp failed with %d\n", err);
		return err;
	}

	if (resp_err) {
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}

	*otp_word = alif_se_service_all_svc_d.otp_svc_d.otp_word;

	return 0;
}

int alif_se_service_enable_pd(uint32_t pd_id)
{
	alif_run_profile_t runp;
	int ret;

	if (pd_id >= 32) { /* power_domains is a uint32_t bitmask */
		return -EINVAL;
	}

	/* Ensure SE is ready to receive service calls */
	ret = alif_se_service_ensure_ready();
	if (ret) {
		return ret;
	}

	ret = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));

	if (ret) {
		LOG_ERR("Unable to lock mutex (err = %d)\n", ret);
		return ret;
	}

	ret = alif_se_service_get_last_set_run_cfg(&runp);
	if (ret) {
		goto out;
	}

	if (runp.power_domains & BIT(pd_id)) {
		ret = 0;
		goto out;
	}

	runp.power_domains |= BIT(pd_id);
	ret = alif_se_service_set_run_cfg(&runp);

out:
	k_mutex_unlock(&svc_mutex);
	return ret;
}

int alif_se_service_update_phy_pwr_gating(uint32_t mask, bool enable)
{
	alif_run_profile_t runp;
	int ret;

	/* Ensure SE is ready to receive service calls */
	ret = alif_se_service_ensure_ready();
	if (ret != 0) {
		return ret;
	}

	ret = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (ret != 0) {
		LOG_ERR("Unable to lock mutex (err = %d)", ret);
		return ret;
	}

	ret = alif_se_service_get_last_set_run_cfg(&runp);
	if (ret != 0) {
		goto out;
	}

	if (enable) {
		runp.phy_pwr_gating |= mask;
	} else {
		runp.phy_pwr_gating &= ~mask;
	}

	ret = alif_se_service_set_run_cfg(&runp);

out:
	k_mutex_unlock(&svc_mutex);
	return ret;
}

int alif_se_service_configure_lpcmp(const alif_lpcmp_configure_t *const config)
{
	int err;

	if (config == NULL) {
		return -EINVAL;
	}

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (err = %d)\n", err);
		return err;
	}

	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));

	alif_se_service_all_svc_d.lp_cmp_configure_svc_d.header.hdr_service_id =
		SERVICE_APPLICATION_LPCMP_CONFIGURE_ID;

	alif_se_service_all_svc_d.lp_cmp_configure_svc_d.comp_lp0_hyst = config->comp_lp0_hyst;
	alif_se_service_all_svc_d.lp_cmp_configure_svc_d.comp_lp0_in_m_sel =
		config->comp_lp0_in_m_sel;
	alif_se_service_all_svc_d.lp_cmp_configure_svc_d.comp_lp0_in_p_sel =
		config->comp_lp0_in_p_sel;
	alif_se_service_all_svc_d.lp_cmp_configure_svc_d.comp_lp_en = config->comp_lp_en;
	alif_se_service_all_svc_d.lp_cmp_configure_svc_d.lpcomp_clk32k_en =
		config->lpcomp_clk32k_en;
	alif_se_service_all_svc_d.lp_cmp_configure_svc_d.lpcomp_clk_sel = config->lpcomp_clk_sel;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.lp_cmp_configure_svc_d,
			     sizeof(alif_se_service_all_svc_d.lp_cmp_configure_svc_d));

	const int resp_err = alif_se_service_all_svc_d.lp_cmp_configure_svc_d.resp_error_code;

	k_mutex_unlock(&svc_mutex);

	if (err) {
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}

	if (resp_err) {
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}

	return 0;
}

int alif_se_service_power_settings_set(const alif_power_setting_t setting, const uint32_t value)
{
	int err;

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	if (setting > ALIF_POWER_SETTING_ANA_PERIPH_EN) {
		return -EINVAL;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (err = %d)\n", err);
		return err;
	}

	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));

	alif_se_service_all_svc_d.power_setting_svc_d.header.hdr_service_id =
		SERVICE_POWER_SETTING_CONFIG_REQ_ID;
	alif_se_service_all_svc_d.power_setting_svc_d.send_setting_type = setting;
	alif_se_service_all_svc_d.power_setting_svc_d.value = value;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.power_setting_svc_d,
			     sizeof(alif_se_service_all_svc_d.power_setting_svc_d));

	const int resp_err = alif_se_service_all_svc_d.power_setting_svc_d.resp_error_code;

	k_mutex_unlock(&svc_mutex);

	if (err) {
		LOG_ERR("%s failed with %d\n", __func__, err);
		return err;
	}

	if (resp_err) {
		LOG_ERR("%s: received response error = %d\n", __func__, resp_err);
		return resp_err;
	}

	return 0;
}

int alif_se_service_boot_cpu(uint32_t cpu_id, uint32_t address)
{
	int err, resp_err = 0;

	/* Ensure SE is ready to receive service calls */
	err = alif_se_service_ensure_ready();
	if (err) {
		return err;
	}

	err = k_mutex_lock(&svc_mutex, K_MSEC(MUTEX_TIMEOUT));
	if (err) {
		LOG_ERR("Unable to lock mutex (error = %d)\n", err);
		return err;
	}

	memset(&alif_se_service_all_svc_d, 0, sizeof(alif_se_service_all_svc_d));
	alif_se_service_all_svc_d.boot_cpu_svc_d.header.hdr_service_id = SERVICE_BOOT_CPU;
	alif_se_service_all_svc_d.boot_cpu_svc_d.send_cpu_id = cpu_id;
	alif_se_service_all_svc_d.boot_cpu_svc_d.send_address = address;

	err = send_msg_to_se((uint32_t *)&alif_se_service_all_svc_d.boot_cpu_svc_d,
			     sizeof(alif_se_service_all_svc_d.boot_cpu_svc_d));

	k_mutex_unlock(&svc_mutex);

	if (err) {
		LOG_ERR("SE service call failed with %d\n", err);
		return err;
	}

	resp_err = alif_se_service_all_svc_d.boot_cpu_svc_d.resp_error_code;

	if (resp_err) {
		LOG_ERR("boot_cpu response error = %d\n", resp_err);
		return resp_err;
	}

	return 0;
}

static void alif_se_service_mbox_cb(const struct device *dev, mbox_channel_id_t channel_id,
				    void *user_data, struct mbox_msg *data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);
	ARG_UNUSED(user_data);
	ARG_UNUSED(data);

	k_sem_give(&rx_signalled);
}

/**
 * @brief Initialize SE service and configure the provided mailbox
 *
 * @return 0 on success.
 * @return -ENODEV - if the mailbox device is not ready
 * @return other negative error codes from mailbox configuration functions.
 */
static int alif_se_service_mhuv2_init(void)
{
	mbox_dev = DEVICE_DT_GET(DT_CHOSEN(alif_secure_cpu_mhu));
	int ret;

	if (!device_is_ready(mbox_dev)) {
		LOG_ERR("MHU device not ready\n");
		return -ENODEV;
	}

	ret = mbox_set_enabled(mbox_dev, CH_ID, true);
	if (ret) {
		LOG_ERR("Failed to enable MHUv2 channel 0 (error = %d)\n", ret);
		return ret;
	}

	ret = mbox_register_callback(mbox_dev, CH_ID, alif_se_service_mbox_cb, NULL);
	if (ret) {
		LOG_ERR("Failed to register MHUv2 callback (error = %d)\n", ret);
		return ret;
	}

	/*
	 * During boot, read the initial run config. Other service calls will
	 * query and edit this configuration
	 */
	ret = alif_se_service_get_run_cfg(&cached_run_profile);
	if (ret) {
		LOG_ERR("Failed to get initial run config (error = %d)\n", ret);
		return ret;
	}
	run_profile_initialized = true;

	return 0;
}

SYS_INIT(alif_se_service_mhuv2_init, POST_KERNEL, CONFIG_ALIF_SE_SERVICE_INIT_PRIORITY);
