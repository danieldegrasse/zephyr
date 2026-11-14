/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <alif_se_service.h>

/* Matches VERSION_RESPONSE_LENGTH in the SE protocol */
#define SE_REVISION_LEN 80U
#define RND_LEN         16U
#define EUI48_EXT_LEN   3U
#define EUI64_EXT_LEN   5U

/* SE CPU identifiers for the M55 cores */
#define SE_CPU_ID_RTSS_HP 2U
#define SE_CPU_ID_RTSS_HE 3U

#define RESET_DELAY_SEC 5

static void print_hex(const char *label, const uint8_t *buf, size_t len)
{
	printk("%s: ", label);
	for (size_t i = 0; i < len; i++) {
		printk("%02x", buf[i]);
	}
	printk("\n");
}

static int show_se_info(void)
{
	uint8_t revision[SE_REVISION_LEN + 1U] = {0};
	uint32_t toc_number;
	uint32_t toc_version;
	int ret;

	ret = alif_se_service_get_se_revision(revision);
	if (ret != 0) {
		printk("get_se_revision failed: %d\n", ret);
		return ret;
	}
	printk("SE revision: %s\n", (char *)revision);

	ret = alif_se_service_get_toc_number(&toc_number);
	if (ret != 0) {
		printk("get_toc_number failed: %d\n", ret);
		return ret;
	}
	printk("TOC entries: %u\n", toc_number);

	ret = alif_se_service_get_toc_version(&toc_version);
	if (ret != 0) {
		printk("get_toc_version failed: %d\n", ret);
		return ret;
	}
	printk("TOC version: 0x%08x\n", toc_version);

	return 0;
}

static int show_device_info(void)
{
	alif_device_revision_data_t dev_data;
	uint8_t eui48_ext[EUI48_EXT_LEN];
	uint8_t eui64_ext[EUI64_EXT_LEN];
	uint32_t part_number;
	int ret;

	ret = alif_se_service_get_device_part_number(&part_number);
	if (ret != 0) {
		printk("get_device_part_number failed: %d\n", ret);
		return ret;
	}
	printk("Part number: 0x%08x\n", part_number);

	ret = alif_se_service_system_get_device_data(&dev_data);
	if (ret != 0) {
		printk("get_device_data failed: %d\n", ret);
		return ret;
	}
	printk("Revision ID: 0x%08x\n", dev_data.revision_id);
	printk("LCS: %u\n", dev_data.LCS);
	print_hex("Serial number", dev_data.SerialN, sizeof(dev_data.SerialN));

	ret = alif_se_system_get_eui_extension(true, eui48_ext);
	if (ret != 0) {
		printk("get_eui_extension (EUI-48) failed: %d\n", ret);
		return ret;
	}
	print_hex("EUI-48 extension", eui48_ext, sizeof(eui48_ext));

	ret = alif_se_system_get_eui_extension(false, eui64_ext);
	if (ret != 0) {
		printk("get_eui_extension (EUI-64) failed: %d\n", ret);
		return ret;
	}
	print_hex("EUI-64 extension", eui64_ext, sizeof(eui64_ext));

	return 0;
}

static int show_random(void)
{
	uint8_t rnd[RND_LEN];
	int ret;

	ret = alif_se_service_get_rnd_num(rnd, sizeof(rnd));
	if (ret != 0) {
		printk("get_rnd_num failed: %d\n", ret);
		return ret;
	}
	print_hex("Random", rnd, sizeof(rnd));

	return 0;
}

static int show_clocks(void)
{
	static const struct {
		alif_clock_setting_t setting;
		const char *name;
	} clocks[] = {
		{ALIF_CLOCK_SETTING_HFOSC_FREQ, "HFOSC"},
		{ALIF_CLOCK_SETTING_EXTSYS0_FREQ, "EXTSYS0"},
		{ALIF_CLOCK_SETTING_EXTSYS1_FREQ, "EXTSYS1"},
		{ALIF_CLOCK_SETTING_AXI_FREQ, "AXI"},
		{ALIF_CLOCK_SETTING_AHB_FREQ, "AHB"},
		{ALIF_CLOCK_SETTING_APB_FREQ, "APB"},
		{ALIF_CLOCK_SETTING_SYSREF_FREQ, "SYSREF"},
	};
	uint32_t freq;
	int ret;

	for (size_t i = 0; i < ARRAY_SIZE(clocks); i++) {
		ret = alif_se_service_clock_setting_get(clocks[i].setting, &freq);
		if (ret != 0) {
			printk("clock_setting_get (%s) failed: %d\n", clocks[i].name, ret);
			return ret;
		}
		printk("%s clock: %u Hz\n", clocks[i].name, freq);
	}

	return 0;
}

static int show_profiles(void)
{
	alif_run_profile_t run;
	alif_off_profile_t off;
	int ret;

	ret = alif_se_service_get_run_cfg(&run);
	if (ret != 0) {
		printk("get_run_cfg failed: %d\n", ret);
		return ret;
	}
	printk("Run profile: power_domains 0x%08x, dcdc %u mV, run_clk_src %d, cpu_clk_freq %d\n",
	       run.power_domains, run.dcdc_voltage, run.run_clk_src, run.cpu_clk_freq);

	/* Write back the current profile; this populates the local cache */
	ret = alif_se_service_set_run_cfg(&run);
	if (ret != 0) {
		printk("set_run_cfg failed: %d\n", ret);
		return ret;
	}

	ret = alif_se_service_get_last_set_run_cfg(&run);
	if (ret != 0) {
		printk("get_last_set_run_cfg failed: %d\n", ret);
		return ret;
	}
	printk("Cached run profile: power_domains 0x%08x\n", run.power_domains);

	ret = alif_se_service_get_off_cfg(&off);
	if (ret != 0) {
		printk("get_off_cfg failed: %d\n", ret);
		return ret;
	}
	printk("Off profile: power_domains 0x%08x, wakeup_events 0x%08x\n", off.power_domains,
	       off.wakeup_events);

	return 0;
}

int main(void)
{
	int ret;

	printk("Alif SE services sample started\n");

	ret = alif_se_service_sync();
	if (ret != 0) {
		printk("SE sync failed: %d\n", ret);
		return 0;
	}

	ret = alif_se_service_heartbeat();
	if (ret != 0) {
		printk("SE heartbeat failed: %d\n", ret);
		return 0;
	}
	printk("SE is alive\n");

	ret = show_se_info();
	if (ret != 0) {
		return ret;
	}
	ret = show_device_info();
	if (ret != 0) {
		return ret;
	}
	ret = show_random();
	if (ret != 0) {
		return ret;
	}
	ret = show_clocks();
	if (ret != 0) {
		return ret;
	}
	ret = show_profiles();
	if (ret != 0) {
		return ret;
	}

	printk("Resetting SOC in %d seconds\n", RESET_DELAY_SEC);
	k_sleep(K_SECONDS(RESET_DELAY_SEC));

	ret = alif_se_service_boot_reset_soc();
	if (ret != 0) {
		printk("Failed to reset SOC: %d\n", ret);
	}

	return ret;
}
