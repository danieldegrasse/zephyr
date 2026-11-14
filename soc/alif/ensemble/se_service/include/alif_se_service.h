/*
 * Copyright (c) 2026, Analog Devices Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Definitions taken from alif aipm.h header, which is
 * Copyright (C) 2023 Alif Semiconductor
 */

#ifndef SOC_ALIF_ENSEMBLE_SE_SERVICE_INCLUDE_ALIF_SE_SERVICE_H
#define SOC_ALIF_ENSEMBLE_SE_SERVICE_INCLUDE_ALIF_SE_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

/** Power domain identifiers */
typedef enum {
	ALIF_PD0 = 0,
	ALIF_PD1,
	ALIF_PD2,
	ALIF_PD3,
	ALIF_PD4,
	ALIF_PD5,
	ALIF_PD6,
	ALIF_PD7,
	ALIF_PD8,
	ALIF_PD9
} alif_power_domain_t;

/** Power domain bit masks */
#define ALIF_PD0_MASK (1 << ALIF_PD0)
#define ALIF_PD1_MASK (1 << ALIF_PD1)
#define ALIF_PD2_MASK (1 << ALIF_PD2)
#define ALIF_PD3_MASK (1 << ALIF_PD3)
#define ALIF_PD4_MASK (1 << ALIF_PD4)
#define ALIF_PD5_MASK (1 << ALIF_PD5)
#define ALIF_PD6_MASK (1 << ALIF_PD6)
#define ALIF_PD7_MASK (1 << ALIF_PD7)
#define ALIF_PD8_MASK (1 << ALIF_PD8)
#define ALIF_PD9_MASK (1 << ALIF_PD9)

/** Power domain aliases */
#define ALIF_PD_VBAT_AON_MASK      ALIF_PD0_MASK
#define ALIF_PD_SRAM_CTRL_AON_MASK ALIF_PD1_MASK
#define ALIF_PD_SSE700_AON_MASK    ALIF_PD2_MASK
#define ALIF_PD_RTSS_HE_MASK       ALIF_PD3_MASK
#define ALIF_PD_SRAMS_MASK         ALIF_PD4_MASK
#define ALIF_PD_SESS_MASK          ALIF_PD5_MASK
#define ALIF_PD_SYST_MASK          ALIF_PD6_MASK
#define ALIF_PD_RTSS_HP_MASK       ALIF_PD7_MASK
#define ALIF_PD_DBSS_MASK          ALIF_PD8_MASK
#define ALIF_PD2_APPS_MASK         ALIF_PD9_MASK

/** Low frequency clock sources */
typedef enum {
	ALIF_CLK_SRC_LFRC = 0,
	ALIF_CLK_SRC_LFXO,
} alif_lfclock_t;

/** High frequency clock sources */
typedef enum {
	ALIF_CLK_SRC_HFRC = 0,
	ALIF_CLK_SRC_HFXO,
	ALIF_CLK_SRC_PLL
} alif_hfclock_t;

/** CPU clock frequencies */
typedef enum {
	ALIF_CLOCK_FREQUENCY_800MHZ,        /* Application CPU values */
	ALIF_CLOCK_FREQUENCY_400MHZ,
	ALIF_CLOCK_FREQUENCY_300MHZ,
	ALIF_CLOCK_FREQUENCY_200MHZ,
	ALIF_CLOCK_FREQUENCY_160MHZ,
	ALIF_CLOCK_FREQUENCY_120MHZ,
	ALIF_CLOCK_FREQUENCY_80MHZ,
	ALIF_CLOCK_FREQUENCY_60MHZ,
	ALIF_CLOCK_FREQUENCY_100MHZ,        /* Peripheral clock values */
	ALIF_CLOCK_FREQUENCY_50MHZ,
	ALIF_CLOCK_FREQUENCY_20MHZ,
	ALIF_CLOCK_FREQUENCY_10MHZ,
	ALIF_CLOCK_FREQUENCY_76_8_RC_MHZ,   /* RC and XO clocks */
	ALIF_CLOCK_FREQUENCY_38_4_RC_MHZ,
	ALIF_CLOCK_FREQUENCY_76_8_XO_MHZ,
	ALIF_CLOCK_FREQUENCY_38_4_XO_MHZ,
	ALIF_CLOCK_FREQUENCY_DISABLED
} alif_clock_frequency_t;

/** Scaled HFRC/HFXO clock frequencies */
typedef enum {
	ALIF_SCALED_FREQ_RC_ACTIVE_76_8_MHZ = 0,    /* HFRC frequencies in ACTIVE mode */
	ALIF_SCALED_FREQ_RC_ACTIVE_38_4_MHZ,
	ALIF_SCALED_FREQ_RC_ACTIVE_19_2_MHZ,
	ALIF_SCALED_FREQ_RC_ACTIVE_9_6_MHZ,
	ALIF_SCALED_FREQ_RC_ACTIVE_4_8_MHZ,
	ALIF_SCALED_FREQ_RC_ACTIVE_2_4_MHZ,
	ALIF_SCALED_FREQ_RC_ACTIVE_1_2_MHZ,
	ALIF_SCALED_FREQ_RC_ACTIVE_0_6_MHZ,

	ALIF_SCALED_FREQ_RC_STDBY_76_8_MHZ = 8,     /* HFRC frequencies in STANDBY mode */
	ALIF_SCALED_FREQ_RC_STDBY_38_4_MHZ,
	ALIF_SCALED_FREQ_RC_STDBY_19_2_MHZ,
	ALIF_SCALED_FREQ_RC_STDBY_4_8_MHZ,
	ALIF_SCALED_FREQ_RC_STDBY_1_2_MHZ,
	ALIF_SCALED_FREQ_RC_STDBY_0_6_MHZ,
	ALIF_SCALED_FREQ_RC_STDBY_0_3_MHZ,
	ALIF_SCALED_FREQ_RC_STDBY_0_075_MHZ,

	ALIF_SCALED_FREQ_XO_LOW_DIV_38_4_MHZ = 16,  /* HFXO frequencies, LOW divider */
	ALIF_SCALED_FREQ_XO_LOW_DIV_19_2_MHZ,
	ALIF_SCALED_FREQ_XO_LOW_DIV_9_6_MHZ,
	ALIF_SCALED_FREQ_XO_LOW_DIV_4_8_MHZ,
	ALIF_SCALED_FREQ_XO_LOW_DIV_2_4_MHZ,
	ALIF_SCALED_FREQ_XO_LOW_DIV_1_2_MHZ,
	ALIF_SCALED_FREQ_XO_LOW_DIV_0_6_MHZ,
	ALIF_SCALED_FREQ_XO_LOW_DIV_0_3_MHZ,

	ALIF_SCALED_FREQ_XO_HIGH_DIV_38_4_MHZ = 24, /* HFXO frequencies, HIGH divider */
	ALIF_SCALED_FREQ_XO_HIGH_DIV_19_2_MHZ,
	ALIF_SCALED_FREQ_XO_HIGH_DIV_9_6_MHZ,
	ALIF_SCALED_FREQ_XO_HIGH_DIV_2_4_MHZ,
	ALIF_SCALED_FREQ_XO_HIGH_DIV_0_6_MHZ,
	ALIF_SCALED_FREQ_XO_HIGH_DIV_0_3_MHZ,
	ALIF_SCALED_FREQ_XO_HIGH_DIV_0_15_MHZ,
	ALIF_SCALED_FREQ_XO_HIGH_DIV_0_0375_MHZ,
	ALIF_SCALED_FREQ_NONE
} alif_scaled_clk_freq_t;

/** DCDC converter modes */
typedef enum {
	ALIF_DCDC_MODE_OFF = 0,
	ALIF_DCDC_MODE_PFM_AUTO,
	ALIF_DCDC_MODE_PFM_FORCED,
	ALIF_DCDC_MODE_PWM
} alif_dcdc_mode_t;

/** Flex I/O voltage levels */
typedef enum {
	ALIF_IOFLEX_LEVEL_3V3,
	ALIF_IOFLEX_LEVEL_1V8
} alif_ioflex_mode_t;

/** Clock dividers */
typedef enum {
	ALIF_DIVIDER_CPUPLL,
	ALIF_DIVIDER_SYSPLL,
	ALIF_DIVIDER_ACLK,
	ALIF_DIVIDER_HCLK,
	ALIF_DIVIDER_PCLK
} alif_clock_divider_t;

/** Power settings */
typedef enum {
	ALIF_POWER_SETTING_BOR_EN,
	ALIF_POWER_SETTING_SCALED_CLK_FREQ,
	ALIF_POWER_SETTING_ANA_PERIPH_EN
} alif_power_setting_t;

/** Clock settings that can be queried from the SE */
typedef enum {
	ALIF_CLOCK_SETTING_HFOSC_FREQ,
	ALIF_CLOCK_SETTING_EXTSYS0_FREQ,
	ALIF_CLOCK_SETTING_EXTSYS1_FREQ,
	ALIF_CLOCK_SETTING_AXI_FREQ,
	ALIF_CLOCK_SETTING_AHB_FREQ,
	ALIF_CLOCK_SETTING_APB_FREQ,
	ALIF_CLOCK_SETTING_SYSREF_FREQ,
	ALIF_CLOCK_SETTING_ACLK_FORCE_EN,
	ALIF_CLOCK_SETTING_ACLK_ENTRY_DELAY
} alif_clock_setting_t;
typedef enum {
	NPU_HP,
	NPU_HE,
	ISIM,
	OSPI_1,
	CANFD,
	SDC,
	USB,
	ETH,
	GPU,
	CDC200,
	CAMERA,
	MIPI_DSI,
	MIPI_CSI,
	LP_PERIPH
} ip_clock_gating_t;

#define NPU_HP_MASK       (1 << NPU_HP)   /* bit0 */
#define NPU_HE_MASK       (1 << NPU_HE)   /* bit1 */
#define OSPI_1_MASK       (1 << OSPI_1)   /* bit3 */
#define CANFD_MASK        (1 << CANFD)    /* bit4 */
#define SDC_MASK          (1 << SDC)      /* bit5 */
#define USB_MASK          (1 << USB)      /* bit6 */
#define ETH_MASK          (1 << ETH)      /* bit7 */
#define GPU_MASK          (1 << GPU)      /* bit8 */
#define CDC200_MASK       (1 << CDC200)   /* bit9 */
#define CAMERA_MASK       (1 << CAMERA)   /* bit10 */
#define MIPI_DSI_MASK     (1 << MIPI_DSI) /* bit11 */
#define MIPI_CSI_MASK     (1 << MIPI_CSI) /* bit12 */
#define LP_PERIPH_MASK    (1 << LP_PERIPH)/* bit13 */

typedef enum {
	LDO_PHY,
	USB_PHY,
	MIPI_TX_DPHY,
	MIPI_RX_DPHY,
	MIPI_PLL_DPHY
} phy_gating_t;

#define LDO_PHY_MASK          (1 << LDO_PHY)       /* bit0 */
#define USB_PHY_MASK          (1 << USB_PHY)       /* bit1 */
#define MIPI_TX_DPHY_MASK     (1 << MIPI_TX_DPHY)  /* bit2 */
#define MIPI_RX_DPHY_MASK     (1 << MIPI_RX_DPHY)  /* bit3 */
#define MIPI_PLL_DPHY_MASK    (1 << MIPI_PLL_DPHY) /* bit4 */

/** Run profile */
typedef struct {
	uint32_t power_domains;                 /* Power domains to stay on */
	uint32_t dcdc_voltage;                  /* DCDC output voltage 750-850mv */
	alif_dcdc_mode_t dcdc_mode;             /* DCDC mode - PWM or PFM */
	alif_lfclock_t aon_clk_src;             /* LFRC/LFXO */
	alif_hfclock_t run_clk_src;             /* HFRC/HFXO/PLL */
	alif_clock_frequency_t cpu_clk_freq;    /* APSS/RTSS-HP/RTSS-HE specific setting */
	alif_scaled_clk_freq_t scaled_clk_freq; /* Scaled HFRC/HFXO frequency */
	uint32_t memory_blocks;                 /* Memory blocks to be retained/powered */
	uint32_t ip_clock_gating;               /* IP clock gating */
	uint32_t phy_pwr_gating;                /* PHY power gating */
	alif_ioflex_mode_t vdd_ioflex_3V3;      /* Enable 3.3V GPIOs */
} alif_run_profile_t;

/** Off profile */
typedef struct {
	uint32_t power_domains;                 /* Power domains to stay on */
	uint32_t dcdc_voltage;                  /* DCDC output voltage 750-850mv */
	alif_dcdc_mode_t dcdc_mode;             /* DCDC mode - PWM or PFM */
	alif_lfclock_t aon_clk_src;             /* LFRC/LFXO */
	alif_hfclock_t stby_clk_src;            /* HFRC/HFXO/PLL */
	alif_scaled_clk_freq_t stby_clk_freq;   /* Selected automatically in SoC standby */
	uint32_t memory_blocks;                 /* Memory blocks to be retained/powered */
	uint32_t ip_clock_gating;               /* IP clock gating */
	uint32_t phy_pwr_gating;                /* PHY power gating */
	alif_ioflex_mode_t vdd_ioflex_3V3;      /* Flex GPIOs voltage */
	uint32_t wakeup_events;
	uint32_t ewic_cfg;
	uint32_t vtor_address;
	uint32_t vtor_address_ns;
} alif_off_profile_t;

/** Low power comparator configuration */
typedef struct {
	uint8_t comp_lp0_hyst;
	uint8_t comp_lp0_in_m_sel;
	uint8_t comp_lp0_in_p_sel;
	uint8_t comp_lp_en;
	uint8_t lpcomp_clk32k_en;
	uint8_t lpcomp_clk_sel;
} alif_lpcmp_configure_t;

/** Device revision and identification data */
typedef struct {
	uint32_t revision_id;
	uint8_t ALIF_PN[16];
	uint8_t HBK0[16];
	uint8_t HBK1[16];
	uint8_t HBK_FW[20];
	uint8_t config[4];
	uint8_t DCU[16];
	uint8_t MfgData[32];
	uint8_t SerialN[8];
	uint8_t LCS;
	uint32_t external_config[4];
	uint32_t flags2;
} alif_device_revision_data_t;

/**
 * @brief Send heartbeat to check if SE is alive.
 *
 * @retval 0 Success.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 */
int alif_se_service_heartbeat(void);

/**
 * @brief Synchronize with SE by sending multiple heartbeat requests.
 *
 * Wakes up the SE and waits until it is ready to service requests.
 *
 * @retval 0 Success. SE is ready to service requests.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 */
int alif_se_service_sync(void);

/**
 * @brief Enable or disable SE services debug output.
 *
 * @param debug_enable true to enable debug, false to disable.
 * @retval 0 Success.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_system_set_services_debug(bool debug_enable);

/**
 * @brief Get random number from SE.
 *
 * @param buffer Pointer to buffer to store the random number.
 * @param length Length of the requested random number in bytes.
 * @retval 0 Success.
 * @retval -EINVAL @p buffer is NULL.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_get_rnd_num(uint8_t *buffer, uint16_t length);

/**
 * @brief Get number of table of contents (TOC) entries.
 *
 * @param ptoc Pointer to store the TOC count.
 * @retval 0 Success.
 * @retval -EINVAL @p ptoc is NULL.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_get_toc_number(uint32_t *ptoc);

/**
 * @brief Get SE firmware revision string.
 *
 * @param prev Buffer to store the firmware revision string
 *             (up to VERSION_RESPONSE_LENGTH characters).
 * @retval 0 Success.
 * @retval -EINVAL @p prev is NULL.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_get_se_revision(uint8_t *prev);

/**
 * @brief Get TOC version.
 *
 * The result is cached after the first successful query.
 *
 * @param pversion Pointer to store the TOC version.
 * @retval 0 Success.
 * @retval -EINVAL @p pversion is NULL.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_get_toc_version(uint32_t *pversion);

/**
 * @brief Get device part number.
 *
 * @param pdev_part Pointer to store the device part number.
 * @retval 0 Success.
 * @retval -EINVAL @p pdev_part is NULL.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_get_device_part_number(uint32_t *pdev_part);

/**
 * @brief Get device revision and identification data.
 *
 * On success, @p pdev_data contains SoC revision, part number, keys,
 * firmware version, wounding data, DCU settings, manufacturing data,
 * serial number, and SoC lifecycle state.
 *
 * @param pdev_data Pointer to store the device data.
 * @retval 0 Success.
 * @retval -EINVAL @p pdev_data is NULL.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_system_get_device_data(alif_device_revision_data_t *pdev_data);

/**
 * @brief Calculate unique EUI-48 or EUI-64 extension from manufacturing data.
 *
 * Internally calls alif_se_service_system_get_device_data() and inherits
 * its error codes.
 *
 * @param is_eui48      true for EUI-48 extension, false for EUI-64.
 * @param eui_extension Buffer to store the calculated extension.
 * @retval 0 Success.
 * @retval -EINVAL Invalid argument.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_system_get_eui_extension(bool is_eui48, uint8_t *eui_extension);

/**
 * @brief Send service request to SE to boot ES0.
 *
 * Starts the ES0 core with the given NVDS data and clock configuration.
 * It is encouraged to use the power manager library instead of calling
 * this directly from the application.
 *
 * @param nvds_buff    NVDS configuration defined by user.
 * @param nvds_size    Size of the NVDS data.
 * @param clock_select ES0 UART and main clock selection.
 * @param hpa_mode     If true, HPA mode is used, otherwise LPA mode is used.
 * @retval 0 Success.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_boot_es0(uint8_t *nvds_buff, uint16_t nvds_size, uint32_t clock_select,
			bool hpa_mode);

/**
 * @brief Send service request to SE to shutdown ES0.
 *
 * Shuts down the ES0 core previously started with alif_se_service_boot_es0().
 * It is encouraged to use the power manager library instead of calling
 * this directly from the application.
 *
 * @retval 0 Success.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_shutdown_es0(void);

/**
 * @brief Get current run profile.
 *
 * Queries the SE firmware for the current run profile.
 *
 * @param pp Pointer to alif_run_profile_t to receive the profile data.
 * @retval 0 Success.
 * @retval -EINVAL @p pp is NULL.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_get_run_cfg(alif_run_profile_t *pp);

/**
 * @brief Get the last run configuration set by this core.
 *
 * Returns the locally cached run profile that was set via
 * alif_se_service_set_run_cfg(). Faster than querying SE firmware and
 * suitable for read-modify-write operations.
 *
 * @note The cached profile reflects what this core has set, not the
 * actual SE firmware state. The state may differ if other cores have
 * made changes, the system has been suspended/resumed, or the profile
 * was never initialized on this core.
 *
 * @param pp Pointer to alif_run_profile_t to receive cached data.
 * @retval 0 Success.
 * @retval -EINVAL @p pp is NULL.
 * @retval -ENODATA Profile cache is not initialized.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 */
int alif_se_service_get_last_set_run_cfg(alif_run_profile_t *pp);

/**
 * @brief Set run profile.
 *
 * Sends the run profile to SE. Skips the SE call if the profile
 * has not changed from the cached value.
 *
 * @param pp Pointer to alif_run_profile_t with the desired profile.
 * @retval 0 Success.
 * @retval -EINVAL @p pp is NULL.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_set_run_cfg(alif_run_profile_t *pp);

/**
 * @brief Get current off profile.
 *
 * @param wp Pointer to alif_off_profile_t to receive the profile data.
 * @retval 0 Success.
 * @retval -EINVAL @p wp is NULL.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_get_off_cfg(alif_off_profile_t *wp);

/**
 * @brief Set off profile.
 *
 * @param wp Pointer to alif_off_profile_t with the desired profile.
 * @retval 0 Success.
 * @retval -EINVAL @p wp is NULL.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_set_off_cfg(alif_off_profile_t *wp);

/**
 * @brief Request SE to enter sleep mode.
 *
 * After a successful call, the SE ready flag is cleared so the next
 * SE service call will re-synchronize (wake up) the SE.
 *
 * @param param Sleep parameter passed to SE.
 * @retval 0 Success.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_se_sleep_req(uint32_t param);

/**
 * @brief Request SE to reset the entire SoC.
 *
 * @retval 0 Success.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 */
int alif_se_service_boot_reset_soc(void);

/**
 * @brief Request SE to reset a specific CPU.
 *
 * @param cpu_id Identifier of the CPU to reset.
 * @retval 0 Success.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_boot_reset_cpu(uint32_t cpu_id);

/**
 * @brief Update the ALIF STOC using the provided image.
 *
 * @param img_addr Pointer to the image data.
 * @param img_size Size of the image in bytes (must be non-zero).
 * @retval 0 Success.
 * @retval -EINVAL @p img_addr is NULL or @p img_size is zero.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_update_stoc(uint8_t *img_addr, uint32_t img_size);

/**
 * @brief Send service request to SE to set the value of a clock divider.
 *
 * Selects the value of a clock divider.
 *
 * @param divider Which divider to set – CPUPLL, SYSPLL, ACLK (Corstone), HCLK, PCLK (Alif).
 * @param value Divider value. 0x0 to 0x1F for Corstone dividers, 0x0 to 0x2 for Alif dividers.
 * @retval 0 Success.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_clock_set_divider(alif_clock_divider_t divider, uint32_t value);

/**
 * @brief Send service request to SE to get a clock frequency setting.
 *
 * Queries the actual clock frequency configured by SE firmware for the
 * given setting. Use this instead of hardcoded values, since SE run
 * profile changes affect these frequencies.
 *
 * @param setting Clock setting to query.
 * @param freq Pointer to store frequency value in Hz.
 * @retval 0 Success.
 * @retval -EINVAL Invalid argument.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_clock_setting_get(alif_clock_setting_t setting, uint32_t *freq);

/**
 * @brief Send service request to SE to process TOC entry.
 *
 * @param image_id ID that matches the TOC entry field image identifier.
 * @retval 0 Success.
 * @retval -EINVAL Invalid argument.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_process_toc_entry(const char *image_id);

/**
 * @brief Read an OTP word specified by an offset.
 *
 * @param otp_offset An offset position to read OTP from.
 * @param otp_word   Pointer to store OTP value.
 * @retval 0 Success.
 * @retval -EINVAL @p otp_word is NULL.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_read_otp(uint32_t otp_offset, uint32_t *otp_word);

/**
 * @brief Enable a power domain via SE service run configuration.
 *
 * Performs a read-modify-write on the last set run profile to enable the
 * specified power domain. Safe to call before the power domain driver is
 * initialized (e.g. from a PM notifier pre_device_resume callback).
 *
 * @param pd_id Power domain bit index (ALIF_PD_* from alif_power_domain.h)
 * @retval 0 on success
 * @retval -EINVAL Invalid pd_id (must be < 32).
 * @retval -ENODATA Profile cache is not initialized.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_enable_pd(uint32_t pd_id);

/**
 * @brief Power PHYs on or off via SE service run configuration.
 *
 * Performs a read-modify-write on the last set run profile to set or clear
 * @p mask in the phy_pwr_gating field. The update is serialized with all
 * other SE service calls.
 *
 * @param mask   PHY bit mask (LDO_PHY_MASK, USB_PHY_MASK, MIPI_*_DPHY_MASK).
 * @param enable true to power the PHYs on, false to power them off.
 * @retval 0 Success.
 * @retval -ENODATA Profile cache is not initialized.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @retval -EBUSY SE is busy. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_update_phy_pwr_gating(uint32_t mask, bool enable);

/**
 * @brief Boot a CPU via SE service.
 *
 * Boots the specified CPU with the given entry point address. The address
 * must be an executable image that is properly set up for the target CPU.
 *
 * @param cpu_id Identifier of the CPU to boot.
 * @param address Entry point address to boot the CPU from.
 * @retval 0 Success.
 * @retval -EINVAL Invalid argument.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_boot_cpu(uint32_t cpu_id, uint32_t address);

/**
 * @brief Configure the LPCMP service via SE.
 *
 * @param config Pointer to the LPCMP configuration structure.
 *
 * @retval 0 Success.
 * @retval -EINVAL Invalid argument.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_configure_lpcmp(const alif_lpcmp_configure_t *config);

/**
 * @brief Set a power setting via SE service.
 *
 * @param setting The type of power setting to set.
 * @param value The value to set for the specified power setting.
 *
 * @retval 0 Success.
 * @retval -EINVAL Invalid argument.
 * @retval -EAGAIN Operation timed out. Retry after a delay.
 * @return Positive error code returned by SE for a failed service request.
 */
int alif_se_service_power_settings_set(alif_power_setting_t setting, uint32_t value);

#ifdef __cplusplus
}
#endif
#endif /* SOC_ALIF_ENSEMBLE_alif_se_service_H_ALIF_alif_se_service_H_ */
