/*
 * Copyright (c) 2026, Analog Devices Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Definitions taken from alif services_lib_protocol.h header, which is
 * Copyright (C) 2024 Alif Semiconductor
 */

#ifndef SOC_ALIF_ENSEMBLE_SE_SERVICE_SRC_SE_PROTOCOL_H
#define SOC_ALIF_ENSEMBLE_SE_SERVICE_SRC_SE_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define IMAGE_NAME_LENGTH       8
#define VERSION_RESPONSE_LENGTH 80

/*
 * Extsys0 configuration field.
 *
 * HPA: DCDC configuration for HPA mode.
 * CSP: DCDC configuration for CSP package.
 * default: PGA and LPA
 */
#define SERVICES_NET_PROC_BOOT_CONFIGURATION_NONE 0x0
#define SERVICES_NET_PROC_BOOT_CONFIGURATION_HPA  0x1
#define SERVICES_NET_PROC_BOOT_CONFIGURATION_CSP  0x2

/**
 * @struct service_header_t
 * @brief  Common packet for all SERVICEs
 */
typedef struct {
	volatile uint16_t hdr_service_id; /* Requested Service ID */
	volatile uint16_t hdr_flags;      /* Request flags */
	volatile uint16_t hdr_error_code; /* Transport layer error code */
	volatile uint16_t hdr_padding;
} service_header_t;

/* SE Deep Sleep */
typedef struct {
	service_header_t header;
	volatile uint32_t  send_param;
	volatile uint32_t  resp_error_code;
} se_sleep_svc_t;

/* AI PM APIs */

/**
 * @struct  aipm_set_run_profile_svc_t
 * @brief aipm run set profile service packet
 */
typedef struct {
	service_header_t header;
	volatile uint32_t send_power_domains;
	volatile uint32_t send_dcdc_voltage;
	volatile uint32_t send_dcdc_mode;
	volatile uint32_t send_aon_clk_src;
	volatile uint32_t send_run_clk_src;
	volatile uint32_t send_cpu_clk_freq;
	volatile uint32_t send_scaled_clk_freq;
	volatile uint32_t send_memory_blocks;
	volatile uint32_t send_ip_clock_gating;
	volatile uint32_t send_phy_pwr_gating;
	volatile uint32_t send_vdd_ioflex_3V3;
	volatile int      resp_error_code;
} aipm_set_run_profile_svc_t;

/**
 * @struct aipm_get_run_profile_svc_t
 * @brief aipm run get profile service packet
 */
typedef struct {
	service_header_t header;
	volatile uint32_t resp_power_domains;
	volatile uint32_t resp_dcdc_voltage;
	volatile uint32_t resp_dcdc_mode;
	volatile uint32_t resp_aon_clk_src;
	volatile uint32_t resp_run_clk_src;
	volatile uint32_t resp_cpu_clk_freq;
	volatile uint32_t resp_scaled_clk_freq;
	volatile uint32_t resp_memory_blocks;
	volatile uint32_t resp_ip_clock_gating;
	volatile uint32_t resp_phy_pwr_gating;
	volatile uint32_t resp_vdd_ioflex_3V3;
	volatile int      resp_error_code;
} aipm_get_run_profile_svc_t;

/**
 * @struct aipm_set_off_profile_svc_t
 * @brief aipm off set profile service packet
 */
typedef struct {
	service_header_t header;
	volatile uint32_t send_power_domains;
	volatile uint32_t send_dcdc_voltage;
	volatile uint32_t send_dcdc_mode;
	volatile uint32_t send_aon_clk_src;
	volatile uint32_t send_stby_clk_src;
	volatile uint32_t send_stby_clk_freq;
	volatile uint32_t send_memory_blocks;
	volatile uint32_t send_ip_clock_gating;
	volatile uint32_t send_phy_pwr_gating;
	volatile uint32_t send_vdd_ioflex_3V3;
	volatile uint32_t send_wakeup_events;
	volatile uint32_t send_ewic_cfg;
	volatile uint32_t send_vtor_address;
	volatile uint32_t send_vtor_address_ns;
	volatile int      resp_error_code;
} aipm_set_off_profile_svc_t;

/**
 * @struct aipm_get_off_profile_svc_t
 * @brief aipm off get profile service packet
 */
typedef struct {
	service_header_t header;
	volatile uint32_t resp_power_domains;
	volatile uint32_t resp_dcdc_voltage;
	volatile uint32_t resp_dcdc_mode;
	volatile uint32_t resp_aon_clk_src;
	volatile uint32_t resp_stby_clk_src;
	volatile uint32_t resp_stby_clk_freq;
	volatile uint32_t resp_memory_blocks;
	volatile uint32_t resp_ip_clock_gating;
	volatile uint32_t resp_phy_pwr_gating;
	volatile uint32_t resp_vdd_ioflex_3V3;
	volatile uint32_t resp_wakeup_events;
	volatile uint32_t resp_ewic_cfg;
	volatile uint32_t resp_vtor_address;
	volatile uint32_t resp_vtor_address_ns;
	volatile int      resp_error_code;
} aipm_get_off_profile_svc_t;

/* Extsys0 services */
typedef struct {
	service_header_t header;
	volatile uint32_t send_nvds_src_addr;
	volatile uint32_t send_nvds_dst_addr;
	volatile uint32_t send_nvds_copy_len;
	volatile uint32_t send_trng_dst_addr;
	volatile uint32_t send_trng_len;
	volatile uint32_t send_internal_clock_select;
	volatile uint32_t send_configuration;
	volatile int      resp_error_code;
} net_proc_boot_svc_t;

typedef struct {
	service_header_t header;
	volatile int     resp_error_code;
} net_proc_shutdown_svc_t;

/* Crypto Services */

/* Get Random Data */

/* MBEDTLS_CTR_DRBG_MAX_REQUEST in cryptocell-rt is 1024, */
/* it was decided that we will use a smaller buffer */
#define MAX_RND_LENGTH 256
typedef struct {
	service_header_t header;
	volatile uint32_t  send_rnd_length;
	volatile uint8_t   resp_rnd[MAX_RND_LENGTH];
	volatile int       resp_error_code;
} get_rnd_svc_t;

/* Boot Services */

/**
 * @struct process_toc_entry_svc_t
 * @brief  Get TOC entry data
 */
typedef struct {
	service_header_t header;
	volatile uint8_t   send_entry_id[IMAGE_NAME_LENGTH];
	volatile uint32_t  resp_error_code;
} process_toc_entry_svc_t;

/**
 * @struct boot_cpu_svc_t
 * @brief  Boot CPU packet
 */
typedef struct {
	service_header_t header;
	volatile uint32_t  send_cpu_id;
	volatile uint32_t  send_address;
	volatile uint32_t  resp_error_code;
} boot_cpu_svc_t;

/* Control CPU */
typedef struct {
	service_header_t header;
	volatile uint32_t  send_cpu_id;
	volatile uint32_t  resp_error_code;
} control_cpu_svc_t;

/**
 * @struct get_se_revision_t
 * @brief  Retrieve SERAM version banner
 */
typedef struct {
	service_header_t header;
	volatile uint32_t  resp_se_revision_length;
	volatile uint8_t   resp_se_revision[VERSION_RESPONSE_LENGTH];
	volatile uint32_t  resp_error_code;
} get_se_revision_t;

/**
 * System Management Services
 */

/**
 * @struct get_toc_version_svc_t
 * @brief  Get TOC Version
 */
typedef struct {
	service_header_t header;
	volatile uint32_t  resp_version;
	volatile uint32_t  resp_error_code;
} get_toc_version_svc_t;

/**
 * @struct get_toc_number_svc_t
 * @brief  Get TOC Number
 */
typedef struct {
	service_header_t header;
	volatile  uint32_t  resp_number_of_toc;
	volatile uint32_t  resp_error_code;
} get_toc_number_svc_t;

/**
 * @struct  otp_data_t
 * @brief   request for otp read or write
 */
typedef struct {
	service_header_t header;
	volatile uint32_t send_offset;     /**< OTP offset to read or write */
	volatile uint32_t otp_word;        /**< OTP contents       */
	volatile uint32_t resp_error_code;
} otp_data_t;

/**
 * @struct get_device_part_svc_t
 * @brief  Get Device Part
 */
typedef struct {
	service_header_t header;
	volatile uint32_t  resp_device_string;
	volatile uint32_t  resp_error_code;
} get_device_part_svc_t;

/**
 * @struct  get_device_revision_data_t
 * @brief   Get all relevant device information
 */
typedef struct {
	service_header_t header;
	volatile uint32_t revision_id;
	volatile uint8_t resp_version[4];
	volatile uint8_t ALIF_PN[16];
	volatile uint8_t HBK0[16];
	volatile uint8_t HBK1[16];
	volatile uint8_t HBK_FW[20];
	volatile uint8_t config[4];
	volatile uint8_t DCU[16];
	volatile uint8_t MfgData[32];
	volatile uint8_t SerialN[8];
	volatile uint8_t LCS;
	volatile uint8_t padding[3];
	volatile uint32_t external_config[4];
	volatile uint32_t flags2;
	volatile uint32_t resp_error_code;
} get_device_revision_data_t;

/**
 * @struct set_services_capabilities_t
 * @brief Set capabilities
 * @note for now it is limited to debug toggle
 */
typedef struct {
	service_header_t   header;
	volatile bool      send_services_debug;
	volatile uint32_t  resp_error_code;
} set_services_capabilities_t;

/* Clocks API */

/* struct for setting clock dividers */
typedef struct {
	service_header_t header;
	uint32_t send_divider;
	uint32_t send_value;
	uint32_t resp_error_code;
} clk_set_clk_divider_svc_t;

/* struct for updating the STOC */
typedef struct {
	service_header_t header;
	uint32_t send_image_address;
	uint32_t send_image_size;
	uint32_t resp_error_code;
} update_stoc_svc_t;

/* Power Get/Configure API */
typedef struct {
	service_header_t header;
	volatile uint32_t send_setting_type;
	volatile uint32_t value;
	volatile uint32_t resp_error_code;
} power_setting_svc_t;

/* Clock Setting Get API */
typedef struct {
	service_header_t header;
	volatile uint32_t send_setting_type;
	volatile uint32_t value;
	volatile uint32_t resp_error_code;
} clock_setting_svc_t;

/* Low Power Comparator Configuration */
typedef struct {
	service_header_t header;
	volatile uint8_t comp_lp0_hyst;
	volatile uint8_t comp_lp0_in_m_sel;
	volatile uint8_t comp_lp0_in_p_sel;
	volatile uint8_t comp_lp_en;
	volatile uint8_t lpcomp_clk32k_en;
	volatile uint8_t lpcomp_clk_sel;
	volatile uint32_t resp_error_code;
} lp_cmp_configure_svc_t;

#ifdef __cplusplus
}
#endif

#endif /* SOC_ALIF_ENSEMBLE_SE_SERVICE_SRC_SE_PROTOCOL_H */
