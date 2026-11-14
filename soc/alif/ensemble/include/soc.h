/*
 * Copyright (c) 2026, Analog Devices Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC__H_
#define _SOC__H_

#ifdef CONFIG_SOC_FAMILY_ENSEMBLE_RTSS
#include "../rtss/soc.h"
#endif

#ifndef _ASMLANGUAGE

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/devicetree.h>
#include <zephyr/sys/sys_io.h>

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(itcm)) &&                                                 \
	DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(itcm_backdoor))
#define _SOC_ALIF_ITCM_BASE          DT_REG_ADDR(DT_NODELABEL(itcm))
#define _SOC_ALIF_ITCM_BACKDOOR_BASE DT_REG_ADDR(DT_NODELABEL(itcm_backdoor))
#define _SOC_ALIF_ITCM_SIZE          DT_REG_SIZE(DT_NODELABEL(itcm))
#define _SOC_ALIF_ITCM_BACKDOOR_SIZE DT_REG_SIZE(DT_NODELABEL(itcm_backdoor))
#define _SOC_ALIF_IN_ITCM(addr)                                                                    \
	((addr) >= _SOC_ALIF_ITCM_BASE && (addr) < (_SOC_ALIF_ITCM_BASE + _SOC_ALIF_ITCM_SIZE))
#define _SOC_ALIF_ITCM_REMAP_OFFSET (_SOC_ALIF_ITCM_BACKDOOR_BASE - _SOC_ALIF_ITCM_BASE)
BUILD_ASSERT(_SOC_ALIF_ITCM_SIZE == _SOC_ALIF_ITCM_BACKDOOR_SIZE,
	     "ITCM and ITCM_BACKDOOR sizes must match");
#else
#define _SOC_ALIF_IN_ITCM(addr)     (0)
#define _SOC_ALIF_ITCM_REMAP_OFFSET (0)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(dtcm)) &&                                                 \
	DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(dtcm_backdoor))
#define _SOC_ALIF_DTCM_BASE          DT_REG_ADDR(DT_NODELABEL(dtcm))
#define _SOC_ALIF_DTCM_BACKDOOR_BASE DT_REG_ADDR(DT_NODELABEL(dtcm_backdoor))
#define _SOC_ALIF_DTCM_SIZE          DT_REG_SIZE(DT_NODELABEL(dtcm))
#define _SOC_ALIF_DTCM_BACKDOOR_SIZE DT_REG_SIZE(DT_NODELABEL(dtcm_backdoor))
#define _SOC_ALIF_IN_DTCM(addr)                                                                    \
	((addr) >= _SOC_ALIF_DTCM_BASE && (addr) < (_SOC_ALIF_DTCM_BASE + _SOC_ALIF_DTCM_SIZE))
#define _SOC_ALIF_DTCM_REMAP_OFFSET (_SOC_ALIF_DTCM_BACKDOOR_BASE - _SOC_ALIF_DTCM_BASE)
BUILD_ASSERT(_SOC_ALIF_DTCM_SIZE == _SOC_ALIF_DTCM_BACKDOOR_SIZE,
	     "DTCM and DTCM_BACKDOOR sizes must match");
#else
#define _SOC_ALIF_IN_DTCM(addr)     (0)
#define _SOC_ALIF_DTCM_REMAP_OFFSET (0)
#endif

static inline mem_addr_t ensemble_tcm_to_global_addr(mem_addr_t local)
{
	if (_SOC_ALIF_IN_ITCM(local)) {
		return local + _SOC_ALIF_ITCM_REMAP_OFFSET;
	}
	if (_SOC_ALIF_IN_DTCM(local)) {
		return local + _SOC_ALIF_DTCM_REMAP_OFFSET;
	}
	return local;
}

#ifdef __cplusplus
}
#endif

#endif /* !_ASMLANGUAGE */

#endif /* _SOC__H_ */
