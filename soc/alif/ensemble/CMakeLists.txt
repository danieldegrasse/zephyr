# Copyright (c) 2025 Alif Semiconductor
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_include_directories(../common)
zephyr_include_directories(include)

# Controller-mode setup for the DesignWare SSI instances
zephyr_sources_ifdef(CONFIG_SPI_DW ssi.c)

# Cluster-specific code for Cortex-M55 RTSS cores
if(CONFIG_SOC_FAMILY_ENSEMBLE_RTSS)
  add_subdirectory(rtss)
endif()

# Cluster-specific code for Cortex-A32 APSS cores
if(CONFIG_SOC_FAMILY_ENSEMBLE_APSS)
  add_subdirectory(apss)
endif()

if(CONFIG_ALIF_SE_SERVICE)
  add_subdirectory(se_service)
endif()

add_subdirectory(${CONFIG_SOC_SERIES})
