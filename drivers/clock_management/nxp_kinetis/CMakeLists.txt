# Copyright 2026 Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_CLOCK_MANAGEMENT_NXP_KINETIS_OSC nxp_kinetis_osc.c)

# Header
add_clock_management_header_ifdef(CONFIG_CLOCK_MANAGEMENT_NXP_KINETIS nxp_kinetis_clocks.h)
