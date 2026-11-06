# Copyright 2026 Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_CLOCK_MANAGEMENT_NXP_KINETIS_OSC nxp_kinetis_osc.c)
zephyr_library_sources_ifdef(CONFIG_CLOCK_MANAGEMENT_NXP_KINETIS_CLOCK_GATE
        nxp_kinetis_clock_gate.c)

# Header
add_clock_management_header_ifdef(CONFIG_CLOCK_MANAGEMENT_NXP_KINETIS nxp_kinetis_clocks.h)
