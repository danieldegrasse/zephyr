# Copyright 2026 Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_CLOCK_MANAGEMENT_NXP_KINETIS_OSC nxp_kinetis_osc.c)
zephyr_library_sources_ifdef(CONFIG_CLOCK_MANAGEMENT_NXP_KINETIS_CLOCK_GATE
        nxp_kinetis_clock_gate.c)
zephyr_library_sources_ifdef(CONFIG_CLOCK_MANAGEMENT_NXP_KINETIS_RTC_OSC nxp_kinetis_rtc_osc.c)
zephyr_library_sources_ifdef(CONFIG_CLOCK_MANAGEMENT_NXP_KINETIS_RTC_CLKOUT
        nxp_kinetis_rtc_clkout.c)
zephyr_library_sources_ifdef(CONFIG_CLOCK_MANAGEMENT_NXP_KINETIS_MCG_FCRDIV
        nxp_kinetis_mcg_fcrdiv.c)
zephyr_library_sources_ifdef(CONFIG_CLOCK_MANAGEMENT_NXP_KINETIS_MCG_MUX nxp_kinetis_mcg_mux.c)
zephyr_library_sources_ifdef(CONFIG_CLOCK_MANAGEMENT_NXP_KINETIS_MCG_CLKS nxp_kinetis_mcg_clks.c)
zephyr_library_sources_ifdef(CONFIG_CLOCK_MANAGEMENT_NXP_KINETIS_MCG_FRDIV
        nxp_kinetis_mcg_frdiv.c)

# Header
add_clock_management_header_ifdef(CONFIG_CLOCK_MANAGEMENT_NXP_KINETIS nxp_kinetis_clocks.h)
