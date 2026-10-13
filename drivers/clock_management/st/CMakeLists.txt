# Copyright 2026 Analog Devices Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_CLOCK_MANAGEMENT_STM32_CLOCK_SOURCE clock_source_stm32.c)

# Header
add_clock_management_header_ifdef(CONFIG_CLOCK_MANAGEMENT_STM32 stm32_clocks.h)
