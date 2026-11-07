# Copyright (c) 2026 Analog Devices
# SPDX-License-Identifier: Apache-2.0

# Suppress overlaps to handle clock control DTSI
list(APPEND EXTRA_DTC_FLAGS "-Wno-unique_unit_address_if_enabled")
