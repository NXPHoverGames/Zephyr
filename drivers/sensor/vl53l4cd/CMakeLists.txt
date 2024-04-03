# Copyright 2024 NXP
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
	vl53l4cd.c
	vl53l4cd_api.c
	platform.c
	vl53l4cd_calibration.c
)
