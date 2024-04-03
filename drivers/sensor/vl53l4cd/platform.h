/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * This file was referred from the ULD API of VL53L4CD by STMicroelectronics.
 * Link: https://www.st.com/en/embedded-software/stsw-img026.html#get-software.
 */

/*
 Copyright (c) 2021, STMicroelectronics - All Rights Reserved

 This file : part of VL53L4CD Ultra Lite Driver and : dual licensed, either
 'STMicroelectronics Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

*******************************************************************************

 'STMicroelectronics Proprietary license'

*******************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document : strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.


*******************************************************************************

 Alternatively, VL53L4CD Ultra Lite Driver may be distributed under the terms of
 'BSD 3-clause "New" or "Revised" License', in which case the following
 provisions apply instead of the ones mentioned above :

*******************************************************************************

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************
*/

#ifndef ZEPHYR_DRIVERS_SENSOR_VL53L4CD_VL53L4CD_PLATFORM_H_
#define ZEPHYR_DRIVERS_SENSOR_VL53L4CD_VL53L4CD_PLATFORM_H_

#pragma once

#include <stdint.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

/**
 * @struct VL53L4CD_Dev
 * @brief  Generic PAL device type that links API and platform abstraction layer.
 */

struct VL53L4CD_Dev {
	const struct device *i2c_bus;
	uint8_t i2c_addr;
};

/**
 *  @brief Driver error type
 */

enum VL53L4CD_Error {
	VL53L4CD_ERROR_NONE = 0U,
	VL53L4CD_ERROR_XTALK_FAILED = 253U,
	VL53L4CD_ERROR_INVALID_ARGUMENT = 254U,
	VL53L4CD_ERROR_TIMEOUT = 255U,
};

/**
 * @brief If the macro below is defined, the device will be programmed to run
 * with I2C Fast Mode Plus (up to 1MHz). Otherwise, default max value is 400kHz.
 */

/* #define VL53L4CD_I2C_FAST_MODE_PLUS */

/**
 * @brief Read 32 bits through I2C.
 */

enum VL53L4CD_Error VL53L4CD_RdDWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint32_t *value);

/**
 * @brief Read 16 bits through I2C.
 */

enum VL53L4CD_Error VL53L4CD_RdWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint16_t *value);

/**
 * @brief Read 8 bits through I2C.
 */

enum VL53L4CD_Error VL53L4CD_RdByte(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint8_t *value);

/**
 * @brief Write 8 bits through I2C.
 */

enum VL53L4CD_Error VL53L4CD_WrByte(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint8_t value);

/**
 * @brief Write 16 bits through I2C.
 */

enum VL53L4CD_Error VL53L4CD_WrWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint16_t value);

/**
 * @brief Write 32 bits through I2C.
 */

enum VL53L4CD_Error VL53L4CD_WrDWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint32_t value);

/**
 * @brief Wait during N milliseconds.
 */

enum VL53L4CD_Error VL53L4CD_PollingDelay(struct VL53L4CD_Dev *dev, uint32_t TimeMs);

#endif  /* ZEPHYR_DRIVERS_SENSOR_VL53L4CD_VL53L4CD_PLATFORM_H_ */
