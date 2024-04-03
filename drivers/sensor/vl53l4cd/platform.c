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

#include "platform.h"

enum VL53L4CD_Error VL53L4CD_RdDWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint32_t *value)
{
	int ret;
	uint8_t register_addr_sz = sizeof(reg_addr), value_sz = sizeof(*value);

	reg_addr = BSWAP_16(reg_addr);

	ret = i2c_write_read(dev->i2c_bus, dev->i2c_addr, &reg_addr, register_addr_sz,
			     value, value_sz);
	if (ret < 0) {
		return VL53L4CD_ERROR_XTALK_FAILED;
	}

	*value = BSWAP_32(*value);

	return VL53L4CD_ERROR_NONE;
}

enum VL53L4CD_Error VL53L4CD_RdWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint16_t *value)
{
	int ret;
	uint8_t register_addr_sz = sizeof(reg_addr), value_sz = sizeof(*value);

	reg_addr = BSWAP_16(reg_addr);

	ret = i2c_write_read(dev->i2c_bus, dev->i2c_addr, &reg_addr, register_addr_sz,
			     value, value_sz);
	if (ret < 0) {
		return VL53L4CD_ERROR_XTALK_FAILED;
	}

	*value = BSWAP_16(*value);

	return VL53L4CD_ERROR_NONE;
}

enum VL53L4CD_Error VL53L4CD_RdByte(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint8_t *value)
{
	int ret;
	uint8_t register_addr_sz = sizeof(reg_addr), value_sz = sizeof(*value);

	reg_addr = BSWAP_16(reg_addr);
	ret = i2c_write_read(dev->i2c_bus, dev->i2c_addr, &reg_addr, register_addr_sz,
			     value, value_sz);
	if (ret < 0) {
		return VL53L4CD_ERROR_XTALK_FAILED;
	}

	return VL53L4CD_ERROR_NONE;
}

enum VL53L4CD_Error VL53L4CD_WrByte(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint8_t value)
{
	int ret;
	uint8_t register_addr_sz = sizeof(reg_addr), value_sz = sizeof(value);
	uint8_t buf_size = register_addr_sz + value_sz, buf[buf_size];

	reg_addr = BSWAP_16(reg_addr);
	memcpy(buf, &reg_addr, register_addr_sz);
	memcpy(buf + register_addr_sz, &value, value_sz);

	ret = i2c_write(dev->i2c_bus, buf, 3, dev->i2c_addr);
	if (ret < 0) {
		return VL53L4CD_ERROR_XTALK_FAILED;
	}

	return VL53L4CD_ERROR_NONE;
}

enum VL53L4CD_Error VL53L4CD_WrWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint16_t value)
{
	int ret;
	uint8_t register_addr_sz = sizeof(reg_addr), value_sz = sizeof(value);
	uint8_t buf_size = register_addr_sz + value_sz, buf[buf_size];

	reg_addr = BSWAP_16(reg_addr);
	value = BSWAP_16(value);
	memcpy(buf, &reg_addr, register_addr_sz);
	memcpy(buf + register_addr_sz, &value, value_sz);

	ret = i2c_write(dev->i2c_bus, buf, buf_size, dev->i2c_addr);
	if (ret < 0) {
		return VL53L4CD_ERROR_XTALK_FAILED;
	}

	return VL53L4CD_ERROR_NONE;
}

enum VL53L4CD_Error VL53L4CD_WrDWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint32_t value)
{
	int ret;
	uint8_t register_addr_sz = sizeof(reg_addr), value_sz = sizeof(value);
	uint8_t buf_size = register_addr_sz + value_sz, buf[buf_size];

	reg_addr = BSWAP_16(reg_addr);
	value = BSWAP_32(value);
	memcpy(buf, &reg_addr, register_addr_sz);
	memcpy(buf + register_addr_sz, &value, value_sz);

	ret = i2c_write(dev->i2c_bus, buf, buf_size, dev->i2c_addr);
	if (ret < 0) {
		return VL53L4CD_ERROR_XTALK_FAILED;
	}

	return VL53L4CD_ERROR_NONE;
}

enum VL53L4CD_Error VL53L4CD_PollingDelay(struct VL53L4CD_Dev *dev, uint32_t TimeMs)
{
	int ret;

	ret = k_sleep(K_MSEC(TimeMs));
	if (ret != 0) {
		return VL53L4CD_ERROR_TIMEOUT;
	}

	return VL53L4CD_ERROR_NONE;
}
