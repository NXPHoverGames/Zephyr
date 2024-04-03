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

#ifndef ZEPHYR_DRIVERS_SENSOR_VL53L4CD_VL53L4CD_API_H_
#define ZEPHYR_DRIVERS_SENSOR_VL53L4CD_VL53L4CD_API_H_

#include "platform.h"

/**
 *  @brief Driver version
 */

#define VL53L4CD_IMPLEMENTATION_VER_MAJOR	2
#define VL53L4CD_IMPLEMENTATION_VER_MINOR	1
#define VL53L4CD_IMPLEMENTATION_VER_BUILD	1
#define VL53L4CD_IMPLEMENTATION_VER_REVISION	0

/**
 *  @brief Inner Macro for API. Not for user, only for development.
 */

#define VL53L4CD_SOFT_RESET				((uint16_t)0x0000)
#define VL53L4CD_I2C_SLAVE__DEVICE_ADDRESS		((uint16_t)0x0001)
#define VL53L4CD_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND	((uint16_t)0x0008)
#define VL53L4CD_XTALK_PLANE_OFFSET_KCPS		((uint16_t)0x0016)
#define VL53L4CD_XTALK_X_PLANE_GRADIENT_KCPS		((uint16_t)0x0018)
#define VL53L4CD_XTALK_Y_PLANE_GRADIENT_KCPS		((uint16_t)0x001A)
#define VL53L4CD_RANGE_OFFSET_MM			((uint16_t)0x001E)
#define VL53L4CD_INNER_OFFSET_MM			((uint16_t)0x0020)
#define VL53L4CD_OUTER_OFFSET_MM			((uint16_t)0x0022)
#define VL53L4CD_GPIO_HV_MUX__CTRL			((uint16_t)0x0030)
#define VL53L4CD_GPIO__TIO_HV_STATUS			((uint16_t)0x0031)
#define VL53L4CD_SYSTEM__INTERRUPT			((uint16_t)0x0046)
#define VL53L4CD_RANGE_CONFIG_A				((uint16_t)0x005E)
#define VL53L4CD_RANGE_CONFIG_B				((uint16_t)0x0061)
#define VL53L4CD_RANGE_CONFIG__SIGMA_THRESH		((uint16_t)0x0064)
#define VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS		((uint16_t)0x0066)
#define VL53L4CD_INTERMEASUREMENT_MS			((uint16_t)0x006C)
#define VL53L4CD_THRESH_HIGH				((uint16_t)0x0072)
#define VL53L4CD_THRESH_LOW				((uint16_t)0x0074)
#define VL53L4CD_SYSTEM__INTERRUPT_CLEAR		((uint16_t)0x0086)
#define VL53L4CD_SYSTEM_START				((uint16_t)0x0087)
#define VL53L4CD_RESULT__RANGE_STATUS			((uint16_t)0x0089)
#define VL53L4CD_RESULT__SPAD_NB			((uint16_t)0x008C)
#define VL53L4CD_RESULT__SIGNAL_RATE			((uint16_t)0x008E)
#define VL53L4CD_RESULT__AMBIENT_RATE			((uint16_t)0x0090)
#define VL53L4CD_RESULT__SIGMA				((uint16_t)0x0092)
#define VL53L4CD_RESULT__DISTANCE			((uint16_t)0x0096)

#define VL53L4CD_RESULT__OSC_CALIBRATE_VAL	((uint16_t)0x00DE)
#define VL53L4CD_FIRMWARE__SYSTEM_STATUS	((uint16_t)0x00E5)
#define VL53L4CD_IDENTIFICATION__MODEL_ID	((uint16_t)0x010F)

#define VL53L4CD_DATA_READY			1
#define VL53L4CD_DATA_NOT_READY			0

/**
 *  @brief defines Software Version
 */

struct VL53L4CD_SW_Version {
	uint8_t major;		/* !< major number */
	uint8_t minor;		/* !< minor number */
	uint8_t build;		/* !< build number */
	uint32_t revision;	/* !< revision number */
};

/**
 *  @brief Packed reading results type
 */

struct VL53L4CD_ResultsData {
	uint8_t range_status;		/* Status of measurements; if status == 0, data are valid */
	uint16_t distance_mm;		/* Measured distance in millimeters */
	uint16_t ambient_rate_kcps;	/* Ambient noise in kcps */
	uint16_t ambient_per_spad_kcps;	/* Ambient noise in kcps/SPAD */
	uint16_t signal_rate_kcps;	/* Measured signal of the target in kcps */
	uint16_t signal_per_spad_kcps;	/* Measured signal of the target in kcps/SPAD */
	uint16_t number_of_spad;	/* Number of SPADs enabled */
	uint16_t sigma_mm;		/* Estimated measurements std deviation in mm */
};

/**
 * @brief This function programs the software driver version.
 * @param (VL53L4CD_Version_t) pVersion : Pointer of structure, containing the
 * software version.
 * @return (enum VL53L4CD_Error) status : 0 if SW version is OK.
 */

enum VL53L4CD_Error VL53L4CD_GetSWVersion(
		struct VL53L4CD_SW_Version *pVersion);

/**
 * @brief This function sets a new I2C address to a sensor. It can be used for
 * example when multiple sensors share the same I2C bus.
 * @param (Dev_t) dev : Device instance to update.
 * @param (uint8_t) new_address : New I2C address.
 * @return (enum VL53L4CD_Error) status : 0 if I2C address has been correctly
 * programmed.
 */

enum VL53L4CD_Error VL53L4CD_SetI2CAddress(
		struct VL53L4CD_Dev *dev,
		uint8_t new_address);

/**
 * @brief This function is used to get the sensor id of VL53L4CD. The sensor id
 * should be 0xEBAA.
 * @param (Dev_t) dev : Device instance.
 * @param (uint16_t) *p_id : Sensor id.
 * @return (enum VL53L4CD_Error) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_GetSensorId(
		struct VL53L4CD_Dev *dev,
		uint16_t *p_id);

/**
 * @brief This function is used to initialize the sensor.
 * @param (Dev_t) dev : Device instance to initialize.
 * @return (enum VL53L4CD_Error) status : 0 if init is OK.
 */

enum VL53L4CD_Error VL53L4CD_SensorInit(
		struct VL53L4CD_Dev *dev);

/**
 * @brief This function clears the interrupt. It needs to be called after a
 * ranging data reading to arm the interrupt for the next data ready event.
 * @param (Dev_t) dev : Device instance.
 * @return (enum VL53L4CD_Error) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_ClearInterrupt(
		struct VL53L4CD_Dev *dev);

/**
 * @brief This function starts a ranging session. The ranging operation is
 * continuous. The clear interrupt has to be done after each get data to allow
 * the interrupt to raise when the next data is ready.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @return (enum VL53L4CD_Error) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_StartRanging(
		struct VL53L4CD_Dev *dev);

/**
 * @brief This function stops the ranging in progress.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @return (enum VL53L4CD_Error) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_StopRanging(
		struct VL53L4CD_Dev *dev);

/**
 * @brief This function check if a new data is available by polling a dedicated
 * register.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint8_t) *p_is_data_ready : Pointer containing a flag to know if a
 * data is ready : 0 = no data ready, 1 = data ready.
 * @return (enum VL53L4CD_Error) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_CheckForDataReady(
		struct VL53L4CD_Dev *dev,
		uint8_t *p_is_data_ready);

/**
 * @brief This function sets new range timing. Timing are composed of
 * TimingBudget and InterMeasurement. TimingBudget represents the timing during
 * VCSEL enabled, and InterMeasurement the time between two measurements.
 * The sensor can have different ranging mode depending of the configuration,
 * please refer to the user manual for more information.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint32_t) timing_budget_ms :  New timing budget in ms. Value can be
 * between 10ms and 200ms. Default is 50ms.
 * @param (uint32_t) inter_measurement_ms :  New inter-measurement in ms. If the
 * value is equal to 0, the ranging period is defined by the timing budget.
 * Otherwise, inter-measurement must be > timing budget. When all the timing
 * budget is consumed, the device goes in low power mode until inter-measurement
 * is done.
 * @return (uint8_t) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_SetRangeTiming(
		struct VL53L4CD_Dev *dev,
		uint32_t timing_budget_ms,
		uint32_t inter_measurement_ms);

/**
 * @brief This function gets the current range timing. Timing are composed of
 * TimingBudget and InterMeasurement. TimingBudget represents the timing during
 * VCSEL enabled, and InterMeasurement the time between two measurements.
 * The sensor can have different ranging mode depending of the configuration,
 * please refer to the user manual for more information.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint32_t) *p_timing_budget_ms :  Pointer containing the current
 * timing budget in ms.
 * @param (uint32_t) *p_inter_measurement_ms :  Pointer containing the current
 * inter-measurement in ms.
 * @return (uint8_t) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_GetRangeTiming(
		struct VL53L4CD_Dev *dev,
		uint32_t *p_timing_budget_ms,
		uint32_t *p_inter_measurement_ms);

/**
 * @brief This function gets the results reported by the sensor.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (struct VL53L4CD_ResultsData) *pResult :  Pointer of structure, filled with the
 * ranging results.
 * @return (uint8_t) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_GetResult(struct VL53L4CD_Dev *dev,
				       struct VL53L4CD_ResultsData *pResult);

/**
 * @brief This function sets a new offset correction in mm. Offset corresponds
 * to the difference in millimeters between real distance and measured distance.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (int16_t) OffsetValueInMm :  Offset value in millimeters. The minimum
 *  value is -1024mm and maximum is 1023mm.
 * @return (uint8_t) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_SetOffset(struct VL53L4CD_Dev *dev, int16_t OffsetValueInMm);

/**
 * @brief This function gets the current offset correction in mm. Offset
 * corresponds to the difference in millimeters between real distance and
 * measured distance.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (int16_t) OffsetValueInMm :  Offset value in millimeters. The minimum
 *  value is -1024mm and maximum is 1023mm.
 * @return (uint8_t) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_GetOffset(struct VL53L4CD_Dev *dev, int16_t *Offset);

/**
 * @brief This function sets a new Xtalk value in kcps. Xtalk represents the
 * correction to apply to the sensor when a protective coverglass is placed
 * at the top of the sensor.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint16_t) XtalkValueKcps : New xtalk value in kcps. The default
 * value is 0 kcps (no coverglass). Minimum is 0 kcps , and maximum is 128
 * kcps.
 * @return (enum VL53L4CD_Error) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_SetXtalk(struct VL53L4CD_Dev *dev, uint16_t XtalkValueKcps);

/**
 * @brief This function gets the current Xtalk value in kcps. Xtalk represents
 * the correction to apply to the sensor when a protective coverglass is placed
 * at the top of the sensor.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint16_t) p_xtalk_kcps : Pointer of current xtalk value in kcps.
 * @return (enum VL53L4CD_Error) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_GetXtalk(struct VL53L4CD_Dev *dev, uint16_t *p_xtalk_kcps);

/**
 * @brief This function sets new detection thresholds. The detection
 * thresholds can be programmed to generate an interrupt on pin 7 (GPIO1), only
 * when a condition on distance is reach. Example:
 * VL53L4CD_SetDistanceThreshold(dev,100,300,0): Below 100 mm
 * VL53L4CD_SetDistanceThreshold(dev,100,300,1): Above 300 mm
 * VL53L4CD_SetDistanceThreshold(dev,100,300,2): Below 100mm or above 300mm
 * VL53L4CD_SetDistanceThreshold(dev,100,300,3): Above 100mm or below 300mm
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint16_t) distance_low_mm : Low distance threshold in millimeters.
 * @param (uint16_t) distance_high_mm : High distance threshold in millimeters.
 * @param (uint8_t) window : Interrupt windows (0=below low threshold;
 * 1=above high threshold; 2=out of low/high windows; 3=in low/high windows)
 * @return (enum VL53L4CD_Error) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_SetDetectionThresholds(struct VL53L4CD_Dev *dev,
		uint16_t distance_low_mm,
		uint16_t distance_high_mm,
		uint8_t window);


/**
 * @brief This function gets the current detection thresholds. The detection
 * thresholds can be programmed to generate an interrupt on pin 7 (GPIO1), only
 * when a condition on distance is reach.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint16_t) *p_distance_low_mm : Pointer of low distance threshold in
 * millimeters.
 * @param (uint16_t) *p_distance_high_mm : Pointer of high distance threshold in
 * millimeters.
 * @param (uint8_t) *p_window : Interrupt windows (0=below low threshold;
 * 1=above high threshold; 2=out of low/high windows; 3=in low/high windows)
 * @return (enum VL53L4CD_Error) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_GetDetectionThresholds(struct VL53L4CD_Dev *dev,
		uint16_t *p_distance_low_mm,
		uint16_t *p_distance_high_mm,
		uint8_t *p_window);

/**
 * @brief This function sets a new signal threshold in kcps. If a
 * target has a lower signal as the programmed value, the result status in
 * structure 'struct VL53L4CD_ResultsData' will be equal to 2.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint16_t) signal_kcps : New signal threshold in kcps. The default
 * value is 1024 kcps. Minimum is 0 kcps (no threshold), and maximum is 16384
 * kcps.
 * @return (enum VL53L4CD_Error) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_SetSignalThreshold(struct VL53L4CD_Dev *dev, uint16_t signal_kcps);

/**
 * @brief This function returns the current signal threshold in kcps. If a
 * target has a lower signal as the programmed value, the result status in
 * structure 'struct VL53L4CD_ResultsData' will be equal to 2.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint16_t) *p_signal_kcps : Pointer of signal threshold in kcps.
 * @return (enum VL53L4CD_Error) status : 0 if OK.
 */

enum VL53L4CD_Error VL53L4CD_GetSignalThreshold(struct VL53L4CD_Dev *dev,
		uint16_t *p_signal_kcps);

/**
 * @brief This function programs a new sigma threshold. The sigma corresponds to
 * the standard deviation of the returned pulse. If the computed sigma is above
 * the programmed value, the result status in structure 'struct VL53L4CD_ResultsData'
 * will be equal to 1.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint16_t) sigma_mm : New sigma threshold in mm. The default value is
 * 15mm. Minimum is 0mm (not threshold), and maximum is 16383mm.
 * @return (enum VL53L4CD_Error) status : 0 if programming is or 255 if value is too
 * high.
 */

enum VL53L4CD_Error VL53L4CD_SetSigmaThreshold(
		struct VL53L4CD_Dev *dev,
		uint16_t sigma_mm);

/**
 * @brief This function gets the current sigma threshold. The sigma corresponds
 * to the standard deviation of the returned pulse. If the computed sigma is
 * above the programmed value, the result status in structure
 * 'struct VL53L4CD_ResultsData' will be equal to 1.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @param (uint16_t) *p_sigma_mm : Current sigma threshold in mm.
 * @return (enum VL53L4CD_Error) status : 0 if programming is OK.
 */

enum VL53L4CD_Error VL53L4CD_GetSigmaThreshold(
		struct VL53L4CD_Dev *dev,
		uint16_t *p_sigma_mm);

/**
 * @brief This function can be called when the temperature might have changed by
 * more than 8 degrees Celsius. The function can only be used if the sensor is
 * not ranging, otherwise, the ranging needs to be stopped using function
 * 'VL53L4CD_StopRanging()'. After calling this function, the ranging can
 * restart normally.
 * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
 * @return (enum VL53L4CD_Error) status : 0 if update is OK.
 */

enum VL53L4CD_Error VL53L4CD_StartTemperatureUpdate(struct VL53L4CD_Dev *dev);

#endif  /* ZEPHYR_DRIVERS_SENSOR_VL53L4CD_VL53L4CD_API_H_ */
