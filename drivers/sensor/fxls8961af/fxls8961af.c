/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_fxls8961af

#include <assert.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/sensor/fxls8961af.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(fxls8961af, CONFIG_FXLS8961AF_LOG_LEVEL);

#define ACC_NODE_LABEL		fxls8961af
#define SPI_NODE_LABEL		lpspi4
#define CHECK_DRDY_TIMEOUT	1000000
#define MESSAGE_HEADER_BYTES	2

/*!
 * @brief Read n bytes starting at register from sensor.
 *
 * @param dev Pointer to the sensor.
 * @param reg_addr Register address.
 * @param buf_addr Pointer to the buffer for read data.
 * @param byte_count Number of bytes to be read.
 *
 * @return status_success if success or status_failure if error.
 */
static status_t FXLS8961AF_ReadReg(const struct device *dev, uint8_t reg_addr,
				   uint8_t *buf_addr, uint8_t byte_count) {
	assert(dev != NULL);
	assert(buf_addr != NULL);
	status_t status = status_success;
	uint16_t message_len = MESSAGE_HEADER_BYTES + byte_count;

	uint8_t buffer[message_len];
	buffer[0] = (1 << 7) | reg_addr;
	const struct spi_buf buf_data = { .buf = &buffer, .len = message_len };
	const struct spi_buf_set buf_set = { .buffers = &buf_data, .count = 1 };

	const fxls8961af_dev_config *cfg = dev->config;
	status = spi_transceive(cfg->spi_dev, &cfg->spi_cfg, &buf_set, &buf_set);

	return status;
}

/*!
 * @brief Write 1 byte to register of sensor.
 *
 * @param dev Pointer to the sensor.
 * @param reg_addr Register address.
 * @param val Data byte to be written.
 *
 * @return status_success if success or status_failure if error.
 */
static status_t FXLS8961AF_WriteReg(const struct device *dev, uint8_t reg_addr,
				    uint8_t val) {
	assert(dev != NULL);
	status_t status = status_success;
	uint16_t message_len = MESSAGE_HEADER_BYTES + 1;

	uint8_t buffer[3] = {( ((uint8_t) ~1) >> 1 ) & reg_addr, 0, val};
	const struct spi_buf buf_data = { .buf = &buffer, .len = message_len };
	const struct spi_buf_set buf_set = { .buffers = &buf_data, .count = 1 };

	const fxls8961af_dev_config *cfg = dev->config;
	status = spi_transceive(cfg->spi_dev, &cfg->spi_cfg, &buf_set, &buf_set);

	return status;
}

/*!
 * @brief Check data is ready.
 *
 * @param dev Pointer to sensor.
 *
 * @return status_success if success or status_failure if error.
 */
static status_t FXLS8961AF_CheckDataReady(const struct device *dev) {
	assert(dev != NULL);

	uint8_t val = 0;
	int count = CHECK_DRDY_TIMEOUT;

	while ((val & SRC_DRDY_MASK) != SRC_DRDY_MASK
		&& (FXLS8961AF_ReadReg(dev, INT_STATUS_REG, &val, 1) || count-- > 0));

	if ((val & SRC_DRDY_MASK) != SRC_DRDY_MASK)
		return status_failure;

	if (val & SRC_OVF_MASK)
		LOG_INF("Overflow: new output data latched before previous set was read.");
	if (val & SRC_ORIENT_MASK)
		LOG_INF("Orientation change: an orientation change event has occurred.");
	if (val & SRC_ASLP_MASK)
		LOG_INF("A WAKE-to-SLEEP/SLEEP-to-WAKE system transition has occurred.");

	return status_success;
}

/*!
 * @brief Read raw sensor data.
 * @note The content of all data reg is locked during the spi/i2c transaction.
 *  Hence, for single byte read transactions, the coherency between MSB and LSB
 *    data bytes can't be granted as they will be associated with distinct transactions.
 *  Hence, it is highly recommended to use multiple bytes read transactions,
 *    in order to at least collect both MSB and LSB data bytes with a single transaction.
 *  Obviously, collecting all three axes data simultaneously using a 6 bytes transaction
 *    will also ensure that the XYZ data triplet corresponds to the same timestamp.
 *
 * @param dev Pointer to sensor.
 *
 * @return status_success if success or status_failure if error.
 */
static status_t FXLS8961AF_ReadRawSensorData(const struct device *dev) {
	assert(dev != NULL);

	if(FXLS8961AF_CheckDataReady(dev)) return status_failure;

	status_t status = status_success;
	fxls8961af_dev_data_t *data = dev->data;
	fxls8961af_raw_sensor_data_t *raw_data = &(data->raw_sensor_data);

	status |= FXLS8961AF_ReadReg(dev, OUT_X_LSB_REG, &(raw_data->accelXLSB), 6);
	status |= FXLS8961AF_ReadReg(dev, TEMP_OUT_REG, &(data->temp_celsius), 1);

	return status;
}

/*!
 * @brief Synthesize the data.
 *
 * @param dev Pointer to sensor.
 *
 * @return status_success if success or status_failure if error.
 */
static status_t FXLS8961AF_SynthesizeData(const struct device *dev) {
	assert(dev != NULL);

	status_t status = status_success;

	fxls8961af_dev_data_t *data = dev->data;
	fxls8961af_raw_sensor_data_t *raw_data = &(data->raw_sensor_data);
	fxls8961af_synthesized_data_t *syn_data = &(data->synthesized_data);

	syn_data->accelX = (int16_t)(uint16_t)
		(((uint16_t) raw_data->accelXMSB << 8) | (uint16_t) raw_data->accelXLSB);
	syn_data->accelY = (int16_t)(uint16_t)
		(((uint16_t) raw_data->accelYMSB << 8) | (uint16_t) raw_data->accelYLSB);
	syn_data->accelZ = (int16_t)(uint16_t)
		(((uint16_t) raw_data->accelZMSB << 8) | (uint16_t) raw_data->accelZLSB);

	return status;
}

/*!
 * @brief Check configuration of spi bus.
 *
 * @param dev Pointer to sensor.
 *
 * @return status_success if success or status_failure if error.
 */
static status_t FXLS8961AF_CheckSpiBusConfiguration(const struct device *dev) {
	assert(dev != NULL);

	const fxls8961af_dev_config *cfg = dev->config;
	const struct spi_config *spi_cfg = &cfg->spi_cfg;

	return (spi_cfg->frequency != FXLS8961AF_SPI_MAX_FREQUENCY_HZ
		|| spi_cfg->operation & SPI_OP_MODE_SLAVE
		|| spi_cfg->operation & SPI_TRANSFER_LSB
		|| spi_cfg->operation & SPI_MODE_CPOL
		|| spi_cfg->operation & SPI_MODE_CPHA
		|| spi_cfg->operation & (SPI_WORD_SIZE_MASK & ~SPI_WORD_SET(8))
		|| spi_cfg->operation & SPI_HALF_DUPLEX
		|| spi_cfg->operation & SPI_CS_ACTIVE_HIGH) ? status_failure : status_success;
}

/*!
 * @brief Verify and initialize the sensor.
 *
 * @param dev Pointer to sensor.
 *
 * @return status_success if success or status_failure if error.
 */
static status_t FXLS8961AF_InitSensor(const struct device *dev) {
	assert(dev != NULL);

	const fxls8961af_dev_config *cfg = dev->config;
	const struct device *spi_dev = cfg->spi_dev;

	if (!device_is_ready(spi_dev)) {
		LOG_ERR("Bus device %s is not ready", spi_dev->name);
		return status_failure;
	}

	if (FXLS8961AF_CheckSpiBusConfiguration(dev)) return status_failure;

	status_t status = status_success;
	uint8_t val = 0;

	k_sleep(K_MSEC(T_BOOT_1_MS)); // Wait for the device to enter Standby mode.

	/* Check WHO_AM_I_REG register. */
	status = FXLS8961AF_ReadReg(dev, WHO_AM_I_REG, &val, 1);
	if (val != FXLS8961AF_WHO_AM_I_DEVICE_ID || status) return status_failure;

	/* Check SENS_CONFIG1 is in the standby mode (by reading ACTIVE field). */
	status = FXLS8961AF_ReadReg(dev, SENS_CONFIG1_REG, &val, 1);
	if ((val & ACTIVE_MODE_MASK) == ACTIVE_MODE_MASK || status)
		return status_failure;

	/* Setup ASLP_COUNT to it's reset value. */
	if (FXLS8961AF_WriteReg(dev, ASLP_COUNT_LSB_REG, ASLP_COUNT_REG_RESET))
		return status_failure;
	if (FXLS8961AF_WriteReg(dev, ASLP_COUNT_MSB_REG, ASLP_COUNT_REG_RESET))
		return status_failure;

	/* Setup SENS_CONFIG1 to enter active mode (by setting ACTIVE field = 1). */
	status = FXLS8961AF_WriteReg(dev, SENS_CONFIG1_REG,
		(val | ACTIVE_MODE_MASK | FSR_4G_MASK) & ~SPI_M_MASK);
	if (status) return status_failure;

	/* Check SENS_CONFIG1 is in the active mode (by reading ACTIVE field). */
	status = FXLS8961AF_ReadReg(dev, SENS_CONFIG1_REG, &val, 1);
	if ((val & ACTIVE_MODE_MASK) != ACTIVE_MODE_MASK || status)
		return status_failure;

	k_sleep(K_NSEC(INITIAL_MEASURE_DELAY_NS)); // Wait for first measurement.

	return status_success;
}

/*!
 * @brief API for reading data.
 *
 * @param dev Pointer to sensor.
 *
 * @return status_success if success or status_failure if error.
 */
static status_t fxls8961af_read_data(const struct device *dev) {
	status_t status = status_success;

	k_sleep(K_NSEC(INCREMENTAL_MEASURE_DELAY_NS));

	status &= FXLS8961AF_ReadRawSensorData(dev);
	status &= FXLS8961AF_SynthesizeData(dev);

	return status;
}

/*!
 * @brief API for displaying data.
 *
 * @param dev Pointer to sensor.
 *
 * @return status_success if success or status_failure if error.
 */
static status_t fxls8961af_display_data(const struct device *dev) {
	fxls8961af_dev_data_t *data = dev->data;
	fxls8961af_synthesized_data_t *syn_data = &(data->synthesized_data);

	k_sem_init(&data->lock, 0, 1);

	LOG_INF("x-axis = %d\t y-axis = %d\t z-axis = %d\t temp (in celsius) = %d",
		syn_data->accelX, syn_data->accelY, syn_data->accelZ, data->temp_celsius
		+ TEMP_OFFSET);

	k_sem_give(&data->lock);

	k_sleep(K_MSEC(100)); // Adding sleep to enable printing of logs.

	return status_success;
}

static const struct fxls8961af_dev_api fxls8961af_api = {
	.read_data = fxls8961af_read_data,
	.display_data = fxls8961af_display_data
};

#define FXLS8961AF_INIT(n)									\
	fxls8961af_dev_data_t fxls8961af_data_##n = {						\
	};											\
	fxls8961af_dev_config fxls8961af_config_##n = {						\
		.spi_dev = DEVICE_DT_GET(DT_NODELABEL(SPI_NODE_LABEL)),				\
		.spi_cfg = {									\
			.frequency = FXLS8961AF_SPI_MAX_FREQUENCY_HZ,				\
			.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8)	\
				| SPI_FULL_DUPLEX,						\
		}										\
	};											\
	DEVICE_DT_INST_DEFINE(n,								\
		FXLS8961AF_InitSensor,								\
		NULL,										\
		&fxls8961af_data_##n,								\
		&fxls8961af_config_##n,								\
		POST_KERNEL,									\
		CONFIG_FXLS8961AF_INIT_PRIORITY,						\
		&fxls8961af_api									\
	);

DT_INST_FOREACH_STATUS_OKAY(FXLS8961AF_INIT);
