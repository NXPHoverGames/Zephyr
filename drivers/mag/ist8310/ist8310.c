/*
 * Copyright (c) 2022 Abel Sensors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ist8310

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/drivers/mag/ist8310.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ist8310, CONFIG_IST8310_LOG_LEVEL);

#define I2C_SLAVE_ADDR          IST8310_I2C_SLAVE_ADDRESS_3

static int ist8310_read_reg(const struct device *dev, uint8_t addr, uint8_t *val) {
	const struct ist8310_config *cfg = dev->config;
	int ret = i2c_reg_read_byte(cfg->i2c_dev, I2C_SLAVE_ADDR, addr, val);

	return ret;
}

static int ist8310_write_reg(const struct device *dev, uint8_t addr, uint8_t val) {
	const struct ist8310_config *cfg = dev->config;
	int ret = i2c_reg_write_byte(cfg->i2c_dev, I2C_SLAVE_ADDR, addr, val);

	return ret;
}

static int ist8310_check_device_id(const struct device *dev) {
	uint8_t reg = 0;
	int ret = ist8310_read_reg(dev, IST8310_WHO_AM_I, &reg);

	if (ret || reg != IST8310_WHO_AM_I_EXPECTED_VALUE)
		return -1;
	else
		return 0;
}

static int ist8310_change_mode(const struct device *dev, uint8_t mode) {
	int ret = ist8310_write_reg(dev, IST8310_CTRL1, mode);

	return ret;
}

static int ist8310_check_drdy_and_dor(const struct device *dev) {
	uint8_t reg = 0;
	int ret = ist8310_read_reg(dev, IST8310_STAT1, &reg);
	bool drdy = reg & IST8310_STAT1_DRDY, dor = reg & IST8310_STAT1_DOR;
	if (dor == true)
		LOG_INF("Bit 'dor' = %d. Data was skipped.", dor);

	if (ret || drdy == false)
		return -1;
	else
		return 0;
}

static int ist8310_configure(const struct device *dev, uint8_t wait_time_ms) {
	int ret = 0;

	struct ist8310_data *data = dev->data;
	if (wait_time_ms >= MIN_WAIT_TIME_MS)
		data->wait_time_ms = wait_time_ms;
	else {
		LOG_ERR("Wait time parameter passed (%d) < min wait time allowed (%d)",
			wait_time_ms, MIN_WAIT_TIME_MS);
		return -1;
	}

	if (ret |= ist8310_check_device_id(dev))
		return ret;

	if (ret |= ist8310_change_mode(dev, IST8310_CTRL1_MODE_SINGLE))
		return ret;
	k_sleep(K_MSEC(data->wait_time_ms));
		// Adding sleep because IST8310 requires MIN_WAIT_TIME_MS to refresh data.

	if (ret |= ist8310_check_drdy_and_dor(dev))
		return ret;

	return ret;
}

int ist8310_read_data(const struct device *dev, uint8_t wait_time_ms) {
	struct ist8310_data *data = dev->data;

	k_sem_init(&data->lock, 0, 1);

	int ret = 0;

	if (ret |= ist8310_configure(dev, wait_time_ms))
		goto out;

	uint8_t reg = 0;
	for (int i = 0; i < AXIS_COUNT; ++i) {
		ret |= ist8310_read_reg(dev, IST8310_OUTPUT_VALUE_X_L + i * 2 + 0, &reg);
		data->mag[i] = reg;
		ret |= ist8310_read_reg(dev, IST8310_OUTPUT_VALUE_X_L + i * 2 + 1, &reg);
		data->mag[i] |= reg << 8;
	}

	ret |= ist8310_read_reg(dev, IST8310_OUTPUT_VALUE_T_L, &reg);
	data->temp = reg;
	ret |= ist8310_read_reg(dev, IST8310_OUTPUT_VALUE_T_H, &reg);
	data->temp |= reg << 8;

out:
	k_sem_give(&data->lock);

	return ret;
}

int ist8310_display_data(const struct device *dev) {
	struct ist8310_data *data = dev->data;

	k_sem_init(&data->lock, 0, 1);

	LOG_INF("x-axis = %d\t y-axis = %d\t z-axis = %d\t temp = %d",
		data->mag[0], data->mag[1], data->mag[2], data->temp);

	k_sem_give(&data->lock);

	k_sleep(K_MSEC(100)); // Adding sleep to enable printing of logs.

	return 0;
}

static int ist8310_init(const struct device *dev) {
	struct ist8310_data *data = dev->data;
	const struct ist8310_config *cfg = dev->config;

	int ret = 0;

	k_sem_init(&data->lock, 0, 1);

	if (!device_is_ready(cfg->i2c_dev)) {
		LOG_ERR("%s device is not ready. returning.", cfg->i2c_dev->name);
		ret = -ENODEV;
		goto out;
	}

	data->mag[0] = 0;
	data->mag[1] = 0;
	data->mag[2] = 0;
	data->temp = 0;

	ret |= ist8310_change_mode(dev, IST8310_CTRL1_MODE_STDBY);
	ret |= ist8310_write_reg(dev, IST8310_PDCTRL, IST8310_PDCTRL_OPTIMIZED_PERFORMANCE);
	ret |= ist8310_write_reg(dev, IST8310_AVGCTRL, IST8310_AVGCTRL_LOW_NOISE_PERFORMANCE);

out:
	k_sem_give(&data->lock);

	return ret;
}

static const struct ist8310_api ist8310_api = {
	.read_data = ist8310_read_data,
	.display_data = ist8310_display_data
};

#define IST8310_INIT(n)																												\
	static struct ist8310_data ist8310_data_##n = {															\
		.wait_time_ms = MIN_WAIT_TIME_MS																					\
	};																																					\
																																							\
	static struct ist8310_config ist8310_config_##n = {													\
		.i2c_dev = DEVICE_DT_GET(DT_NODELABEL(lpi2c1))														\
	};																																					\
																																							\
	DEVICE_DT_INST_DEFINE(n, 																										\
		ist8310_init, 																														\
		NULL, 																																		\
		&ist8310_data_##n, 																												\
		&ist8310_config_##n,   																										\
		POST_KERNEL, 																															\
		CONFIG_IST8310_INIT_PRIORITY, 																						\
		&ist8310_api 																															\
	);

DT_INST_FOREACH_STATUS_OKAY(IST8310_INIT);
