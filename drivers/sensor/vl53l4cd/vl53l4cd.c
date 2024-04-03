/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_vl53l4cd

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#include "vl53l4cd_api.h"
#include "platform.h"

LOG_MODULE_REGISTER(VL53L4CD, CONFIG_SENSOR_LOG_LEVEL);

#define VL53L4CD_INITIAL_ADDR		0x29
#define VL53L4CD_BOOT_TIME_US		1200	/* VL53L4CD firmware boot period = 1.2 ms */

#define VL53L4CD_MODEL_ID		0xEB
#define VL53L4CD_MODULE_TYPE		0xAA
#define VL53L4CD_CHIP_ID		(((uint16_t)VL53L4CD_MODEL_ID) << 8 | VL53L4CD_MODULE_TYPE)

enum VL53L4CD_POWER_STATUS {
	VL53L4CD_XSHUT_ON = 1,
	VL53L4CD_XSHUT_OFF = 0,
};

struct vl53l4cd_data {
	/* struct k_sem lock; */ /* TODO: implement spin lock for data structure. */
	bool started;
	struct VL53L4CD_Dev vl53l4cd; /* TODO: replace this with struct i2c_dt_spec. */
	struct VL53L4CD_ResultsData result_data;

#ifdef CONFIG_VL53L4CD_INTERRUPT_MODE
	struct gpio_callback gpio_cb;
	struct k_work work;
	const struct device *dev;
#endif
};

struct vl53l4cd_config {
	const struct i2c_dt_spec i2c;
#ifdef CONFIG_VL53L4CD_XSHUT
	struct gpio_dt_spec xshut;
#endif
#ifdef CONFIG_VL53L4CD_INTERRUPT_MODE
	struct gpio_dt_spec gpio1;
#endif
};

static int vl53l4cd_read_sensor(struct vl53l4cd_data *data)
{
	enum VL53L4CD_Error ret;

	ret = VL53L4CD_GetResult(&data->vl53l4cd, &data->result_data);
	if (ret != VL53L4CD_ERROR_NONE) {
		LOG_ERR("Error while getting data from sensor. Returned %d.", ret);
		return -EIO;
	}

	ret = VL53L4CD_ClearInterrupt(&data->vl53l4cd);
	if (ret != VL53L4CD_ERROR_NONE) {
		LOG_ERR("Error clearing interrupt on the sensor. Returned %d.", ret);
		return -EBUSY;
	}

	return 0;
}

#ifdef CONFIG_VL53L4CD_XSHUT
static int vl53l4cd_sensor_power(const struct device *dev, enum VL53L4CD_POWER_STATUS status)
{
	const struct vl53l4cd_config *const config = dev->config;
	int ret;

	if (config->xshut.port) {
		ret = gpio_pin_set_dt(&config->xshut, status);
		if (ret < 0) {
			LOG_ERR("[%s] Unable to set XSHUT GPIO. Returned %d.", dev->name, ret);
			return ret;
		}

		if (status == VL53L4CD_XSHUT_ON) {
			k_sleep(K_USEC(VL53L4CD_BOOT_TIME_US)); /* Wait for device to boot up. */
			LOG_DBG("[%s] Resumed.", dev->name);
		} else {
			LOG_DBG("[%s] Suspended.", dev->name);
		}
	}

	return 0;
}
#endif

static int vl53l4cd_start(const struct device *dev)
{
	enum VL53L4CD_Error ret;
	struct vl53l4cd_data *data = dev->data;
	uint16_t vl53l4cd_id;

#ifdef CONFIG_VL53L4CD_XSHUT
	ret = vl53l4cd_sensor_power(dev, VL53L4CD_XSHUT_ON);
	if (ret < 0) {
		LOG_ERR("[%s] Unable to resume sensor.", dev->name);
		return -EIO;
	}
#endif

#ifdef CONFIG_VL53L4CD_RECONFIGURE_ADDRESS
	const struct vl53l4cd_config *config = dev->config;

	if (config->i2c.addr != VL53L4CD_INITIAL_ADDR) {
		ret = VL53L4CD_SetI2CAddress(&(data->vl53l4cd), config->i2c.addr << 1);
		if (ret != 0) {
			LOG_ERR("[%s] Unable to reconfigure I2C address.", dev->name);
			return -EIO;
		}

		data->vl53l4cd.i2c_dev_addr = config->i2c.addr;
		k_sleep(K_USEC(VL53L4CD_BOOT_TIME_US)); /* Wait for device to boot up. */
	}
#endif

	ret = VL53L4CD_GetSensorId(&(data->vl53l4cd), &vl53l4cd_id);
	if ((ret != VL53L4CD_ERROR_NONE) || (vl53l4cd_id != VL53L4CD_CHIP_ID)) {
		LOG_ERR("[%s] Issue with device identification. Returned = %d.", dev->name, ret);
		return -ENOTSUP;
	}

	ret = VL53L4CD_SensorInit(&(data->vl53l4cd));
	if (ret != VL53L4CD_ERROR_NONE) {
		LOG_DBG("[%s] Failed to initialize the sensor.", dev->name);
		return -EBUSY;
	}

	ret = VL53L4CD_StartRanging(&(data->vl53l4cd));
	if (ret != VL53L4CD_ERROR_NONE) {
		LOG_ERR("[%s] Error while starting sensor. Returned %d.", dev->name, ret);
		return -EBUSY;
	}

	data->started = true;

	return 0;
}

static int vl53l4cd_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret;
	struct vl53l4cd_data *data = dev->data;

	__ASSERT_NO_MSG((chan == SENSOR_CHAN_DISTANCE) || (chan == SENSOR_CHAN_PROX));

	if (data->started == false) {
		ret = vl53l4cd_start(dev);
		if (ret < 0) {
			return ret;
		}
	}

	ret = vl53l4cd_read_sensor(data);
	if (ret < 0) {
		LOG_ERR("[%s] Could not perform measurement; error = %d.", dev->name, ret);
		return ret;
	}

	return 0;
}

static int vl53l4cd_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct vl53l4cd_data *data = dev->data;

	if (chan == SENSOR_CHAN_PROX) {
		if (data->result_data.distance_mm < CONFIG_VL53L4CD_PROXIMITY_THRESHOLD) {
			val->val1 = 1;
		} else {
			val->val1 = 0;
		}

		val->val2 = 0;
	} else if (chan == SENSOR_CHAN_DISTANCE) {
		val->val1 = data->result_data.distance_mm / 1000;
		val->val2 = (data->result_data.distance_mm % 1000) * 1000;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

/* TODO: check if we need to implement more apis. */
static const struct sensor_driver_api vl53l4cd_api = {
	.sample_fetch = vl53l4cd_sample_fetch,
	.channel_get = vl53l4cd_channel_get,
};

#ifdef CONFIG_VL53L4CD_INTERRUPT_MODE
static void vl53l4cd_worker(struct k_work *work)
{
	struct vl53l4cd_data *data = CONTAINER_OF(work, struct vl53l4cd_data, work);
	int ret;

	ret = vl53l4cd_read_sensor(data);
	if (ret < 0) {
		LOG_ERR("Reading data from sensor failed. Returned %d.\n", ret);
	}
}

static void vl53l4cd_gpio_callback(const struct device *dev, struct gpio_callback *cb,
				   uint32_t pins)
{
	struct vl53l4cd_data *data = CONTAINER_OF(cb, struct vl53l4cd_data, gpio_cb);

	(void)k_work_submit(&data->work);
}

static int vl53l4cd_init_interrupt(const struct device *dev)
{
	struct vl53l4cd_data *data = dev->data;
	const struct vl53l4cd_config *config = dev->config;
	int ret;

	data->dev = dev; /* TODO: what is the role of this statement. */

	if (!gpio_is_ready_dt(&config->gpio1)) {
		LOG_ERR("[%s] GPIO %s is not ready.", dev->name, config->gpio1.port->name);
		return -ENODEV;
	}

	/* TODO: what is the use of GPIO_PULL_UP. */
	ret = gpio_pin_configure_dt(&config->gpio1, GPIO_INPUT | GPIO_PULL_UP);
	if (ret < 0) {
		LOG_ERR("[%s] Unable to configure GPIO as INPUT. Returned %d.", dev->name, ret);
		return -EIO;
	}

	gpio_init_callback(&data->gpio_cb, vl53l4cd_gpio_callback, BIT(config->gpio1.pin));

	ret = gpio_add_callback(config->gpio1.port, &data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to set GPIO callback. Returned %d.", dev->name, ret);
		return -EIO;
	}

	ret = gpio_pin_interrupt_configure_dt(&config->gpio1, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("[%s] Unable to config GPIO interrupt. Returned %d.", dev->name, ret);
		return -EIO;
	}

	data->work.handler = vl53l4cd_worker;

	return 0;
}
#endif

static int vl53l4cd_init(const struct device *dev)
{
	int ret;
	struct vl53l4cd_data *data = dev->data;
	const struct vl53l4cd_config *config = dev->config;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("[%s] I2C bus is not ready. Exiting.", dev->name);
		return -ENODEV;
	}

#if defined(CONFIG_VL53L4CD_RECONFIGURE_ADDRESS) || defined(CONFIG_PM_DEVICE)
	if (config->xshut.port == NULL) {
		LOG_ERR("[%s] Missing required XSHUT GPIO spec.", dev->name);
		return -ENOTSUP;
	}
#endif

	/* Initialize device's data structure (struct vl53l4cd_data). */
	data->started = false;
	data->vl53l4cd.i2c_bus = config->i2c.bus;
	data->vl53l4cd.i2c_addr = VL53L4CD_INITIAL_ADDR;

#ifdef CONFIG_VL53L4CD_XSHUT
	if (config->xshut.port) {
		ret = gpio_pin_configure_dt(&config->xshut, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("[%s] Unable to configure GPIO as output. Returned %d.", dev->name,
				ret);
			return -EIO;
		}
	}
#endif

#ifdef CONFIG_VL53L4CD_RECONFIGURE_ADDRESS
	/* Shutdown all vl53l4cd sensors on the i2c bus, so that, at each sensor's 1st transaction
	 * they can be enabled one at a time (one after another) and programmed with their address.
	 */
	ret = vl53l4cd_sensor_power(dev, VL53L4CD_XSHUT_OFF);
	if (ret < 0) {
		LOG_ERR("[%s] Unable to suspend sensor.", dev->name);
		return -EIO;
	}
#else
	if (config->i2c.addr != VL53L4CD_INITIAL_ADDR) {
		LOG_ERR("[%s] Invalid device address (address should be 0x%x or "
			"CONFIG_VL53L4CD_RECONFIGURE_ADDRESS should be enabled).",
			dev->name, VL53L4CD_INITIAL_ADDR);
		return -ENOTSUP;
	}

	ret = vl53l4cd_start(dev);
	if (ret < 0) {
		return ret;
	}
#endif

#ifdef CONFIG_VL53L4CD_INTERRUPT_MODE
	if (config->gpio1.port) {
		ret = vl53l4cd_init_interrupt(dev);
		if (ret < 0) {
			LOG_ERR("[%s] Failed to initialize GPIO interrupt.", dev->name);
			return -EIO;
		}
	}
#endif

	return 0;
}

#define VL53L4CD_INIT(inst)								\
	struct vl53l4cd_data vl53l4cd_data_##inst = {					\
	};										\
											\
	struct vl53l4cd_config vl53l4cd_config_##inst = {				\
		.i2c = I2C_DT_SPEC_INST_GET(inst),					\
		IF_ENABLED(CONFIG_VL53L4CD_XSHUT, (					\
			.xshut = GPIO_DT_SPEC_INST_GET_OR(inst, xshut_gpios, {0}),	\
			)								\
		)									\
		IF_ENABLED(CONFIG_VL53L4CD_INTERRUPT_MODE, (				\
			.gpio1 = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),	\
			)								\
		)									\
	};										\
											\
	SENSOR_DEVICE_DT_INST_DEFINE(							\
		inst,									\
		vl53l4cd_init,								\
		NULL,									\
		&vl53l4cd_data_##inst,							\
		&vl53l4cd_config_##inst,						\
		POST_KERNEL,								\
		CONFIG_SENSOR_INIT_PRIORITY,						\
		&vl53l4cd_api								\
	);

DT_INST_FOREACH_STATUS_OKAY(VL53L4CD_INIT);
