/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>

#define VL53L4CD_SAMPLE_MEASUREMENT_DELAY_MS		1000

static int vl53l4cd_sample_data_and_display_data(const struct device *const sensor)
{
	int ret;
	struct sensor_value value;

	/* If interrupt mode is used, then we don't need to call sensor_sample_fetch. */
#ifndef CONFIG_VL53L4CD_INTERRUPT_MODE
	ret = sensor_sample_fetch(sensor);
	if (ret) {
		printk("[%s] sensor_sample_fetch failed. Returned %d.\n", sensor->name, ret);
		return ret;
	}
#endif

	ret = sensor_channel_get(sensor, SENSOR_CHAN_PROX, &value);
	if (ret) {
		printk("[%s] sensor_channel_get failed. Returned %d.\n", sensor->name, ret);
		return ret;
	}
	printk("[%s] proximity = %d\n", sensor->name, value.val1);

	ret = sensor_channel_get(sensor, SENSOR_CHAN_DISTANCE, &value);
	if (ret) {
		printk("[%s] sensor_channel_get failed. Returned %d.\n", sensor->name, ret);
		return ret;
	}
	printf("[%s] distance = %.3f m\n", sensor->name, sensor_value_to_double(&value));

	k_sleep(K_MSEC(VL53L4CD_SAMPLE_MEASUREMENT_DELAY_MS));

	return 0;
}

int main(void)
{
	int ret;
	const struct device *const sensor_0 = DEVICE_DT_GET(DT_NODELABEL(st_vl53l4cd_0));
	const struct device *const sensor_1 = DEVICE_DT_GET(DT_NODELABEL(st_vl53l4cd_0));

	if (!device_is_ready(sensor_0) || !device_is_ready(sensor_1)) {
		printk("[%s, %s] sensor devices are not ready.\n", sensor_0->name, sensor_1->name);
		return -ENODEV;
	}

	/* If CONFIG_VL53L4CD_RECONFIGURE_ADDRESS=y, sensors are started at the first fetch. */
#ifdef CONFIG_VL53L4CD_RECONFIGURE_ADDRESS
	ret = sensor_sample_fetch(sensor_0) || sensor_sample_fetch(sensor_1);
	if (ret) {
		printk("[%s, %s] address reconfiguration failed. Returned %d.\n", sensor_0->name,
		       sensor_1->name, ret);
		return ret;
	}
#endif

	while (1) {
		ret = vl53l4cd_sample_data_and_display_data(sensor_0);
		if (ret) {
			return ret;
		}

		ret = vl53l4cd_sample_data_and_display_data(sensor_1);
		if (ret) {
			return ret;
		}
	}

	return 0;
}
