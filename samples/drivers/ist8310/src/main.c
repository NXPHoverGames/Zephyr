/*
 * Copyright (c) 2022 Abel Sensors.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/drivers/mag/ist8310.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(ist8310, CONFIG_IST8310_LOG_LEVEL);

#define WAIT_TIME_MS            6

static const struct device *ist8310_dev;
static const struct ist8310_api *ist8310_api;

void main(void)
{
	ist8310_dev = DEVICE_DT_GET(DT_NODELABEL(ist8310));
	if (!device_is_ready(ist8310_dev)) {
		LOG_ERR("Device %s is not ready. Returning.", ist8310_dev->name);
		return;
	}

	ist8310_api = ist8310_dev->api;

	int ret = 0;
	while (true) {
		ret |= ist8310_api->read_data(ist8310_dev, WAIT_TIME_MS);
		ret |= ist8310_api->display_data(ist8310_dev);

		if (ret) {
			LOG_ERR("Error %d while reading data. Returning.", ret);
			return;
		}
	}
}
