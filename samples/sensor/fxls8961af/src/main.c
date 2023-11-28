/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/sensor/fxls8961af.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(fxls8961af, CONFIG_FXLS8961AF_LOG_LEVEL);

static const struct device *fxls8961af_dev;
static const struct fxls8961af_dev_api *fxls8961af_api;

int main(void)
{
	fxls8961af_dev = DEVICE_DT_GET(DT_NODELABEL(fxls8961af));
	if (!device_is_ready(fxls8961af_dev)) {
		LOG_ERR("Device %s is not ready. Returning.", fxls8961af_dev->name);
		return -1;
	}

	fxls8961af_api = fxls8961af_dev->api;

	int ret = 0;
	while (true) {
		ret |= fxls8961af_api->read_data(fxls8961af_dev);
		ret |= fxls8961af_api->display_data(fxls8961af_dev);

		if (ret) {
			LOG_ERR("Error %d while reading data. Returning.", ret);
			return -1;
		}

		k_sleep(K_MSEC(100));
	}

	return 0;
}
