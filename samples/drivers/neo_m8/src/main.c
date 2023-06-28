/*
 * Copyright (c) 2022 Abel Sensors.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gnss/ublox_neo_m8.h>

#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(neom8, CONFIG_NEOM8_LOG_LEVEL);

static const struct device *neo_dev;
static const struct neom8_api *neo_api;
static const struct device *uart_dev;
static const struct uart_driver_api *uart_api;

void initialize_device_and_api() {
	neo_dev = DEVICE_DT_GET(DT_NODELABEL(neom8));
	if (!device_is_ready(neo_dev)) {
		LOG_ERR("%s device is not ready. returning.", neo_dev->name);
		return;
	}

	neo_api = neo_dev->api;

  uart_dev = DEVICE_DT_GET(DT_NODELABEL(lpuart2));
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("%s device is not ready. returning.", uart_dev->name);
		return;
	}

	uart_api = uart_dev->api;
}

void set_baudrate(int baudrate) {
  /*
    "setting baudrate of neo-m8 to 0x9600 by calling 'cfg_prt' of neo_api."

    parameters:
      baudrate: the baudrate of uart used to send messages to neo-m8 module.
  */

  struct uart_config *config;
    uart_api->config_get(uart_dev, config);
  config->baudrate = baudrate;

  int ret = uart_api->configure(uart_dev, config);
  if (ret)
    LOG_ERR("UART configure error %d", ret);

	neo_api->cfg_prt(neo_dev, 0x01);
}

void configure_baudrate() {
  /*
    "calls set_baudrate for every possible baudrate of neo-m8 module."
    note: the default (pre-setted) baudrate of neo-m8 can't be determined,
      without calling cfg_prt and checking for acknowledgement by the module.
  */

  int baudrates[8] = {
    4800,
    9600,
    19200,
    38400,
    57600,
    115200,
    230400,
    460800,
  };

  for (int i = 0; i < 8; ++i)
    set_baudrate(baudrates[i]);

  set_baudrate(baudrates[3]); // seting the baudrate of uart to 0x9600.
}

void configure_messages() {
	neo_api->cfg_msg(neo_dev, NMEA_DTM, 0);
	neo_api->cfg_msg(neo_dev, NMEA_GBS, 0);
	neo_api->cfg_msg(neo_dev, NMEA_GLL, 0);
	neo_api->cfg_msg(neo_dev, NMEA_GNS, 0);
	neo_api->cfg_msg(neo_dev, NMEA_GRS, 0);
	neo_api->cfg_msg(neo_dev, NMEA_GSA, 0);
	neo_api->cfg_msg(neo_dev, NMEA_GST, 0);
	neo_api->cfg_msg(neo_dev, NMEA_GSV, 0);
	neo_api->cfg_msg(neo_dev, NMEA_RMC, 0);
	neo_api->cfg_msg(neo_dev, NMEA_VLW, 0);
	neo_api->cfg_msg(neo_dev, NMEA_VTG, 0);
	neo_api->cfg_msg(neo_dev, NMEA_ZDA, 0);
}

void configure_neo_m8() {
  LOG_INF("configuring baudrate.");
  configure_baudrate();

  LOG_INF("configuring neo_m8 module.");
	neo_api->cfg_prt(neo_dev, 0x01);
	neo_api->cfg_nav5(neo_dev, Stationary, P_2D, 0, 1, 5, 100, 100, 100, 350,
		0, 60, 0, 0, 0, AutoUTC);
	neo_api->cfg_gnss(neo_dev, 0, 32, 5, 0, 8, 16, 0, 0x01010001, 1, 1, 3, 0,
		0x01010001, 3, 8, 16, 0, 0x01010000, 5, 0, 3, 0, 0x01010001, 6, 8, 14, 0,
		0x01010001);

  LOG_INF("configuring messages.");
  configure_messages();

	LOG_INF("\n");

	return;
}

void display_gnss_data() {
	struct time neotime;
	float lat;
	char ns;
	float lon;
	char ew;
	float alt;
	int sat;

	neo_api->get_time(neo_dev, &neotime);
	neo_api->get_latitude(neo_dev, &lat);
	neo_api->get_ns(neo_dev, &ns);
	neo_api->get_longitude(neo_dev, &lon);
	neo_api->get_ew(neo_dev, &ew);
	neo_api->get_altitude(neo_dev, &alt);
	neo_api->get_satellites(neo_dev, &sat);

	LOG_INF("Hour: %d", neotime.hour);
	LOG_INF("Minute: %d", neotime.min);
	LOG_INF("Second: %d", neotime.sec);
	LOG_INF("Latitude: %.5f", lat);
	LOG_INF("North/South: %c", ns);
	LOG_INF("Longitude: %.5f", lon);
	LOG_INF("East/West: %c", ew);
	LOG_INF("Altitude: %.2f", alt);
	LOG_INF("Satellites: %d", sat);

  LOG_INF("\n");
}

void main(void)
{
	LOG_INF("sample application for testing ublox_neo_m8: started.\n");

  initialize_device_and_api();

  configure_neo_m8();

	int rc;
	while (true) {
		rc = neo_api->fetch_data(neo_dev);
		if (rc) {
			LOG_ERR("error %d while reading data. breaking out of the loop.\n", rc);
			break;
		}

		display_gnss_data();
		k_sleep(K_MSEC(100));
	}

	LOG_INF("sample application for testing ublox_neo_m8: completed.\n");
}
