#include <zephyr/types.h>

/*
	IST8310 typedef definitions.
*/

typedef int (*ist8310_api_read_data)(const struct device *dev,
	uint8_t wait_time_ms);
typedef int (*ist8310_api_display_data)(const struct device *dev);

/*
	IST8310 data structures.
*/

struct ist8310_data {
	int16_t mag[3];
	int16_t temp;

	struct k_sem lock;
	uint8_t wait_time_ms;
};

struct ist8310_config {
	const struct device *i2c_dev;
};

__subsystem struct ist8310_api {
	ist8310_api_read_data read_data;
	ist8310_api_display_data display_data;
};

/*
	IST8310 internal addresses.
*/

#define IST8310_I2C_SLAVE_ADDRESS_1           0x0C
#define IST8310_I2C_SLAVE_ADDRESS_2           0x0D
#define IST8310_I2C_SLAVE_ADDRESS_3           0x0E
#define IST8310_I2C_SLAVE_ADDRESS_4           0x0F

#define IST8310_WHO_AM_I                      0x00

#define IST8310_STAT1                         0x02

// Measurement data is stored in 2's complement form.
#define IST8310_OUTPUT_VALUE_X_L              0x03
#define IST8310_OUTPUT_VALUE_X_H              0x04
#define IST8310_OUTPUT_VALUE_Y_L              0x05
#define IST8310_OUTPUT_VALUE_Y_H              0x06
#define IST8310_OUTPUT_VALUE_Z_L              0x07
#define IST8310_OUTPUT_VALUE_Z_H              0x08

#define IST8310_STAT2                         0x09

#define IST8310_CTRL1                         0x0A

#define IST8310_CTRL2                         0x0B

#define IST8310_STR                           0x0c

// Measurement data is stored in 2's complement form.
#define IST8310_OUTPUT_VALUE_T_L              0x1C
#define IST8310_OUTPUT_VALUE_T_H              0x1D

#define IST8310_AVGCTRL                       0X41

#define IST8310_PDCTRL                        0X42

/*
	IST8310 internal constants.
*/

#define MIN_WAIT_TIME_MS                    6

#define AXIS_COUNT                          3

#define IST8310_WHO_AM_I_EXPECTED_VALUE     0x10

#define IST8310_STAT1_DRDY_SHFITS           0x00
#define IST8310_STAT1_DRDY                  (1 << IST8310_STAT1_DRDY_SHFITS)
#define IST8310_STAT1_DOR_SHFITS            0x01
#define IST8310_STAT1_DOR                   (1 << IST8310_STAT1_DOR_SHFITS)

#define IST8310_STAT2_INTERRUPT_SHFITS      0x03
#define IST8310_STAT2_INTERRUPT             (1 << IST8310_STAT2_INT_SHFITS)

#define IST8310_CTRL1_MODE_SHFITS           0
#define IST8310_CTRL1_MODE_STDBY            (0 << IST8310_CTRL1_MODE_SHFITS)
#define IST8310_CTRL1_MODE_SINGLE           (1 << IST8310_CTRL1_MODE_SHFITS)

#define IST8310_CTRL2_SRST_SHFITS           0x00
#define IST8310_CTRL2_SRST                  (1 << IST8310_CTRL2_SRST_SHFITS)
#define IST8310_CTRL2_DRP_SHIFTS            0x02
#define IST8310_CTRL2_DRP                   (1 << IST8310_CTRL2_DRP_SHIFTS)
#define IST8310_CTRL2_DREN_SHIFTS           0x03
#define IST8310_CTRL2_DREN                  (1 << IST8310_CTRL2_DREN_SHIFTS)

#define IST8310_STR_SELF_TEST_SHFITS        6
#define IST8310_STR_SELF_TEST_ON            (1 << IST8310_STR_SELF_TEST_SHFITS)
#define IST8310_STR_SELF_TEST_OFF           (0 << IST8310_STR_SELF_TEST_SHFITS)

#define IST8310_AVGCTRL_LOW_NOISE_PERFORMANCE     0B00100100

#define IST8310_PDCTRL_OPTIMIZED_PERFORMANCE      0B11000000
