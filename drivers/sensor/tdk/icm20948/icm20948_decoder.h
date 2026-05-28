#ifndef ZEPHYR_DRIVERS_SENSOR_TDK_ICM20948_ICM20948_DECODER_H_
#define ZEPHYR_DRIVERS_SENSOR_TDK_ICM20948_ICM20948_DECODER_H_
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include "icm20948.h"


struct icm20948_decoder_header {
	uint64_t timestamp;
	uint8_t is_fifo;
	uint8_t gyro_fs;
	uint8_t accel_fs;
};
struct icm20948_fifo_data {
	struct icm20948_decoder_header header;
	uint8_t int_status2;
	uint8_t int_status3;
	uint8_t fifo_en1;
	uint8_t fifo_en2;
	uint8_t fifo_mode;
	uint8_t gyro_odr;
	uint8_t accel_odr;
	uint16_t fifo_count;
	uint8_t fifo_count_buf[2];
	uint16_t rtc_freq;
};
#endif