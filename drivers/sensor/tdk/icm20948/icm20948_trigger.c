/*
 * Copyright (c) 2025 Aditya Ganesh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/util.h"
#include "stdint.h"
#include "zephyr/drivers/i2c.h"
#include "zephyr/drivers/gpio.h"
#include "zephyr/drivers/sensor.h"
#include "zephyr/init.h"
#include "zephyr/logging/log.h"
#include "zephyr/sys/__assert.h"
#include "icm20948.h"

#include <math.h>


LOG_MODULE_DECLARE(ICM20948, CONFIG_SENSOR_LOG_LEVEL);
struct sensor_value packet[3000];
k_ticks_t total_ticks[3000];
uint16_t tick_index = 0;
k_ticks_t oldticks = 0;
static void icm20948_gpio_callback(const struct device *dev,
				  struct gpio_callback *cb, uint32_t pins)
{
	struct icm20948_data *data = CONTAINER_OF(cb, struct icm20948_data, gpio_cb);
	const struct icm20948_config *cfg = data->dev->config;
    uint8_t ready_to_read = 0;

    gpio_pin_interrupt_configure_dt(&cfg->int_pin, GPIO_INT_DISABLE);
	k_sem_give(&data->gpio_sem);
	if (IS_ENABLED(CONFIG_ICM20948_TRIGGER)) {
		icm20948_fifo_event(data->dev);
	}
}

static int icm20948_convert_accel(const struct icm20948_config *cfg, int32_t raw_accel_value,
				  struct sensor_value *output_value)
{
	int64_t sensitivity = 0; /* value equivalent for 1g */

	switch (cfg->accel_fs) {
	case ICM20948_ACCEL_FS_SEL_2G:
		sensitivity = 16384;
		break;
	case ICM20948_ACCEL_FS_SEL_4G:
		sensitivity = 8192;
		break;
	case ICM20948_ACCEL_FS_SEL_8G:
		sensitivity = 4096;
		break;
	case ICM20948_ACCEL_FS_SEL_16G:
		sensitivity = 2048;
		break;
	default:
		return -EINVAL;
	}

	/* Convert to micrometers/s^2 */
	int64_t in_ms = raw_accel_value * SENSOR_G;

	/* meters/s^2 whole values */
	output_value->val1 = in_ms / (sensitivity * 1000000LL);

	/* micrometers/s^2 */
	output_value->val2 = (in_ms - (output_value->val1 * sensitivity * 1000000LL)) / sensitivity;

	return 0;
}


static void icm20948_thread_cb(const struct device *dev)
{
	struct icm20948_data *drv_data = dev->data;
	const struct icm20948_config *cfg = dev->config;

	// if (drv_data->data_ready_handler != NULL) {
	// 	drv_data->data_ready_handler(dev,
	// 				     drv_data->data_ready_trigger);
	// }

	if (i2c_reg_update_byte_dt(&cfg->i2c,
				ICM20948_REG_BANK_SEL,
				0x30,
				0 << 4)) {
		LOG_ERR("Error switching banks");
		return;
	}

	uint8_t cnth = 0, cntl = 0;
	uint8_t user_ctrl = 0;
	uint8_t fifo_en2 = 0;
	uint8_t int_status_2 = 0;
	uint8_t int_status_3 = 0;

	i2c_reg_read_byte_dt(&cfg->i2c, ICM20948_REG_USER_CTRL, &user_ctrl);
	i2c_reg_read_byte_dt(&cfg->i2c, ICM20948_REG_FIFO_EN_2, &fifo_en2);
	i2c_reg_read_byte_dt(&cfg->i2c, ICM20948_REG_INT_STATUS_2, &int_status_2);
	i2c_reg_read_byte_dt(&cfg->i2c, ICM20948_REG_INT_STATUS_3, &int_status_3);
	i2c_reg_read_byte_dt(&cfg->i2c, ICM20948_FIFO_CNTH, &cnth);
	i2c_reg_read_byte_dt(&cfg->i2c, ICM20948_FIFO_CNTL, &cntl);
	uint16_t fifo_count = ((uint16_t)(cnth & 0x1F) << 8) | cntl;



	/* Accel-only packet = 6 bytes */
	uint16_t packet_size = 6;

	uint8_t packet_sample[6];
	uint16_t bytes_to_read = fifo_count;
	uint16_t packet_index = 0;
	k_ticks_t ticks = k_uptime_ticks();
	oldticks = ticks;

	total_ticks[tick_index] = ticks;
	tick_index += 1;

	while (bytes_to_read >= packet_size) {
		if (i2c_burst_read_dt(&cfg->i2c,
					ICM20948_REG_FIFO_R_W,
					packet_sample,
					packet_size)) {
			LOG_ERR("Failed to read FIFO packet");
			gpio_pin_interrupt_configure_dt(&cfg->int_pin,
				GPIO_INT_EDGE_TO_ACTIVE);
			return;
		}
		icm20948_convert_accel(cfg, (int16_t)((packet_sample[0] << 8) | packet_sample[1]),
					&packet[packet_index]);
		packet_index += 1;
		bytes_to_read -= packet_size;
	}
	uint8_t reg = 0;

	// /* Assert reset */
	// i2c_reg_read_byte_dt(&cfg->i2c,
	// 			ICM20948_FIFO_RST,
	// 			&reg);

	// reg &= 0xe0;

	// reg |= 0x1f;

	// /* Assert reset */
	// i2c_reg_write_byte_dt(&cfg->i2c,
	// 			ICM20948_FIFO_RST,
	// 			reg);

	// k_sleep(K_MSEC(1));

	// i2c_reg_read_byte_dt(&cfg->i2c,
	// 			ICM20948_FIFO_RST,
	// 			&reg);

	// reg &= 0xe0;

	// reg |= 0x1e;

	// /* Assert reset */
	// i2c_reg_write_byte_dt(&cfg->i2c,
	// 			ICM20948_FIFO_RST,
	// 			reg);


	gpio_pin_interrupt_configure_dt(&cfg->int_pin,
					GPIO_INT_EDGE_TO_ACTIVE);


}

static void icm20948_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct icm20948_data *drv_data = p1;

	while (1) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		icm20948_thread_cb(drv_data->dev);
	}
}



int icm20948_init_interrupt(const struct device *dev)
{
	struct icm20948_data *drv_data = dev->data;
	const struct icm20948_config *cfg = dev->config;
	if (!gpio_is_ready_dt(&cfg->int_pin)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

	drv_data->dev = dev;

	gpio_pin_configure_dt(&cfg->int_pin, GPIO_INPUT);

	gpio_init_callback(&drv_data->gpio_cb,
			   icm20948_gpio_callback,
			   BIT(cfg->int_pin.pin));

	if (gpio_add_callback(cfg->int_pin.port, &drv_data->gpio_cb) < 0) {
		LOG_ERR("Failed to set gpio callback");
		return -EIO;
	}

	if(i2c_reg_update_byte_dt(&cfg->i2c, ICM20948_REG_BANK_SEL, 0x30,0 << 4)){
        LOG_ERR("Error switching banks");
        return -EIO;
    }







	/*Interrupt pin configuration
	 * Configure interrupt pin to be active high, push-pull, and latched until cleared.
	 * Also enable bypass mode so that the host can directly access the magnetometer.
	*/
	if (i2c_reg_write_byte_dt(&cfg->i2c, ICM20948_REG_INT_PIN_CFG,
				  ICM20948_INT1_ANYRD_2CLEAR|ICM20948_BYPASS_EN) < 0) {
		LOG_ERR("Failed to enable data ready interrupt.");
		return -EIO;
	}

	/*
	* Enable FIFO watermark interrupt.
	*
	* NOTE:
	* Previously you had:
	*
	*   ICM20948_REG_INT_ENABLE_1 = 0x01
	*
	* That enables DATA_RDY interrupt, not FIFO watermark.
	*/
	if (i2c_reg_write_byte_dt(&cfg->i2c,
				ICM20948_REG_INT_ENABLE_2,
				0x01) < 0) {
		LOG_ERR("Failed to enable FIFO overflow interrupt");
		return -EIO;
	}

	/*
	* Enable FIFO watermark interrupt.
	*
	* NOTE:
	* Previously you had:
	*
	*   ICM20948_REG_INT_ENABLE_1 = 0x01
	*
	* That enables DATA_RDY interrupt, not FIFO watermark.
	*/
	if (i2c_reg_write_byte_dt(&cfg->i2c,
				ICM20948_REG_INT_ENABLE_3,
				0x01) < 0) {
		LOG_ERR("Failed to enable FIFO watermark interrupt");
		return -EIO;
	}


    k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_ICM20948_THREAD_STACK_SIZE,
			icm20948_thread, drv_data,
			NULL, NULL, K_PRIO_COOP(CONFIG_ICM20948_THREAD_PRIORITY),
			0, K_NO_WAIT);

	oldticks = k_uptime_ticks();

	return 0;
}


