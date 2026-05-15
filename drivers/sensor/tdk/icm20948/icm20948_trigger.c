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

static void icm20948_gpio_callback(const struct device *dev,
				  struct gpio_callback *cb, uint32_t pins)
{
	struct icm20948_data *drv_data =
		CONTAINER_OF(cb, struct icm20948_data, gpio_cb);
	const struct icm20948_config *cfg = drv_data->dev->config;
    uint8_t ready_to_read = 0;

    gpio_pin_interrupt_configure_dt(&cfg->int_pin, GPIO_INT_DISABLE);
	k_sem_give(&drv_data->gpio_sem);

}

static void icm20948_thread_cb(const struct device *dev)
{
	struct icm20948_data *drv_data = dev->data;
	const struct icm20948_config *cfg = dev->config;

	// if (drv_data->data_ready_handler != NULL) {
	// 	drv_data->data_ready_handler(dev,
	// 				     drv_data->data_ready_trigger);
	// }
    uint8_t ready_to_read = 0;

	int ret = i2c_reg_read_byte_dt(&cfg->i2c, ICM20948_REG_INT_STATUS_1, &ready_to_read);

	if (ret) {
		LOG_ERR("data not ready to read.\n");
	}

    LOG_INF("Interrupt received from pin %d", cfg->int_pin.pin);

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
	drv_data->dev = dev;
	if (!gpio_is_ready_dt(&cfg->int_pin)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

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


	*/
	if (i2c_reg_write_byte_dt(&cfg->i2c, ICM20948_REG_INT_PIN_CFG,
				  ICM20948_INT1_LATCH_EN|ICM20948_INT1_ANYRD_2CLEAR|ICM20948_BYPASS_EN) < 0) {
		LOG_ERR("Failed to enable data ready interrupt.");
		return -EIO;
	}


	/* enable data ready interrupt */
	if (i2c_reg_write_byte_dt(&cfg->i2c, ICM20948_REG_INT_ENABLE_1,
				  0x01) < 0) {
		LOG_ERR("Failed to enable data ready interrupt.");
		return -EIO;
	}


    k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_ICM20948_THREAD_STACK_SIZE,
			icm20948_thread, drv_data,
			NULL, NULL, K_PRIO_COOP(CONFIG_ICM20948_THREAD_PRIORITY),
			0, K_NO_WAIT);



    gpio_pin_interrupt_configure_dt(&cfg->int_pin,
					GPIO_INT_EDGE_TO_ACTIVE);

	return 0;
}


