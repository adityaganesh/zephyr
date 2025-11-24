/* ST Microelectronics I3G4250D gyro driver
 *
 * Copyright (c) 2021 Jonathan Hahn
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/i3g4250d.pdf
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "i3g4250d.h"


int count = 0;
LOG_MODULE_DECLARE(i3g4250d, CONFIG_SENSOR_LOG_LEVEL);

int i3g4250d_enable_drdy_int(const struct device *dev, int enable)
{
    const struct i3g4250d_device_config *cfg = dev->config;
    stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
    i3g4250d_int1_route_t val1 = {};
    i3g4250d_int2_route_t val2 = {};
    int ret;

    val1.i1_int1 = 1;
    //val2.i2_drdy = 1;
	int16_t buf[3] = { 0 };
    ret = i3g4250d_angular_rate_raw_get(ctx, buf);

    /* set interrupt */
    if (enable) {
        ret = i3g4250d_pin_int1_route_set(ctx, val1);
    } else {
        ret = i3g4250d_pin_int2_route_get(ctx, &val2);
		if (ret < 0) {
			LOG_ERR("pint_int2_route_get error");
			return ret;
		}
        val2.i2_drdy = 1;
        ret = i3g4250d_pin_int2_route_set(ctx, val2);
    }

    return ret;
}

static void i3g4250d_gpio_callback(const struct device *dev,
				    struct gpio_callback *cb, uint32_t pins)
{
    struct i3g4250d_data *i3g4250d = dev->data;
    const struct i3g4250d_device_config *cfg = dev->config;
    stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
    int ret;
    uint8_t reg;
	int16_t buf[3] = { 0 };
    LOG_INF("INTERRUPT HIT");
	ret = i3g4250d_flag_data_ready_get(ctx, &reg);
	if (ret < 0 || reg != 1) {
		return ret;
	}

	ret = i3g4250d_angular_rate_raw_get(ctx, buf);
	if (ret < 0) {
		LOG_ERR("Failed to fetch raw data sample!");
		return ret;
	}

}

int i3g4250d_init_interrupt(const struct device *dev)
{
    const struct i3g4250d_device_config *cfg = dev->config;
    struct i3g4250d_data *i3g4250d = dev->data;
    stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
    int ret;

    i3g4250d->drdy_gpio = (cfg->drdy_pin == 1) ?
            (struct gpio_dt_spec *)&cfg->int1_gpio :
            (struct gpio_dt_spec *)&cfg->int2_gpio;



    ret = gpio_pin_configure_dt(i3g4250d->drdy_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_DBG("Could not configure gpio");
        return ret;
    }

    gpio_init_callback(&i3g4250d->gpio_cb,
            i3g4250d_gpio_callback,
            BIT(i3g4250d->drdy_gpio->pin));

    if (gpio_add_callback(i3g4250d->drdy_gpio->port, &i3g4250d->gpio_cb) < 0) {
        LOG_DBG("Could not set gpio callback");
        return -EIO;
    }

    i3g4250d_enable_drdy_int(dev, 0);


	LOG_INF("Trigger setup done");

	return gpio_pin_interrupt_configure_dt(i3g4250d->drdy_gpio,
					       GPIO_INT_EDGE_TO_ACTIVE);
}