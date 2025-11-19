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

LOG_MODULE_REGISTER(i3g4250d, CONFIG_SENSOR_LOG_LEVEL);

int lsm6dsv16x_init_interrupt(const struct device *dev)
{
    const struct i3g4250d_device_config *cfg = dev->config;
    stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
    int ret;

    /* Enable interrupt pin */
    ret = i3g4250d_pin_int1_route_set(ctx, I3G4250D_INT1_DRDY_ENABLE);
    if (ret != 0) {
        LOG_ERR("Failed setting drdy interrupt");
        return ret;
    }

    ret = i3g4250d_pin_int2_route_set(ctx, I3G4250D_INT2_DRDY_ENABLE);
    if (ret != 0) {
        LOG_ERR("Failed setting drdy interrupt");
        return ret;
    }

    return 0;
}