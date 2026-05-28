#ifndef ZEPHYR_DRIVERS_SENSOR_TDK_ICM20948_ICM20948_RTIO_H_
#define ZEPHYR_DRIVERS_SENSOR_TDK_ICM20948_ICM20948_RTIO_H_

#include <zephyr/device.h>
#include <zephyr/rtio/rtio.h>
#include "icm20948_bus.h"
#include "icm20948.h"
void icm20948_submit(const struct device *sensor, struct rtio_iodev_sqe *iodev_sqe);

void icm20948_fifo_event(const struct device *dev);

#endif