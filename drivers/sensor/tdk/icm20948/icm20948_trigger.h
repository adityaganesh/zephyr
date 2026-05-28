#ifndef ZEPHYR_DRIVERS_SENSOR_TDK_ICM20948_TRIGGER_H_
#define ZEPHYR_DRIVERS_SENSOR_TDK_ICM20948_TRIGGER_H_

#include "icm20948.h"
#include "icm20948_rtio.h"
#ifdef CONFIG_ICM20948_TRIGGER
int icm20948_init_interrupt(const struct device *dev);
#endif /* CONFIG_ICM20948_TRIGGER */
#endif