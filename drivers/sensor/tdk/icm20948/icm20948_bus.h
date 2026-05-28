#ifndef ZEPHYR_DRIVERS_SENSOR_ICM20948_BUS_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM20948_BUS_H_

#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>

struct icm20948_bus {
    struct rtio *ctx;
    struct rtio_iodev *iodev;
};



#endif /* ZEPHYR_DRIVERS_SENSOR_ICM20948_BUS_H_ */