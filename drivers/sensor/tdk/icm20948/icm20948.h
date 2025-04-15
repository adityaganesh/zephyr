#ifndef ZEPHYR_DRIVERS_SENSOR_ICM20948_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM20948_H_

#include "zephyr/drivers/i2c.h"
#include <stdint.h>

struct icm20948_data {
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;

  int16_t temp;

  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;

  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;
  int16_t magn_scale_x;
  int16_t magn_scale_y;
  int16_t magn_scale_z;
};

struct icm20948_config {
  struct i2c_dt_spec i2c;
  uint8_t accel_fs;
  uint8_t gyro_fs;
  uint8_t mag_freq;
};

// TODO migrate to Kconfig
typedef enum {
  ACCEL_DLPFCFG_0 = 0 << 3,
  ACCEL_DLPFCFG_1 = 1 << 3,
  ACCEL_DLPFCFG_2 = 2 << 3,
  ACCEL_DLPFCFG_3 = 3 << 3,
  ACCEL_DLPFCFG_4 = 4 << 3,
  ACCEL_DLPFCFG_5 = 5 << 3,
  ACCEL_DLPFCFG_6 = 6 << 3,
  ACCEL_DLPFCFG_7 = 7 << 3,
} icm20948_accel_dlpfcfg;

typedef enum {
  ACCEL_FS_SEL_2G = 0,
  ACCEL_FS_SEL_4G = 1,
  ACCEL_FS_SEL_8G = 2,
  ACCEL_FS_SEL_16G = 3
} accel_fs_sel;

typedef enum { ACCEL_FCHOICE_DISABLE, ACCEL_FCHOICE_ENABLE } accel_fchoice;

// TODO migrate to Kconfig
typedef enum {
  GYRO_FS_250 = 0,
  GYRO_FS_500 = 1,
  GYRO_FS_1000 = 2,
  GYRO_FS_2000 = 3,
} gyro_fs_sel;

// TODO migrate to Kconfig
typedef enum {
  GYRO_DLPFCFG_0 = 0 << 3,
  GYRO_DLPFCFG_1 = 1 << 3,
  GYRO_DLPFCFG_2 = 2 << 3,
  GYRO_DLPFCFG_3 = 2 << 3,
  GYRO_DLPFCFG_4 = 4 << 3,
  GYRO_DLPFCFG_5 = 5 << 3,
  GYRO_DLPFCFG_6 = 6 << 3,
  GYRO_DLPFCFG_7 = 7 << 3,
} gyro_dlpfcfg;

typedef enum {
  GYRO_F_DISABLE, // Disable low pass filter
  GYRO_F_ENABLE   // Enable low pass filter
} gyro_fchoice;

typedef enum {
  AK09916_MODE_PWR_DWN = 0x00,
  AK09916_MODE_SINGLE = 0x01,
  AK09916_MODE_CONT_1 = 0x02,
  AK09916_MODE_CONT_2 = 0x04,
  AK09916_MODE_CONT_3 = 0x06,
  AK09916_MODE_CONT_4 = 0x08,
  AK09916_MODE_SELF_TEST = 0x10,
} ak09916_mode;
#endif /* ZEPHYR_DRIVERS_SENSOR_ICM20948_H_ */
