/****************************************************************************
 * include/nuttx/sensors/bmi085.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_BMI085_H
#define __INCLUDE_NUTTX_SENSORS_BMI085_H

/****************************************************************************
* Included Files
****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#if defined(CONFIG_SENSORS_BMI085) || defined(CONFIG_SENSORS_BMI085_SCU)

/****************************************************************************
* Pre-processor Definitions
****************************************************************************/

#define BMI085_SPI_MAXFREQUENCY 10000000

/* Configuration ************************************************************/

/* Power mode */

#define BMI085_PM_SUSPEND     (0x00)
#define BMI085_PM_NORMAL      (0x01)
#define BMI085_PM_DEEPSUSPEND (0x02)

/* Output data rate */

#define BMI160_ACCEL_ODR_12_5HZ (0x01)
#define BMI085_ACCEL_ODR_25HZ   (0x02)
#define BMI085_ACCEL_ODR_50HZ   (0x03)
#define BMI085_ACCEL_ODR_100HZ  (0x04)
#define BMI085_ACCEL_ODR_200HZ  (0x05)
#define BMI085_ACCEL_ODR_400HZ  (0x06)
#define BMI085_ACCEL_ODR_800HZ  (0x07)
#define BMI085_ACCEL_ODR_1600HZ (0x08)

/* IOCTL Commands ***********************************************************/

/****************************************************************************
* Public Types
****************************************************************************/

/****************************************************************************
* struct 6-axis data
****************************************************************************/

struct accel_t
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct gyro_t
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct accel_gyro_st_s
{
  struct gyro_t  gyro;
  struct accel_t accel;
  uint32_t sensor_time;
  uint16_t sensor_temp;
};

struct spi_dev_s;
struct i2c_master_s;

/****************************************************************************
* Public Function Prototypes
****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
* Name: BMI085_register
*
* Description:
*   Register the BMI085 character device as 'devpath'
*
* Input Parameters:
*   devpath - The full path to the driver to register. E.g., "/dev/accel0"
*   dev     - An instance of the SPI or I2C interface to use to communicate
*             with BMI085
*
* Returned Value:
*   Zero (OK) on success; a negated errno value on failure.
*
****************************************************************************/

#ifdef CONFIG_SENSORS_BMI085_I2C
#  ifdef CONFIG_SENSORS_BMI085_UORB
int bmi085_register_uorb(int devno, FAR struct i2c_master_s *dev);
#  else
int bmi085_register(FAR const char *devpath, FAR struct i2c_master_s *dev);
#  endif /* CONFIG_SENSORS_BMI085_UORB */
#else /* CONFIG_BMI085_SPI */
#  ifdef CONFIG_SENSORS_BMI085_UORB
int bmi085_register_uorb(int devno, FAR struct spi_dev_s *dev);
#  else
int bmi085_register(FAR const char *devpath, FAR struct spi_dev_s *dev);
#  endif /* CONFIG_SENSORS_BMI085_UORB */
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_BMI085 */
#endif /* __INCLUDE_NUTTX_SENSORS_BMI085_H */
