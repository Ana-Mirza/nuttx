/****************************************************************************
 * dr`vers/sensors/bmi085_base.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <string.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>

#include "bmi085_base.h"

#if defined(CONFIG_SENSORS_BMI085)

/****************************************************************************
* Pre-processor Definitions
****************************************************************************/

/****************************************************************************
* Private Types
****************************************************************************/

/****************************************************************************
* Private Functions
****************************************************************************/

/****************************************************************************
* Name: bmi085_configspi
*
* Description:
*
****************************************************************************/

#ifdef CONFIG_SENSORS_BMI085_SPI
static void bmi085_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the BMI085 */

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, BMI085_SPI_MAXFREQUENCY);
}
#endif

/****************************************************************************
* Private Data
****************************************************************************/

/****************************************************************************
* Public Functions
****************************************************************************/

/****************************************************************************
* Name: bmi085_getreg8
*
* Description:
*   Read from an 8-bit BMI085 register
*
****************************************************************************/

uint8_t bmi085_getreg8(FAR struct bmi085_dev_s *priv, uint8_t i2c_addr, uint8_t regaddr)
{
  uint8_t regval = 0;

#ifdef CONFIG_SENSORS_BMI085_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->config->freq;
  msg[0].addr      = i2c_addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->config->freq;
  msg[1].addr      = i2c_addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &regval;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }
#endif

  return regval;
}

/****************************************************************************
* Name: bmi085_putreg8
*
* Description:
*   Write a value to an 8-bit BMI085 register
*
****************************************************************************/

void bmi085_putreg8(FAR struct bmi085_dev_s *priv, uint8_t i2c_addr, uint8_t regaddr,
                    uint8_t regval)
{
#ifdef CONFIG_SENSORS_BMI085_I2C
  struct i2c_msg_s msg[2];
  int ret;
  uint8_t txbuffer[2];

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = priv->config->freq;
  msg[0].addr      = i2c_addr;
  msg[0].flags     = 0;
  msg[0].buffer    = txbuffer;
  msg[0].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }
#endif
}
/****************************************************************************
* Name: bmi085_getreg16
*
* Description:
*   Read 16-bits of data from an BMI085 register
*
****************************************************************************/

uint16_t bmi085_getreg16(FAR struct bmi085_dev_s *priv, uint8_t i2c_addr, uint8_t regaddr)
{
  uint16_t regval = 0;

#ifdef CONFIG_SENSORS_BMI085_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->config->freq;
  msg[0].addr      = i2c_addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->config->freq;
  msg[1].addr      = i2c_addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = (FAR uint8_t *)&regval;
  msg[1].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }
#endif

  return regval;
}

/****************************************************************************
* Name: bmi085_getregs
*
* Description:
*   Read cnt bytes from specified dev_addr and reg_addr
*
****************************************************************************/

void bmi085_getregs(FAR struct bmi085_dev_s *priv, uint8_t i2c_addr, uint8_t regaddr,
                    uint8_t *regval, int len)
{
#ifdef CONFIG_SENSORS_BMI085_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->config->freq;
  msg[0].addr      = i2c_addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->config->freq;
  msg[1].addr      = i2c_addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = len;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

#endif
}

/****************************************************************************
* Name: bmi085_checkid
*
* Description:
*   Read and verify the BMI085 chip ID
*
****************************************************************************/

int bmi085_checkid(FAR struct bmi085_dev_s *priv)
{
  uint8_t devid = 0;

  /* Read Accelerometer device ID */

  devid = bmi085_getreg8(priv, priv->config->acc_addr, ACCEL_CHIP_ID_ADDR);
  sninfo("acc devid: %04x\n", devid);

  if (devid != (uint16_t) ACC_DEVID)
    {
      /* ID is not Correct */
      return -ENODEV;
    }

  /* Read Gyro device ID */

  devid = bmi085_getreg8(priv, priv->config->gyro_addr, GYRO_CHIP_ID_ADDR);
  sninfo("gyro devid: %04x\n", devid);

  if (devid != (uint16_t) GYRO_DEVID)
    {
      /* ID is not Correct */
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: bmi085_set_normal_imu
 *
 * Description:
 *   set bmi085 to normal IMU mode.
 *
 ****************************************************************************/

void bmi085_set_normal_imu(FAR struct bmi085_dev_s *priv) 
{
  /* Set accel & gyro as normal mode. */
  bmi085_putreg8(priv, priv->config->acc_addr, ACCEL_PWR_CNTRL_ADDR, ACCEL_ENABLE_CMD);
  up_mdelay(30);
  bmi085_putreg8(priv, priv->config->gyro_addr, GYRO_LPM1, GYRO_PWR_NORMAL);
  up_mdelay(30);

  /* Set accel & gyro output data rate. */
  bmi085_putreg8(priv, priv->config->acc_addr, ACCEL_ODR_ADDR,
      ACCEL_NORMAL_AVG4 | ACCEL_ODR_50_HZ);
  bmi085_putreg8(priv, priv->config->gyro_addr, GYRO_ODR_ADDR,
      GYRO_ODR_100HZ_BW_32HZ);
}

/****************************************************************************
 * Name: bmi085_data_read
 *
 * Description:
 *   get bmi085 acc and gyro data.
 *
 ****************************************************************************/

void bmi085_data_read(FAR struct bmi085_dev_s *priv, FAR struct accel_gyro_st_s *p)
{
  /* Get exclusive access to the driver data structure */

  int ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      sninfo("Mutex lock failed for data read.\n");
      return;
    }

  /* Read accelerometer and time data. */
  bmi085_acc_read(priv, p);

  /* Read temperature data. */
  bmi085_temp_read(priv, p);

  /* Read gyro data. */
  bmi085_gyro_read(priv, p);

  /* Unlock driver data structure */
  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: bmi085_acc_read
 *
 * Description:
 *   get bmi085 acc and time data.
 *
 ****************************************************************************/

void bmi085_acc_read(FAR struct bmi085_dev_s *priv, FAR struct accel_gyro_st_s *p)
{
  int16_t data[3];
  u_int8_t acc_data[9];
  FAR struct accel_t *accel_p = &(p->accel);
  uint32_t *sensor_time = &(p->sensor_time);

  bmi085_getregs(priv, priv->config->acc_addr, ACCEL_ACCEL_DATA_ADDR, acc_data, 9);

  data[0] = (int16_t)(acc_data[1] << 8) | acc_data[0];
  data[1] = (int16_t)(acc_data[3] << 8) | acc_data[2];
  data[2] = (int16_t)(acc_data[5] << 8) | acc_data[4];

  uint8_t range = bmi085_getreg8(priv, priv->config->acc_addr, ACCEL_RANGE_ADDR);
  uint16_t accel_range_mg = 1 << (range + 1);
  sninfo("Data 16-bit ACC_RANGE_MG--->: %d\n", accel_range_mg);

  float scale = (float)accel_range_mg * 1000 / 32768.0f;
  accel_p->x = data[0] * scale;
  accel_p->y = data[1] * scale;
  accel_p->z = data[2] * scale;

  sninfo("Data 16-bit ACC_X--->: %d mg\n", data[0]);
  sninfo("Data 16-bit ACC_Y--->: %d mg\n", data[1]);
  sninfo("Data 16-bit ACC_Z--->: %d mg\n", data[2]);

  /* Time data */
  *sensor_time = (acc_data[8] << 16) | (acc_data[7] << 8) | acc_data[6];
}

/****************************************************************************
 * Name: bmi085_gyro_read
 *
 * Description:
 *   get bmi085 gyro data.
 *
 ****************************************************************************/

void bmi085_gyro_read(FAR struct bmi085_dev_s *priv, FAR struct accel_gyro_st_s *p)
{
  int16_t data_g[3];
  u_int8_t gyro_data[6];
  FAR struct gyro_t *gyro_p = &(p->gyro);

  bmi085_getregs(priv, priv->config->gyro_addr, GYRO_DATA_ADDR, gyro_data, 6);

  data_g[0] = (int16_t)(gyro_data[1] << 8) | gyro_data[0];
  data_g[1] = (int16_t)(gyro_data[3] << 8) | gyro_data[2];
  data_g[2] = (int16_t)(gyro_data[5] << 8) | gyro_data[4];

  float gyro_scale = 2000.0f / 32768.0f * D2R;
  gyro_p->x = data_g[0] * gyro_scale;
  gyro_p->y = data_g[1] * gyro_scale;
  gyro_p->z = data_g[2] * gyro_scale;

  sninfo("Data 16-bit GYRO_X--->: %d\n", gyro_p->x);
  sninfo("Data 16-bit GYRO_Y--->: %d\n", gyro_p->y);
  sninfo("Data 16-bit GYRO_Z--->: %d\n", gyro_p->z);
}

/****************************************************************************
 * Name: bmi085_data_read
 *
 * Description:
 *   get bmi085 acc and gyro data.
 *
 ****************************************************************************/

void bmi085_temp_read(FAR struct bmi085_dev_s *priv, FAR struct accel_gyro_st_s *p)
{
  int16_t temp_int11;
  u_int8_t acc_data[9];
  uint16_t *sensor_temp = &(p->sensor_temp); 

  bmi085_getregs(priv, priv->config->acc_addr, ACCEL_TEMP_DATA_ADDR, acc_data, 2);
  uint16_t temp_uint11 = (acc_data[0] * 8) + (acc_data[1] / 32);

  if (temp_uint11 > 1023) {
    temp_int11 = temp_uint11 - 2048;
  } else {
    temp_int11 = temp_uint11;
  }

  *sensor_temp = (float) temp_int11 * 0.125f + 23.0f;
}

/****************************************************************************
 * Name: bmi085_enable_irq
 *
 * Description:
 *   enable bmi085 gyro interrupts.
 *
 ****************************************************************************/

void bmi085_enable_irq(FAR struct bmi085_dev_s *priv, bool enable) 
{
  /* Set pin modes INT1*/
  if (enable) {

    /* Configure acc output pin INT1*/
    bmi085_pin_mode_int1(priv);

    /* Map pins for INT1 */
    bmi085_accel_map_int1(priv);

    priv->status = ACTIVITY;

    sninfo("bmi085_enable_irq: eabled interrupts.\n");
  }
}

/****************************************************************************
 * Name: bmi085_status_irq
 *
 * Description:
 *   Returns interrupt status flag.
 *
 ****************************************************************************/
int bmi085_status_irq(FAR struct bmi085_dev_s *priv) 
{
  return (int)priv->status;
}

/****************************************************************************
 * Name: bmi085_set_data
 *
 * Description:
 *   Sets user buffer pointing to the low level data buffer.
 *
 ****************************************************************************/
void bmi085_set_data(FAR struct bmi085_dev_s *priv, FAR struct accel_gyro_st_s *p)
{
  p = &priv->sample;
}

/****************************************************************************
 * Name: bmi085_pin_mode_int1
 *
 * Description:
 *   Configures int1 line for interrupts.
 *
 ****************************************************************************/
void bmi085_pin_mode_int1(FAR struct bmi085_dev_s *priv) 
{
  uint8_t write_reg = 0;
  uint8_t read_reg = 0;

  /* Set interrupt configs */
  read_reg = bmi085_getreg8(priv, priv->config->acc_addr, ACCEL_INT1_IO_CTRL_ADDR);
  write_reg = SET_FIELD(read_reg,ACCEL_INT1_IO_CTRL,
    (ACCEL_INT_OUTPUT | ACCEL_INT_PUSHPULL | ACCEL_INT_LVL_HIGH));

  /* Write back the configs */
  bmi085_putreg8(priv, priv->config->acc_addr, ACCEL_INT1_IO_CTRL_ADDR, write_reg);

  sninfo("pin mode int1 activated.\n");
}

/****************************************************************************
 * Name: bmi085_set_data
 *
 * Description:
 *   Maps int1 line for interrupts.
 *
 ****************************************************************************/
void bmi085_accel_map_int1(FAR struct bmi085_dev_s *priv)
{
  uint8_t write_reg = 0;
  write_reg = bmi085_getreg8(priv, priv->config->acc_addr, ACCEL_INT1_DRDY_ADDR);

  /* Set pin 2 to value 0x00 */
  write_reg = SET_FIELD(write_reg,ACCEL_INT1_DRDY,1);

  bmi085_putreg8(priv, priv->config->acc_addr, ACCEL_INT1_DRDY_ADDR, write_reg);
  up_mdelay(10);

  sninfo("int1 mapped.\n");
}

#endif /* CONFIG_SENSORS_BMI085 */
