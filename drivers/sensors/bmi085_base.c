/****************************************************************************
 * drivers/sensors/bmi085_base.c
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

uint8_t bmi085_getreg8(FAR struct bmi085_dev_s *priv, uint8_t regaddr)
{
  uint8_t regval = 0;

#ifdef CONFIG_SENSORS_BMI085_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->accel_addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->accel_addr;
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

void bmi085_putreg8(FAR struct bmi085_dev_s *priv, uint8_t regaddr,
                    uint8_t regval)
{
#ifdef CONFIG_SENSORS_BMI085_I2C
  struct i2c_msg_s msg[2];
  int ret;
  uint8_t txbuffer[2];

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->accel_addr;
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

uint16_t bmi085_getreg16(FAR struct bmi085_dev_s *priv, uint8_t regaddr)
{
  uint16_t regval = 0;

#ifdef CONFIG_SENSORS_BMI085_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->accel_addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->accel_addr;
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

void bmi085_getregs(FAR struct bmi085_dev_s *priv, uint8_t regaddr,
                    uint8_t *regval, int len)
{
#ifdef CONFIG_SENSORS_BMI085_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->accel_addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->accel_addr;
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

  /* Read device ID  */

  devid = bmi085_getreg8(priv, ACCEL_CHIP_ID_ADDR);
  sninfo("devid: %04x\n", devid);

  if (devid != (uint16_t) DEVID)
    {
      /* ID is not Correct */

      return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_SENSORS_BMI085 */
