/****************************************************************************
 * drivers/sensors/bmi085.c
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

/* Macros to get and set register fields */
#define GET_FIELD(regname,value) ((value & regname##_MASK) >> regname##_POS)
#define	SET_FIELD(regval,regname,value) ((regval & ~regname##_MASK) | ((value << regname##_POS) & regname##_MASK))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Character driver methods */

static int     bmi085_open(FAR struct file *filep);
static int     bmi085_close(FAR struct file *filep);
static ssize_t bmi085_read(FAR struct file *filep, FAR char *buffer,
                            size_t len);
static int     bmi085_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_bmi085fops =
{
bmi085_open,     /* open */
bmi085_close,    /* close */
bmi085_read,     /* read */
NULL,            /* write */
NULL,            /* seek */
bmi085_ioctl,    /* ioctl */
};

/****************************************************************************
 * Name: bmi085_set_normal_imu
 *
 * Description:
 *   set bmi085 to normal IMU mode.
 *
 ****************************************************************************/

static void bmi085_set_normal_imu(FAR struct bmi085_dev_s *priv)
{
/* Set accel & gyro as normal mode. */

/* Set accel & gyro output data rate. */

}

/****************************************************************************
 * Name: bmi085_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int bmi085_open(FAR struct file *filep)
{
FAR struct inode        *inode = filep->f_inode;
FAR struct bmi085_dev_s *priv  = inode->i_private;

bmi085_set_normal_imu(priv);

return OK;
}

/****************************************************************************
 * Name: bmi085_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int bmi085_close(FAR struct file *filep)
{
FAR struct inode        *inode = filep->f_inode;
FAR struct bmi085_dev_s *priv  = inode->i_private;

/* Set suspend mode to each sensors. */

return OK;
}

/****************************************************************************
 * Name: bmi085_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t bmi085_read(FAR struct file *filep, FAR char *buffer,
                        size_t len)
{
FAR struct inode        *inode = filep->f_inode;
FAR struct bmi085_dev_s *priv  = inode->i_private;
FAR struct accel_gyro_st_s *p = (FAR struct accel_gyro_st_s *)buffer;

if (len < sizeof(struct accel_gyro_st_s))
    {
    snerr("Expected buffer size is %zu\n", sizeof(struct accel_gyro_st_s));
    return 0;
    }

/* Adjust sensing time into 24 bit */

p->sensor_time >>= 8;

return len;
}

static void bmi085_enable_stepcounter(FAR struct bmi085_dev_s *priv,
                                    int enable)
{
uint8_t val;

}

/****************************************************************************
 * Name: bmi085_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int bmi085_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
FAR struct inode        *inode = filep->f_inode;
FAR struct bmi085_dev_s *priv  = inode->i_private;
int ret = OK;

return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi085_register
 *
 * Description:
 *   Register the BMI085 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *   dev     - An instance of the SPI interface to use to communicate with
 *             BMI085
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI085_I2C
int bmi085_register(FAR const char *devpath, FAR struct i2c_master_s *dev)
#else /* CONFIG_SENSORS_BMI085_SPI */
int bmi085_register(FAR const char *devpath, FAR struct spi_dev_s *dev)
#endif
{
FAR struct bmi085_dev_s *priv;
int ret;

priv = (FAR struct bmi085_dev_s *)kmm_malloc(sizeof(struct bmi085_dev_s));
if (!priv)
    {
    snerr("Failed to allocate instance\n");
    return -ENOMEM;
    }

#ifdef CONFIG_SENSORS_BMI085_I2C
priv->i2c = dev;
priv->acc_addr = BMI085_ACC_I2C_ADDR;
priv->gyro_addr = BMI085_GYRO_I2C_ADDR;
priv->freq = BMI085_I2C_FREQ;
#endif

ret = bmi085_checkid(priv);
if (ret < 0)
    {
    snerr("Wrong Device ID!\n");
    kmm_free(priv);
    return ret;
    }

ret = register_driver(devpath, &g_bmi085fops, 0666, priv);
if (ret < 0)
    {
    snerr("Failed to register driver: %d\n", ret);
    kmm_free(priv);
    }

sninfo("BMI085 driver loaded successfully!\n");
return OK;
}

#endif /* CONFIG_SENSORS_BMI085 */
