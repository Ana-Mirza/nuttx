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
// #include <nuttx/sensors/bmi085.h>

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
  priv->status = INACTIVITY;

  /* Set accel and gyro mode. */
  bmi085_set_normal_imu(priv);

  sninfo("BMI085 sensor activated.\n");
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
  bmi085_putreg8(priv, priv->config->acc_addr, ACCEL_PWR_CNTRL_ADDR, ACCEL_DISABLE_CMD);
  up_mdelay(30);

  bmi085_putreg8(priv, priv->config->gyro_addr, GYRO_LPM1, GYRO_PWR_SUSPEND);
  up_mdelay(30);

  sninfo("BMI085 sensor suspended.\n");
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

  if (priv->status == ACTIVITY) {
    p = &priv->sample;
    return len;
  }

  sninfo("bmi085_read: reading data.\n");
  bmi085_data_read(priv, p);

  return len;
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

  switch (cmd)
    {
      /* Enable bmi160 interrups. Arg: bool value */

      case SNIOC_ENABLEIRQ:
        {
          bmi085_enable_irq(priv, (bool)arg);
        }
        break;

      case SNIOC_STATUSIRQ:
        {
          return bmi085_status_irq(priv);
        }

      case SNIOC_DATAIRQ:
        {
          bmi085_set_data(priv, (FAR struct accel_gyro_st_s *)arg);
        }
        break;

      default:
        snerr("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

 /****************************************************************************
 * Name: bmi085_worker
 *
 * Description:
 *   This is the "bottom half" of the BMI085 interrupt handler
 *
 ****************************************************************************/
static void bmi085_worker(FAR void *arg)
{
  FAR struct bmi085_dev_s *priv = (FAR struct bmi085_dev_s *)arg;
  uint8_t regval;

  sninfo("bmi085_worker: BMI085 worker.\n");

  DEBUGASSERT(priv && priv->config);

  /* Get the global interrupt status */

  regval =  bmi085_getreg8(priv, priv->config->acc_addr, ACCEL_INT_STAT_1);
  // regval =  bmi085_getreg8(priv, priv->config->acc_addr, ACCEL_DRDY_ADDR);

  /* Check for a data ready interrupt */

  sninfo("BMI085 worker checking.\n");
  if ((regval & INT_DATA_READY) != 0)
    {
      /* Read accelerometer and time data. */
      bmi085_acc_read(priv, &priv->sample);
      sninfo("BMI085 data ready.\n");

      if (priv->status == DATA_READY) {
        priv->status = OVERRUN;
        sninfo("bmi085_interrupt: status - overrun (%d).\n", priv->status);
      } else {
        priv->status = DATA_READY;
        sninfo("bmi085_interrupt: status - data ready (%d).\n", priv->status);
      }
    }

  /* Re-enable the BMI085 GPIO interrupt */

  priv->config->enable(priv->config, true);
}

/****************************************************************************
 * Name: bmi085_interrupt
 *
 * Description:
 *  The BMI085 interrupt handler
 *
 ****************************************************************************/

static void bmi085_interrupt(int irq, void *context,
  FAR void *arg)
{
  FAR struct bmi085_dev_s *priv = (FAR struct bmi085_dev_s *)arg;
  FAR struct bmi085_config_s *config = priv->config;
  int ret;

  /* Disable further interrupts */

  config->enable(config, false);

  /* Check if interrupt work is already queue.  If it is already busy, then
   * we already have interrupt processing in the pipeline and we need to do
   * nothing more.
   */

  if (work_available(&priv->work))
    {
      /* Transfer processing to the worker thread. Since BMI085
       * interrupts are disabled while the work is pending, no special
       * action should be required to protect the work queue.
       */

      sninfo("bmi085_interrupt: work queued.\n");
      ret = work_queue(HPWORK, &priv->work, (worker_t)bmi085_worker, priv, 0);
      if (ret != 0)
        {
          snerr("ERROR: Failed to queue work: %d\n", ret);
        }
    }

  sninfo("bmi085_interrupt: ended.\n");

  /* Clear any pending interrupts and return success */
  config->clear(config);
}

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
int bmi085_register(FAR const char *devpath, FAR struct i2c_master_s *dev, 
                    FAR struct bmi085_config_s *config)
#else /* CONFIG_SENSORS_BMI085_SPI */
int bmi085_register(FAR const char *devpath, FAR struct spi_dev_s *dev,
                    FAR struct bmi085_config_s *config)
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

  memset(&priv->work, 0, sizeof(struct work_s));

  /* Initialize the device state structure */
  nxmutex_init(&priv->lock);
  priv->config = config;

  #ifdef CONFIG_SENSORS_BMI085_I2C
  priv->i2c = dev;
  priv->config->acc_addr = BMI085_ACC_I2C_ADDR;
  priv->config->gyro_addr = BMI085_GYRO_I2C_ADDR;
  priv->config->freq = BMI085_I2C_FREQ;
  #endif

  /* Read and verify the BMI085 device ID */
  ret = bmi085_checkid(priv);
  if (ret < 0)
    {
    snerr("Wrong Device ID!\n");
    kmm_free(priv);
    return ret;
    }

  /* Attach the BMI085 gpio callbacks. */
  priv->config->attach(config,
    (bmi085_handler_t)bmi085_interrupt,
    (FAR void *)priv);
  priv->config->clear(config);
  priv->config->enable(config, true);
  sninfo("BMI085: attached and enabled irq.\n");

  /* Get exclusive access to the device structure */
  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      snerr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Register the character driver */
  ret = register_driver(devpath, &g_bmi085fops, 0666, priv);
  if (ret < 0)
    {
    snerr("Failed to register driver: %d\n", ret);
    kmm_free(priv);
    }

  /* Accelerometer is initialized */
  priv->status |= BMI085_STAT_INITIALIZED;
  nxmutex_unlock(&priv->lock);

  sninfo("BMI085 driver loaded successfully!\n");
  return OK;
}

#endif /* CONFIG_SENSORS_BMI085 */
