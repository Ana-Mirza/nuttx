/****************************************************************************
 * /boards/custom-boards/esp32s3-hectorwatch/src/esp32s3_bmi085.c
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

#include <stdlib.h>
#include <debug.h>
#include <stdio.h>
#include <stdbool.h>

#include <assert.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

#include "esp32s3_i2c.h"
#include "esp32s3_gpio.h"
#include "hardware/esp32s3_gpio_sigmap.h"

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bmi085.h>

#include "esp32s3-hectorwatch.h"

/****************************************************************************
* Public Functions
****************************************************************************/

/****************************************************************************
* Name: esp32s3_bmi085_initialize
*
* Description:
*   Initialize and register the BMI085 driver.
*
* Input Parameters:
*   busno - The I2C bus number
*
* Returned Value:
*   Zero (OK) on success; a negated errno value on failure.
*
****************************************************************************/
struct bmi085_config_s *config;

static int bmi085_attach(FAR struct bmi085_config_s *config,
  bmi085_handler_t handler,
  FAR void *arg)
{

  int irq = ESP32S3_PIN2IRQ(IMU_INT1);

  esp32s3_gpioirqdisable(irq);

  int ret = irq_attach(irq, (xcpt_t)handler, arg);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: irq_attach() failed: %d\n", ret);
      return ret;
    }

  /* Configure for rising edge (IMU active HIGH) */
  esp32s3_gpioirqenable(irq, RISING);
  sninfo("bmi085_attach: IMU_INT1 attached.\n");

  return OK;
}

static void bmi085_enable(FAR struct bmi085_config_s *config, bool enable)
{
  int irq = ESP32S3_PIN2IRQ(IMU_INT1);
  if (enable) {
    esp32s3_gpioirqenable(irq, RISING);
    sninfo("bmi085_enable: IMU_INT1 enabled.\n");
  } else {
    esp32s3_gpioirqdisable(irq);
    sninfo("bmi085_enable: IMU_INT1 enabled.\n");
  }
}

static void bmi085_clear(FAR struct bmi085_config_s *config)
{
  // No specific action needed to clear interrupt
}

int esp32s3_bmi085_initialize(int busno)
{
  struct i2c_master_s *i2c;
  char devpath[12];

  /* Initialize i2c bus */

  sninfo("Initializing BMI085!\n");

  i2c = esp32s3_i2cbus_initialize(busno);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  /* Create config structure */
  config = (struct bmi085_config_s *)kmm_malloc(sizeof(struct bmi085_config_s));

  /* Assign interrupt handlers */
  config->attach = bmi085_attach;
  config->enable = bmi085_enable;
  config->clear = bmi085_clear;

  /* Configure GPIO */
  esp32s3_configgpio(IMU_INT1, INPUT_FUNCTION_2 | PULLUP);

  (void)snprintf(devpath, sizeof(devpath), "/dev/bmi085");

  /* Register the bmi085 sensor */
  return bmi085_register(devpath, i2c, config);
}

/* CONFIG_BOARDCTL_BMI085 */
