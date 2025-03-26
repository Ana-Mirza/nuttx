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
 #include <assert.h>
 #include <nuttx/arch.h>
 #include <nuttx/board.h>
 
 #include <nuttx/i2c/i2c_master.h>
 #include <nuttx/sensors/bmi085.h>

 #include "esp32s3_i2c.h"
 #include "esp32s3-hectorwatch.h"
 
 #if defined(CONFIG_ESP32S3_I2C) && defined(CONFIG_SENSORS_BMI085)
 
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
  *   devno - The device number, used to build the device path as /dev/pressN
  *   busno - The I2C bus number
  *
  * Returned Value:
  *   Zero (OK) on success; a negated errno value on failure.
  *
  ****************************************************************************/
 
 int esp32s3_bmi085_initialize(int devno, int busno)
 {
    struct i2c_master_s *i2c;
    char devpath[14];
  
    /* Initialize i2c bus */
  
    i2c = esp32s3_i2cbus_initialize(busno);
    if (i2c == NULL)
      {
        return -ENODEV;
      }
  
    /* Register the bmi085 sensor */
  
    (void)snprintf(devpath, sizeof(devpath), "/dev/bmi085_%d", devno);
  
    return bmi085_register(devpath, i2c);
 }
 
 #endif /* CONFIG_BOARDCTL_BMI085 */
 