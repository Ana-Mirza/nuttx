/****************************************************************************
 * drivers/sensors/bmi085_base.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BMI085_COMMOM_H
#define __INCLUDE_NUTTX_SENSORS_BMI085_COMMOM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bmi085.h>

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <fixedmath.h>
#include <math.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ACC_DEVID           0x1f
#define GYRO_DEVID          0x0f

/* I2C  Address
*
* NOTE: If SDO pin is pulled to VDDIO, use 0x69 for gyroscope
*       and 0x19 for accelerometer
*/

#ifdef CONFIG_BMI085_I2C_ADDR_68
#define BMI085_GYRO_I2C_ADDR    0x68
#define BMI085_ACC_I2C_ADDR     0x18
#else
#define BMI085_GYRO_I2C_ADDR    0x69
#define BMI085_ACC_I2C_ADDR     0x19
#endif

#define BMI085_I2C_FREQ     400000

/****************************************************************************
 * Accelerometer Definitions
 ****************************************************************************/

/* Register 0x40 - ACCEL_CONFIG accel bandwidth and ODR */

#define ACCEL_OSR4           (8 << 4)
#define ACCEL_OSR2           (9 << 4)
#define ACCEL_NORMAL_AVG4    (10 << 4)

#define ACCEL_ODR_12_5HZ    0x05
#define ACCEL_ODR_25_HZ     0x06
#define ACCEL_ODR_50_HZ    0x07
#define ACCEL_ODR_100_HZ    0x08
#define ACCEL_ODR_200_HZ    0x09
#define ACCEL_ODR_400_HZ    0x0A
#define ACCEL_ODR_800_HZ    0x0B
#define ACCEL_ODR_1600_HZ   0x0C

/* Accelerometer Range */
#define ACCEL_RANGE_2G   0x00
#define ACCEL_RANGE_4G   0x01
#define ACCEL_RANGE_8G   0x02
#define ACCEL_RANGE_16G  0x03

/* Constants */
#define ACCEL_CHIP_ID           0x1F
#define ACCEL_RESET_CMD         0xB6
#define ACCEL_ENABLE_CMD        0x04
#define ACCEL_DISABLE_CMD       0x00
#define ACCEL_SUSPEND_MODE_CMD  0x03
#define ACCEL_ACTIVE_MODE_CMD   0x00
#define ACCEL_INT_INPUT         0x11
#define ACCEL_INT_OUTPUT        0x08
#define ACCEL_INT_OPENDRAIN     0x04
#define ACCEL_INT_PUSHPULL      0x00
#define ACCEL_INT_LVL_HIGH      0x02
#define ACCEL_INT_LVL_LOW       0x00
#define ACCEL_POS_SELF_TEST     0x0D
#define ACCEL_NEG_SELF_TEST     0x09
#define ACCEL_DIS_SELF_TEST     0x00

/*  Accellerometer registers */
#define ACCEL_CHIP_ID_ADDR          0x00
#define ACCEL_CHIP_ID_MASK          0xFF
#define ACCEL_CHIP_ID_POS           0

#define ACCEL_FATAL_ERR_ADDR        0x02
#define ACCEL_FATAL_ERR_MASK        0x01
#define ACCEL_FATAL_ERR_POS         0

#define ACCEL_ERR_CODE_ADDR         0x02
#define ACCEL_ERR_CODE_MASK         0x1C
#define ACCEL_ERR_CODE_POS          2

#define ACCEL_DRDY_ADDR             0x03
#define ACCEL_DRDY_MASK             0x80
#define ACCEL_DRDY_POS              7

#define ACCEL_ODR_ADDR              0x40
#define ACCEL_ODR_MASK              0xFF
#define ACCEL_ODR_POS               0

#define ACCEL_RANGE_ADDR            0x41
#define ACCEL_RANGE_MASK            0x03
#define ACCEL_RANGE_POS             0

#define ACCEL_INT1_IO_CTRL_ADDR     0x53
#define ACCEL_INT1_IO_CTRL_MASK     0x1F
#define ACCEL_INT1_IO_CTRL_POS      0

#define ACCEL_INT2_IO_CTRL_ADDR     0x54
#define ACCEL_INT2_IO_CTRL_MASK     0x1F
#define ACCEL_INT2_IO_CTRL_POS      0

#define ACCEL_INT1_DRDY_ADDR        0x58
#define ACCEL_INT1_DRDY_MASK        0x04
#define ACCEL_INT1_DRDY_POS         2

#define ACCEL_INT2_DRDY_ADDR        0x58
#define ACCEL_INT2_DRDY_MASK        0x40
#define ACCEL_INT2_DRDY_POS         6

#define ACCEL_SELF_TEST_ADDR        0x6D
#define ACCEL_SELF_TEST_MASK        0xFF
#define ACCEL_SELF_TEST_POS         0

#define ACCEL_PWR_CONF_ADDR         0x7C
#define ACCEL_PWR_CONF_MASK         0xFF
#define ACCEL_PWR_CONF_POS          0

#define ACCEL_PWR_CNTRL_ADDR        0x7D
#define ACCEL_PWR_CNTRL_MASK        0xFF
#define ACCEL_PWR_CNTRL_POS         0

#define ACCEL_SOFT_RESET_ADDR       0x7E
#define ACCEL_SOFT_RESET_MASK       0xFF
#define ACCEL_SOFT_RESET_POS        0

#define ACCEL_ACCEL_DATA_ADDR       0x12
#define ACCEL_TEMP_DATA_ADDR        0x22

/* Convert G to m/s/s */
#define G 9.807f

/****************************************************************************
 * Gyroscope Definitions
 ****************************************************************************/

/* Register 0x42 - GYRO_CONFIG accel bandwidth */

#define GYRO_RANGE_2000DPS  0x00
#define GYRO_RANGE_1000DPS  0x01
#define GYRO_RANGE_500DPS   0x02
#define GYRO_RANGE_250DPS   0x03
#define GYRO_RANGE_125DPS   0x04

#define GYRO_ODR_2000HZ_BW_532HZ  0x80
#define GYRO_ODR_2000HZ_BW_230HZ  0x81
#define GYRO_ODR_1000HZ_BW_116HZ  0x82
#define GYRO_ODR_400HZ_BW_47HZ    0x83
#define GYRO_ODR_200HZ_BW_23HZ    0x84
#define GYRO_ODR_100HZ_BW_12HZ    0x85
#define GYRO_ODR_200HZ_BW_64HZ    0x86
#define GYRO_ODR_100HZ_BW_32HZ    0x87

/* Register 0x11 - CMD */

#define GYRO_PWR_NORMAL        0x00
#define GYRO_PWR_SUSPEND       0x80
#define GYRO_PWR_DEEP_SUSPEND  0x20

/* Constants */
#define GYRO_CHIP_ID            0x0F
#define GYRO_RESET_CMD          0xB6
#define GYRO_ENABLE_DRDY_INT    0x80
#define GYRO_DIS_DRDY_INT       0x00
#define GYRO_INT_OPENDRAIN      0x02
#define GYRO_INT_PUSHPULL       0x00
#define GYRO_INT_LVL_HIGH       0x01
#define GYRO_INT_LVL_LOW        0x00

/* Registers */
#define GYRO_CHIP_ID_ADDR          0x00
#define GYRO_CHIP_ID_MASK          0xFF
#define GYRO_CHIP_ID_POS           0

#define GYRO_DRDY_ADDR             0x0A
#define GYRO_DRDY_MASK             0x80
#define GYRO_DRDY_POS              7

#define GYRO_RANGE_ADDR            0x0F
#define GYRO_RANGE_MASK            0xFF
#define GYRO_RANGE_POS             0

#define GYRO_ODR_ADDR              0x10
#define GYRO_ODR_MASK              0xFF
#define GYRO_ODR_POS               0

#define GYRO_SOFT_RESET_ADDR       0x14
#define GYRO_SOFT_RESET_MASK       0xFF
#define GYRO_SOFT_RESET_POS        0

#define GYRO_INT_CNTRL_ADDR        0x15
#define GYRO_INT_CNTRL_MASK        0xFF
#define GYRO_INT_CNTRL_POS         0

#define GYRO_INT3_IO_CTRL_ADDR     0x16
#define GYRO_INT3_IO_CTRL_MASK     0x03
#define GYRO_INT3_IO_CTRL_POS      0

#define GYRO_INT4_IO_CTRL_ADDR     0x16
#define GYRO_INT4_IO_CTRL_MASK     0x0C
#define GYRO_INT4_IO_CTRL_POS      2

#define GYRO_INT3_DRDY_ADDR        0x18
#define GYRO_INT3_DRDY_MASK        0x01
#define GYRO_INT3_DRDY_POS         0

#define GYRO_INT4_DRDY_ADDR        0x18
#define GYRO_INT4_DRDY_MASK        0x80
#define GYRO_INT4_DRDY_POS         7

#define GYRO_DATA_ADDR             0x02
#define GYRO_LPM1                  0X11

/* Convert deg/s to rad/s */
#define D2R M_PI / 180.0f

/****************************************************************************
 * BMI085 Constants
 ****************************************************************************/

#define BMI085_ACC_DISABLE                 0
#define BMI085_ACC_ENABLE                  1
#define BMI085_ACC_DATA_SYNC_LEN           1

#define BMI085_ACC_DATA_SYNC_MODE_MASK     0x0003
#define BMI085_ACC_DATA_SYNC_MODE_OFF      0x00
#define BMI085_ACC_DATA_SYNC_MODE_400HZ    0x01
#define BMI085_ACC_DATA_SYNC_MODE_1000HZ   0x02
#define BMI085_ACC_DATA_SYNC_MODE_2000HZ   0x03

#define BMI085_ACC_INTA_DISABLE            0x00
#define BMI085_ACC_INTA_ENABLE             0x01
#define BMI085_ACC_INTB_DISABLE            0x00
#define BMI085_ACC_INTB_ENABLE             0x02

/* Registers */
#define BMI085_ACC_INIT_CTRL_ADDR          0x59
#define BMI085_ACC_FEATURE_LSB_ADDR        0x5B
#define BMI085_ACC_FEATURE_MSB_ADDR        0x5C
#define BMI085_ACC_FEATURE_CFG_ADDR        0x5E
#define BMI085_ACC_INTERNAL_STATUS_ADDR    0x2A
#define BMI085_ACC_DATA_SYNC_ADDR          0x02
#define BMI085_ACC_INT1_MAP_ADDR           0x56
#define BMI085_ACC_INT2_MAP_ADDR           0x57


/****************************************************************************
 * Public Types
 ****************************************************************************/

struct bmi085_dev_s
{
#ifdef CONFIG_SENSORS_BMI085_I2C
FAR struct i2c_master_s *i2c; /* I2C interface */
uint8_t acc_addr;                 /* I2C address */
uint8_t gyro_addr;                 /* I2C address */
int freq;                     /* Frequency <= 3.4MHz */

#else /* CONFIG_SENSORS_BMI085_SPI */
FAR struct spi_dev_s *spi;    /* SPI interface */

#endif
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

uint8_t bmi085_getreg8(FAR struct bmi085_dev_s *priv, uint8_t i2c_addr, uint8_t regaddr);
void bmi085_putreg8(FAR struct bmi085_dev_s *priv, uint8_t i2c_addr, uint8_t regaddr,
                    uint8_t regval);
uint16_t bmi085_getreg16(FAR struct bmi085_dev_s *priv, uint8_t i2c_addr, uint8_t regaddr);
void bmi085_getregs(FAR struct bmi085_dev_s *priv, uint8_t i2c_addr, uint8_t regaddr,
                    uint8_t *regval, int len);

int bmi085_checkid(FAR struct bmi085_dev_s *priv);

/****************************************************************************
 * Deice Specific Function Prototypes
 ****************************************************************************/

void bmi085_set_normal_imu(FAR struct bmi085_dev_s *priv);
void bmi085_data_read(FAR struct bmi085_dev_s *priv, FAR struct accel_gyro_st_s *p);
void bmi085_acc_read(FAR struct bmi085_dev_s *priv, FAR struct accel_gyro_st_s *p);
void bmi085_gyro_read(FAR struct bmi085_dev_s *priv, FAR struct accel_gyro_st_s *p);
void bmi085_temp_read(FAR struct bmi085_dev_s *priv, FAR struct accel_gyro_st_s *p);

#endif /* __INCLUDE_NUTTX_SENSORS_BMI085_COMMOM_H */
