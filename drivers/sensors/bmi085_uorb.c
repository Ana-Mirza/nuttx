/****************************************************************************
 * drivers/sensors/bmi085_uorb.c
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

#include <nuttx/config.h>
#include <nuttx/nuttx.h>

#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/random.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/signal.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/param.h>
#include <nuttx/wqueue.h>

#if defined(CONFIG_SENSORS_BMI085_UORB)

/****************************************************************************
* Pre-processor Definitions
****************************************************************************/

#define BMI085_DEFAULT_INTERVAL 10000  /* Default conversion interval. */

/****************************************************************************
* Private Types
****************************************************************************/

/* Sensor ODR */

struct bmi085_odr_s
{
	uint8_t regval;    /* the data of register */
	uint32_t odr;      /* the unit is us */
};

/* Device struct */

struct bmi085_dev_uorb_s
{
	/* sensor_lowerhalf_s must be in the first line. */

	struct sensor_lowerhalf_s lower;      /* Lower half sensor driver. */
	FAR struct bmi085_config_s *config; /* Board configuration data */

	struct work_s work;                   /* Interrupt handler worker. */
	uint32_t interval;                    /* Sensor acquisition interval. */
	uint8_t addr;    			      				  /* I2C address */
	bool enabled;                        /* If this sensor is enabled */
	bool interrupts;                     /* Whether or not interrupts are */

	mutex_t lock;
	sem_t run;
  sem_t waitsem;                       /* Used to wait for the availability of data */

  struct work_s timeout;               /* Supports timeout work */

	struct bmi085_dev_s dev;
};

/****************************************************************************
* Private Function Prototypes
****************************************************************************/

/* Sensor handle functions */

static void bmi085_accel_enable(FAR struct bmi085_dev_uorb_s *priv,
																bool enable);
static void bmi085_gyro_enable(FAR struct bmi085_dev_uorb_s *priv,
															bool enable);

/* Sensor ops functions */

static int bmi085_set_accel_interval(FAR struct sensor_lowerhalf_s *lower,
																		FAR struct file *filep,
																		FAR uint32_t *period_us);
static int bmi085_set_gyro_interval(FAR struct sensor_lowerhalf_s *lower,
																		FAR struct file *filep,
																		FAR uint32_t *period_us);
static int bmi085_accel_activate(FAR struct sensor_lowerhalf_s *lower,
																FAR struct file *filep,
																bool enable);
static int bmi085_gyro_activate(FAR struct sensor_lowerhalf_s *lower,
																FAR struct file *filep,
																bool enable);

/* Sensor poll/interrupt functions */

static void accel_worker(FAR void *arg);
static void gyro_worker(FAR void *arg);
static int bmi085_findodr(uint32_t time,
													FAR const struct bmi085_odr_s *odr_s,
													int len);

/****************************************************************************
* Private Data
****************************************************************************/

static const struct sensor_ops_s g_bmi085_accel_ops =
{
	.fetch = NULL,
	.activate     = bmi085_accel_activate,      /* Enable/disable sensor. */
	.set_interval = bmi085_set_accel_interval,  /* Set output data period. */
};

static const struct sensor_ops_s g_bmi085_gyro_ops =
{
	.fetch = NULL,
	.activate     = bmi085_gyro_activate,      /* Enable/disable sensor. */
	.set_interval = bmi085_set_gyro_interval,  /* Set output data period. */
};

static const struct bmi085_odr_s g_bmi085_gyro_odr[] =
{
  { GYRO_ODR_100HZ_BW_12HZ,    10000 },  /* 100 Hz */
  { GYRO_ODR_100HZ_BW_32HZ,    10000 },  /* 100 Hz */
  { GYRO_ODR_200HZ_BW_23HZ,     5000 },  /* 200 Hz */
  { GYRO_ODR_200HZ_BW_64HZ,     5000 },  /* 200 Hz */
  { GYRO_ODR_400HZ_BW_47HZ,     2500 },  /* 400 Hz */
  { GYRO_ODR_1000HZ_BW_116HZ,   1000 },  /* 1000 Hz */
  { GYRO_ODR_2000HZ_BW_230HZ,    500 },  /* 2000 Hz */
  { GYRO_ODR_2000HZ_BW_532HZ,    500 },  /* 2000 Hz (same ODR as above, different BW) */
};

static const struct bmi085_odr_s g_bmi085_accel_odr[] =
{
	{ ACCEL_ODR_12_5HZ,   80000 }, /* 12.5 Hz = 80.0 ms */
	{ ACCEL_ODR_25_HZ,     40000 }, /* 25 Hz = 40.0 ms */
	{ ACCEL_ODR_50_HZ,     20000 }, /* 50 Hz = 20.0 ms */
	{ ACCEL_ODR_100_HZ,    10000 }, /* 100 Hz = 10.0 ms */
	{ ACCEL_ODR_200_HZ,     5000 }, /* 200 Hz = 5.0 ms */
	{ ACCEL_ODR_400_HZ,     2500 }, /* 400 Hz = 2.5 ms */
	{ ACCEL_ODR_800_HZ,     1250 }, /* 800 Hz = 1.25 ms */
	{ ACCEL_ODR_1600_HZ,     625 }, /* 1600 Hz = 0.625 ms */
};

/****************************************************************************
* Name: bmi085_findodr
*
* Description:
*   Find the period that matches best.
*
* Input Parameters:
*   time  - Desired interval.
*   odr_s - Array of sensor output data rate.
*   len   - Array length.
*
* Returned Value:
*   Index of the best fit ODR.
*
* Assumptions/Limitations:
*   none.
*
****************************************************************************/

static int bmi085_findodr(uint32_t time,
													FAR const struct bmi085_odr_s *odr_s,
													int len)
{
	int i;

	for (i = 0; i < len; i++)
		{
			if (time == odr_s[i].odr)
				{
					return i;
				}
		}

	return i - 1;
}

/****************************************************************************
* Name: bmi085_accel_enable
*
* Description:
*   Enable or disable sensor device. when enable sensor, sensor will
*   work in  current mode(if not set, use default mode). when disable
*   sensor, it will disable sense path and stop convert.
*
* Input Parameters:
*   priv   - The instance of lower half sensor driver
*   enable - true(enable) and false(disable)
*
* Returned Value:
*   Return 0 if the driver was success; A negated errno
*   value is returned on any failure.
*
* Assumptions/Limitations:
*   none.
*
****************************************************************************/

static void bmi085_accel_enable(FAR struct bmi085_dev_uorb_s *priv,
																bool enable)
{
	int idx;

	if (enable)
		{
			/* Set accel as normal mode. */

			bmi085_putreg8(&priv->dev, priv->addr, ACCEL_PWR_CNTRL_ADDR, ACCEL_ENABLE_CMD);
			nxsig_usleep(30000);

			idx = bmi085_findodr(priv->interval, g_bmi085_accel_odr,
													nitems(g_bmi085_accel_odr));
			bmi085_putreg8(&priv->dev, priv->addr, ACCEL_ODR_ADDR,
										ACCEL_NORMAL_AVG4 | g_bmi085_accel_odr[idx].regval);

			priv->enabled = true;
			sninfo("BMI085 acc enabled.\n");
		}
	else
		{
			/* Set suspend mode to sensors. */

			work_cancel(HPWORK, &priv->work);
			bmi085_putreg8(&priv->dev, priv->addr, ACCEL_PWR_CNTRL_ADDR, ACCEL_DISABLE_CMD);
		}
}

/****************************************************************************
* Name: bmi085_gyro_enable
*
* Description:
*   Enable or disable sensor device. when enable sensor, sensor will
*   work in  current mode(if not set, use default mode). when disable
*   sensor, it will disable sense path and stop convert.
*
* Input Parameters:
*   priv   - The instance of lower half sensor driver
*   enable - true(enable) and false(disable)
*
* Returned Value:
*   Return 0 if the driver was success; A negated errno
*   value is returned on any failure.
*
* Assumptions/Limitations:
*   none.
*
****************************************************************************/

static void bmi085_gyro_enable(FAR struct bmi085_dev_uorb_s *priv,
															bool enable)
{
	int idx;

	if (enable)
		{
			/* Set gyro as normal mode. */

			bmi085_putreg8(&priv->dev, priv->addr, GYRO_LPM1, GYRO_PWR_NORMAL);
			nxsig_usleep(30000);

			idx = bmi085_findodr(priv->interval, g_bmi085_gyro_odr,
													nitems(g_bmi085_gyro_odr));
			bmi085_putreg8(&priv->dev, priv->addr, GYRO_ODR_ADDR,
										g_bmi085_gyro_odr[idx].regval);
			priv->enabled = true;
			sninfo("BMI085 gyro enabled.\n");
		}
	else
		{
			work_cancel(HPWORK, &priv->work);

			/* Set suspend mode to sensors. */

			bmi085_putreg8(&priv->dev, priv->addr, GYRO_LPM1, GYRO_PWR_SUSPEND);
		}
}

/****************************************************************************
* Name: bmi085_set_accel_interval
*
* Description:
*   Set the sensor output data period in microseconds for a given sensor.
*   If *period_us > max_delay it will be truncated to max_delay and if
*   *period_us < min_delay it will be replaced by min_delay.
*
* Input Parameters:
*   lower     - The instance of lower half sensor driver.
*   filep     - The pointer of file, represents each user using the sensor.
*   period_us - The time between report data, in us. It may by overwrite
*                by lower half driver.
*
* Returned Value:
*   Return 0 if the driver was success; A negated errno
*   value is returned on any failure.
*
* Assumptions/Limitations:
*   none.
*
****************************************************************************/

static int bmi085_set_accel_interval(FAR struct sensor_lowerhalf_s *lower,
																		FAR struct file *filep,
																		FAR uint32_t *period_us)
{
	FAR struct bmi085_dev_uorb_s *priv = (FAR struct bmi085_dev_uorb_s *)lower;
	int num;

	/* Sanity check. */

	if (NULL == priv || NULL == period_us)
		{
			return -EINVAL;
		}

	num = bmi085_findodr(*period_us, g_bmi085_accel_odr,
											nitems(g_bmi085_accel_odr));
	bmi085_putreg8(&priv->dev, priv->addr, ACCEL_ODR_ADDR,
								ACCEL_NORMAL_AVG4 | g_bmi085_accel_odr[num].regval);

	priv->interval = g_bmi085_accel_odr[num].odr;
	*period_us = priv->interval;
	return OK;
}

/****************************************************************************
* Name: bmi085_set_gyro_interval
*
* Description:
*   Set the sensor output data period in microseconds for a given sensor.
*   If *period_us > max_delay it will be truncated to max_delay and if
*   *period_us < min_delay it will be replaced by min_delay.
*
* Input Parameters:
*   lower     - The instance of lower half sensor driver.
*   filep     - The pointer of file, represents each user using the sensor.
*   period_us - The time between report data, in us. It may by overwrite
*                by lower half driver.
*
* Returned Value:
*   Return 0 if the driver was success; A negated errno
*   value is returned on any failure.
*
* Assumptions/Limitations:
*   none.
*
****************************************************************************/

static int bmi085_set_gyro_interval(FAR struct sensor_lowerhalf_s *lower,
																		FAR struct file *filep,
																		FAR uint32_t *period_us)
{
	FAR struct bmi085_dev_uorb_s *priv = (FAR struct bmi085_dev_uorb_s *)lower;
	int num;

	/* Sanity check. */

	if (NULL == priv || NULL == period_us)
		{
			return -EINVAL;
		}

	num = bmi085_findodr(*period_us, g_bmi085_gyro_odr,
											nitems(g_bmi085_gyro_odr));
	bmi085_putreg8(&priv->dev, priv->addr, GYRO_ODR_ADDR,
								g_bmi085_gyro_odr[num].regval);

	priv->interval = g_bmi085_gyro_odr[num].odr;
	*period_us = priv->interval;
	return OK;
}

/****************************************************************************
* Name: bmi085_gyro_activate
*
* Description:
*   Enable or disable sensor device. when enable sensor, sensor will
*   work in  current mode(if not set, use default mode). when disable
*   sensor, it will disable sense path and stop convert.
*
* Input Parameters:
*   lower  - The instance of lower half sensor driver.
*   filep  - The pointer of file, represents each user using the sensor.
*   enable - true(enable) and false(disable).
*
* Returned Value:
*   Return 0 if the driver was success; A negated errno
*   value is returned on any failure.
*
* Assumptions/Limitations:
*   none.
*
****************************************************************************/

static int bmi085_gyro_activate(FAR struct sensor_lowerhalf_s *lower,
																FAR struct file *filep,
																bool enable)
{
	FAR struct bmi085_dev_uorb_s *priv = (FAR struct bmi085_dev_uorb_s *)lower;

	bmi085_gyro_enable(priv, enable);

	return OK;
}

/****************************************************************************
* Name: bmi085_accel_activate
*
* Description:
*   Enable or disable sensor device. when enable sensor, sensor will
*   work in  current mode(if not set, use default mode). when disable
*   sensor, it will disable sense path and stop convert.
*
* Input Parameters:
*   lower  - The instance of lower half sensor driver.
*   filep  - The pointer of file, represents each user using the sensor.
*   enable - true(enable) and false(disable).
*
* Returned Value:
*   Return 0 if the driver was success; A negated errno
*   value is returned on any failure.
*
* Assumptions/Limitations:
*   none.
*
****************************************************************************/

static int bmi085_accel_activate(FAR struct sensor_lowerhalf_s *lower,
																FAR struct file *filep,
																bool enable)
{
	FAR struct bmi085_dev_uorb_s *priv = (FAR struct bmi085_dev_uorb_s *)lower;

	bmi085_accel_enable(priv, enable);

	return OK;
}

/****************************************************************************
 * Name: gyro_int_enable
 *
 * Description:
 *      Enables/disables the gyroscope interrupt.
 *
 ****************************************************************************/

static int gyro_int_enable(FAR struct bmi085_dev_uorb_s *dev, bool enable)
{
	int err;
	uint8_t enable_bits = enable ? 0x80 : 0x00;
	uint8_t disable_bits = enable ? 0x00 : 0x80;

	// err = bmi085_set_bits(dev, dev->gyro.addr, BMI085_GYR_INT_CTRL, enable_bits,
	// 												disable_bits);


	dev->interrupts = enable; /* Update state */
	return err;
}
 
 /****************************************************************************
	* Name: accel_int_enable
	*
	* Description:
	*      Enables/disables the accelerometer interrupt.
	*
	****************************************************************************/
 
static int accel_int_enable(FAR struct bmi085_dev_uorb_s *priv, bool enable)
{
	int err = 0;

	/* Set pin modes INT1*/
	if (enable) {

		/* Configure acc output pin INT1*/
		bmi085_pin_mode_int1(&priv->dev);

		/* Map pins for INT1 */
		bmi085_accel_map_int1(&priv->dev);

		sninfo("accel_int_enable: enabled interrupts.\n");
	}

	priv->interrupts = enable; /* Update state */

	return err;
}
 
/****************************************************************************
	* Name: bmi085_convert_temp
	*
	* Description:
	*   Converts raw temperature reading into units of degrees Celsius.
	*
	****************************************************************************/
 
static float bmi085_convert_temp(uint16_t temp)
{
	int16_t temp_int11;

	if (temp > 1023) {
		temp_int11 = temp - 2048;
	} else {
		temp_int11 = temp;
	}
	return (float) (temp_int11 * 0.125f + 23.0f);
}
 
/****************************************************************************
	* Name: bmi085_read_gyro
	*
	* Description:
	*   Reads gyroscope data into UORB structure.
	*
	****************************************************************************/
 
static int bmi085_read_gyro(FAR struct bmi085_dev_uorb_s *priv,
	 FAR struct sensor_gyro *data)
{

	int err = 0;
	int16_t gyro_data[3];
	u_int8_t raw_data[6];

	bmi085_getregs(&priv->dev, priv->addr, GYRO_DATA_ADDR, raw_data, 6);

	/* Combine MSB and LSB data */

	gyro_data[0] = (int16_t)(raw_data[1] << 8) | raw_data[0];
	gyro_data[1] = (int16_t)(raw_data[3] << 8) | raw_data[2];
	gyro_data[2] = (int16_t)(raw_data[5] << 8) | raw_data[4];

	/* Convert data into the format required */
	float gyro_scale = 2000.0f / 32768.0f * D2R;

	data->x =
	(float)(gyro_data[1]) * gyro_scale;
	data->y =
	(float)(gyro_data[2]) * gyro_scale;
	data->z =
	(float)(gyro_data[3]) * gyro_scale;

	return err;
}
 
/****************************************************************************
 * Name: bmi085_read_accel
 *
 * Description:
 *   Reads accelerometer data into UORB structure.
 *
 ****************************************************************************/
 
static int bmi085_read_accel(FAR struct bmi085_dev_uorb_s *priv,
	FAR struct sensor_accel *data)
{
	int16_t raw_data[3]; /* 3 accel (xyz) */
	u_int8_t acc_data[9];
	int16_t raw_temp;    /* Temperature */
	u_int8_t temp_data[2];
	int err = 0;

	/* Get accelerometer data */

	bmi085_getregs(&priv->dev, priv->addr, ACCEL_ACCEL_DATA_ADDR, acc_data, 9);

	raw_data[0] = (int16_t)(acc_data[1] << 8) | acc_data[0];
	raw_data[1] = (int16_t)(acc_data[3] << 8) | acc_data[2];
	raw_data[2] = (int16_t)(acc_data[5] << 8) | acc_data[4];

	/* Get temperature data */

	bmi085_getregs(&priv->dev, priv->addr, ACCEL_TEMP_DATA_ADDR, temp_data, 2);
	raw_temp = (temp_data[0] * 8) + (temp_data[1] / 32);

	/* Convert data into the required format */
	uint8_t range = bmi085_getreg8(&priv->dev, priv->addr, ACCEL_RANGE_ADDR);
  uint16_t accel_range_mg = 1 << (range + 1);
	float scale = (float)accel_range_mg * 1000 / 32768.0f;

	data->timestamp = (acc_data[8] << 16) | (acc_data[7] << 8) | acc_data[6];
	data->temperature = bmi085_convert_temp(raw_temp);
	data->x = (float)(raw_data[0]) * scale;
	data->y = (float)(raw_data[1]) * scale;
	data->z = (float)(raw_data[2]) * scale;

	return err;
}
 
/****************************************************************************
	* Name: push_gyro
	*
	* Description:
	*   Push gyro data to the UORB upper half.
	*
	****************************************************************************/

static int push_gyro(FAR struct bmi085_dev_uorb_s *dev)
{
	int err;
	struct sensor_gyro data;

	err = nxmutex_lock(&dev->lock);
	if (err < 0)
		{
			return err;
		}

	err = bmi085_read_gyro(dev, &data);
	if (err < 0)
		{
			goto early_ret;
		}

	dev->lower.push_event(dev->lower.priv, &data, sizeof(data));

early_ret:
	nxmutex_unlock(&dev->lock);
	return err;
}

/****************************************************************************
	* Name: push_accel
	*
	* Description:
	*   Push accelerometer data to the UORB upper half.
	*
	****************************************************************************/
 
static int push_accel(FAR struct bmi085_dev_uorb_s *dev)
{
	int err;
	struct sensor_accel data;

	err = nxmutex_lock(&dev->lock);
	if (err < 0)
		{
			return err;
		}

	err = bmi085_read_accel(dev, &data);
	if (err < 0)
		{
			goto early_ret;
		}

	dev->lower.push_event(dev->lower.priv, &data, sizeof(data));

early_ret:
	nxmutex_unlock(&dev->lock);
	return err;
}
 
/****************************************************************************
	* Name: gyro_worker
	*
	* Description:
	*   Worker thread called by gyroscope interrupt handler.
	*
	****************************************************************************/
 
static void gyro_worker(FAR void *arg)
{
	push_gyro(arg);
}

/****************************************************************************
 * Name: accel_worker
 *
 * Description:
 *   Worker thread called by accelerometer interrupt handler.
 *
 ****************************************************************************/

static void accel_worker(FAR void *arg)
{
	push_accel(arg);
}

/****************************************************************************
 * Name: gyro_int_handler
 *
 * Description:
 *   Interrupt handler for gyroscope interrupts.
 *
 ****************************************************************************/

static int gyro_int_handler(int irq, FAR void *context, FAR void *arg)
{
	FAR struct bmi085_dev_uorb_s *priv = (FAR struct bmi085_dev_uorb_s *)(arg);
	int err;
	(void)(context);

	DEBUGASSERT(arg != NULL);

	/* Start high priority worker thread */

	err = work_queue(HPWORK, &priv->work, &gyro_worker, priv, 0);

	if (err < 0)
		{
			snerr("Could not queue BMI085 gyro work queue: %d\n", err);
		}

	return err;
}
 
/****************************************************************************
	* Name: accel_int_handler
	*
	* Description:
	*   Interrupt handler for accelerometer interrupts.
	*
	****************************************************************************/
 
static int accel_int_handler(int irq, FAR void *context, FAR void *arg)
{
	FAR struct bmi085_dev_uorb_s *priv = (FAR struct bmi085_dev_uorb_s *)(arg);
	int err;
	(void)(context);

	DEBUGASSERT(arg != NULL);

	/* Start high priority worker thread */

	err = work_queue(HPWORK, &priv->work, &accel_worker, priv, 0);

	if (err < 0)
		{
			snerr("Could not queue BMI085 accel work queue: %d\n", err);
		}

	return err;
}
 
/****************************************************************************
	* Name: gyro_thread
	*
	* Description:
	*   Polling thread for gyroscope measurements
	*
	****************************************************************************/

static int gyro_thread(int argc, char **argv)
{
	FAR struct bmi085_dev_uorb_s *dev =
			(FAR struct bmi085_dev_uorb_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
	int err = 0;

	while (true)
		{
			/* If the sensor is disabled we wait indefinitely */

			if (!dev->enabled)
				{
					err = nxsem_wait(&dev->run);
					if (err < 0)
						{
							continue;
						}
				}

			/* If the sensor is enabled, grab some data */

			err = push_gyro(dev);
			if (err < 0)
				{
					continue;
				}

			/* Wait for next measurement cycle */

			nxsig_usleep(dev->interval);
		}

	return err;
}
 
/****************************************************************************
	* Name: accel_thread
	*
	* Description:
	*   Polling thread for accelerometer measurements.
	*
	****************************************************************************/
 
static int accel_thread(int argc, char **argv)
{
	FAR struct bmi085_dev_uorb_s *dev =
			(FAR struct bmi085_dev_uorb_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
	int err = 0;

	while (true)
		{
			/* If the sensor is disabled we wait indefinitely */

			if (!dev->enabled)
				{
					err = nxsem_wait(&dev->run);
					if (err < 0)
						{
							continue;
						}
				}

			/* If the sensor is enabled, grab some data */

			err = push_accel(dev);
			if (err < 0)
				{
					continue;
				}

			/* Wait for next measurement cycle */

			nxsig_usleep(dev->interval);
		}

	return err;
}

/****************************************************************************
* Public Functions
****************************************************************************/

/****************************************************************************
* Name: bmi085_register_accel
*
* Description:
*   Register the BMI085 accel sensor.
*
* Input Parameters:
*   devno   - Sensor device number.
*   config  - Interrupt fuctions.
*
* Returned Value:
*   Description of the value returned by this function (if any),
*   including an enumeration of all possible error values.
*
* Assumptions/Limitations:
*   none.
*
****************************************************************************/

#ifdef CONFIG_SENSORS_BMI085_I2C
static int bmi085_register_accel(int devno,
																FAR struct i2c_master_s *dev,
																FAR struct bmi085_config_s *config)
#else /* CONFIG_BMI085_SPI */
static int bmi085_register_accel(int devno,
																FAR struct spi_dev_s *dev,
																FAR struct bmi085_config_s *config)
#endif
{
	FAR struct bmi085_dev_uorb_s *priv;
	FAR char *argv[2];
  char arg1[32];
	int ret = OK;

	/* Sanity check */

	DEBUGASSERT(dev != NULL);

	#if !defined(CONFIG_SCHED_HPWORK)
		if (config->xl_attach != NULL)
			{
				snerr("CONFIG_SCHED_HPWORK required for interrupt driven measuring.");
				return -ENOSYS;
			}
	#endif

	/* Initialize the device structure */

	priv = kmm_zalloc(sizeof(*priv));
	if (priv == NULL)
		{
			return -ENOMEM;
		}

	/* config accelerometer */

#ifdef CONFIG_SENSORS_BMI085_I2C
	priv->dev.i2c  = dev;
	priv->addr = BMI085_ACC_I2C_ADDR;
	config->acc_addr = BMI085_ACC_I2C_ADDR;
	config->gyro_addr = BMI085_GYRO_I2C_ADDR;
	config->freq = BMI085_I2C_FREQ;
	priv->dev.config = config;
	priv->config = config;

#else /* CONFIG_SENSORS_BMI085_SPI */
	priv->dev.spi = dev;
#endif

	priv->lower.ops = &g_bmi085_accel_ops;
	priv->lower.type = SENSOR_TYPE_ACCELEROMETER;
	priv->lower.uncalibrated = true;
	priv->interval = BMI085_DEFAULT_INTERVAL;
	priv->lower.nbuffer = 1;

	/* Create mutex */

	ret = nxmutex_init(&priv->lock);
	if (ret < 0)
    {
      snerr("Failed to initialize mutex: %d\n", ret);
      goto free_mem;
    }

		/* Create accel semaphore */

		ret = nxsem_init(&priv->run, 0, 0);
		if (ret < 0)
			{
				snerr("Failed to initialize accel semaphore: %d\n", ret);
				goto del_sem;
			}

	/* Read and verify the deviceid */

	ret = bmi085_checkid(&priv->dev);
	if (ret < 0)
		{
			snerr("Wrong Device ID!\n");
			goto free_mem;
		}

	/* Register the character driver */

	ret = sensor_register(&priv->lower, devno);
	if (ret < 0)
		{
			snerr("Failed to register accel driver: %d\n", ret);
			goto unreg_accel;
		}

	/* Accelerometer measuring setup */

  if (config->xl_attach != NULL)
    {
      /* Register accel interrupt handler */

      ret = config->xl_attach(accel_int_handler, priv);
      if (ret < 0)
        {
          snerr("Failed to register accel interrupt handler: %d\n", ret);
          goto unreg_handler;
        }

      ret = accel_int_enable(priv, true);
      if (ret < 0)
        {
          snerr("Failed to register accel interrupt handler: %d\n", ret);
          goto unreg_handler;
        }

      sninfo("BMI085 accel interrupt handler attached.");
    }
  else
    {
      /* Register accel polling thread */

      snprintf(arg1, 16, "%p", priv);
      argv[0] = arg1;
      argv[1] = NULL;
      ret = kthread_create("bmi085_xl_thread", SCHED_PRIORITY_DEFAULT,
                           CONFIG_EXAMPLES_BMI085_STACKSIZE,
                           accel_thread, argv);
      if (ret < 0)
        {
          snerr("Failed to register accel polling thread: %d\n", ret);
          goto unreg_handler;
        }

      sninfo("BMI085 accel using polling thread.");
    }

	if (ret < 0)
	{
	unreg_handler:
		if (config->xl_attach != NULL)
			{
				kthread_delete(ret);
			}
	unreg_accel:
		sensor_unregister(&priv->lower, devno);
	del_sem:
		nxsem_destroy(&priv->run);
	del_mutex:
		nxmutex_destroy(&priv->lock);
	free_mem:
		kmm_free(priv);
		snerr("ERROR: Failed to register BMI085 driver: %d\n", ret);
	} else {
		sninfo("BMI085 driver registered!");
	}

	return ret;
}

/****************************************************************************
* Name: bmi085_register_gyro
*
* Description:
*   Register the BMI085 gyro sensor.
*
* Input Parameters:
*   devno   - Sensor device number.
*   config  - Interrupt fuctions.
*
* Returned Value:
*   Description of the value returned by this function (if any),
*   including an enumeration of all possible error values.
*
* Assumptions/Limitations:
*   none.
*
****************************************************************************/

#ifdef CONFIG_SENSORS_BMI085_I2C
static int bmi085_register_gyro(int devno,
																FAR struct i2c_master_s *dev,
																FAR struct bmi085_config_s *config)
#else /* CONFIG_BMI085_SPI */
static int bmi085_register_gyro(int devno,
																FAR struct spi_dev_s *dev,
																FAR struct bmi085_config_s *config)
#endif
{
	FAR struct bmi085_dev_uorb_s *priv;
	FAR char *argv[2];
	char arg1[32];
	int ret ;

	/* Sanity check */

	DEBUGASSERT(dev != NULL);

	#if !defined(CONFIG_SCHED_HPWORK)
		if (config->gy_attach != NULL)
			{
				snerr("CONFIG_SCHED_HPWORK required for interrupt driven measuring.");
				return -ENOSYS;
			}
	#endif

	/* Initialize the device structure */

	priv = kmm_zalloc(sizeof(*priv));
	if (priv == NULL)
		{
			return -ENOMEM;
		}

	/* config gyroscope */

#ifdef CONFIG_SENSORS_BMI085_I2C
	priv->dev.i2c  = dev;
	priv->addr = BMI085_GYRO_I2C_ADDR;
	config->acc_addr = BMI085_ACC_I2C_ADDR;
	config->gyro_addr = BMI085_GYRO_I2C_ADDR;
	config->freq = BMI085_I2C_FREQ;
	priv->dev.config = config;
	priv->config = config;

#else /* CONFIG_SENSORS_BMI085_SPI */
	priv->dev.spi = dev;
#endif

	priv->lower.ops = &g_bmi085_gyro_ops;
	priv->lower.type = SENSOR_TYPE_GYROSCOPE;
	priv->lower.uncalibrated = true;
	priv->interval = BMI085_DEFAULT_INTERVAL;
	priv->lower.nbuffer = 1;

	/* Create mutex */

	ret = nxmutex_init(&priv->lock);
	if (ret < 0)
    {
      snerr("Failed to initialize mutex: %d\n", ret);
      goto free_mem;
    }

	 /* Create gyro semaphore */

	 ret = nxsem_init(&priv->run, 0, 0);
	 if (ret < 0)
		 {
			 snerr("Failed to initialize accel semaphore: %d\n", ret);
			 goto del_sem;
		 }

	/* Read and verify the device id */

	ret = bmi085_checkid(&priv->dev);
	if (ret < 0)
		{
			snerr("Wrong Device ID!\n");
			goto free_mem;
		}

	/* Register the character driver */

	ret = sensor_register(&priv->lower, devno);
	if (ret < 0)
		{
			snerr("Failed to register accel driver: %d\n", ret);
			goto unreg_accel;
		}

	/* Gyroscope measuring setup */

  if (config->gy_attach != NULL)
    {
      /* Register gyro interrupt handler */

      ret = config->xl_attach(gyro_int_handler, priv);
      if (ret < 0)
        {
          snerr("Failed to register accel interrupt handler: %d\n", ret);
          goto unreg_handler;
        }

      ret = gyro_int_enable(priv, true);
      if (ret < 0)
        {
          snerr("Failed to register accel interrupt handler: %d\n", ret);
          goto unreg_handler;
        }

      sninfo("BMI085 accel interrupt handler attached.");
    }
  else
    {
      /* Register gyro polling thread */

      snprintf(arg1, 16, "%p", priv);
      argv[0] = arg1;
      argv[1] = NULL;
      ret = kthread_create("bmi085_xl_thread", SCHED_PRIORITY_DEFAULT,
                           CONFIG_EXAMPLES_BMI085_STACKSIZE,
                           accel_thread, argv);
      if (ret < 0)
        {
          snerr("Failed to register accel polling thread: %d\n", ret);
          goto unreg_handler;
        }

      sninfo("BMI085 accel using polling thread.");
    }

	if (ret < 0)
	{
	unreg_handler:
		if (config->gy_attach != NULL)
			{
				kthread_delete(ret);
			}
	unreg_accel:
		sensor_unregister(&priv->lower, devno);
	del_sem:
		nxsem_destroy(&priv->run);
	del_mutex:
		nxmutex_destroy(&priv->lock);
	free_mem:
		kmm_free(priv);
		snerr("ERROR: Failed to register BMI085 driver: %d\n", ret);
	} else {
		sninfo("BMI085 driver registered!");
	}

	return ret;
}

/****************************************************************************
* Public Functions
****************************************************************************/

/****************************************************************************
* Name: bmi085_register
*
* Description:
*   Register the BMI085 accel and gyro sensor.
*
* Input Parameters:
*   devno   - Sensor device number.
*   dev     - An instance of the SPI or I2C interface to use to communicate
*             with BMI085
*
* Returned Value:
*   Description of the value returned by this function (if any),
*   including an enumeration of all possible error values.
*
* Assumptions/Limitations:
*   none.
*
****************************************************************************/

#ifdef CONFIG_SENSORS_BMI085_I2C
int bmi085_register_uorb(int devno, FAR struct i2c_master_s *dev, 
	FAR struct bmi085_config_s *config)
#else /* CONFIG_BMI085_SPI */
int bmi085_register_uorb(int devno, FAR struct spi_dev_s *dev,
	FAR struct bmi085_config_s *config)
#endif
{
	int ret = OK;

	ret = bmi085_register_accel(devno, dev, config);
	DEBUGASSERT(ret >= 0);

	ret = bmi085_register_gyro(devno, dev, config);
	DEBUGASSERT(ret >= 0);

	sninfo("BMI085 driver loaded successfully!\n");
	return ret;
}

#endif /* CONFIG_SENSORS_BMI085_UORB */
