/*
 * STMicroelectronics lsm6ds3 driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#include	<linux/input/lsm6ds3.h>
#include	"lsm6ds3_core.h"

/* COMMON VALUES FOR ACCEL-GYRO SENSORS */
#define LSM6DS3_WAI_ADDRESS			0x0f
#define LSM6DS3_WAI_EXP			0x69
#define LSM6DS3_AXIS_EN_MASK			0x38
#define LSM6DS3_INT1_ADDR			0x0d
#define LSM6DS3_INT2_ADDR			0x0e
#define LSM6DS3_INT1_FULL			0x20
#define LSM6DS3_INT1_FTH			0x08
#define LSM6DS3_MD1_ADDR			0x5e
#define LSM6DS3_ODR_LIST_NUM			5
#define LSM6DS3_ODR_POWER_OFF_VAL		0x00
#define LSM6DS3_ODR_26HZ_VAL			0x02
#define LSM6DS3_ODR_52HZ_VAL			0x03
#define LSM6DS3_ODR_104HZ_VAL		0x04
#define LSM6DS3_ODR_208HZ_VAL		0x05
#define LSM6DS3_ODR_416HZ_VAL		0x06
#define LSM6DS3_FS_LIST_NUM			4
#define LSM6DS3_BDU_ADDR			0x12
#define LSM6DS3_BDU_MASK			0x40
#define LSM6DS3_EN_BIT			0x01
#define LSM6DS3_DIS_BIT			0x00
#define LSM6DS3_FUNC_EN_ADDR			0x19
#define LSM6DS3_FUNC_EN_MASK			0x04
#define LSM6DS3_FUNC_CFG_ACCESS_ADDR		0x01
#define LSM6DS3_FUNC_CFG_ACCESS_MASK		0x01
#define LSM6DS3_FUNC_CFG_ACCESS_MASK2	0x04
#define LSM6DS3_FUNC_CFG_REG2_MASK		0x80
#define LSM6DS3_FUNC_CFG_START1_ADDR		0x62
#define LSM6DS3_FUNC_CFG_START2_ADDR		0x63
#define LSM6DS3_SELFTEST_ADDR		0x14
#define LSM6DS3_SELFTEST_ACCEL_MASK		0x03
#define LSM6DS3_SELFTEST_GYRO_MASK		0x0c
#define LSM6DS3_GYRO_READY_TIME_FROM_PD_MS	150
#define LSM6DS3_ACCEL_READY_TIME_FROM_PD_MS	10
#define LSM6DS3_SELF_TEST_DISABLED_VAL	0x00
#define LSM6DS3_SELF_TEST_POS_SIGN_VAL	0x01
#define LSM6DS3_SELF_TEST_NEG_ACCEL_SIGN_VAL	0x02
#define LSM6DS3_SELF_TEST_NEG_GYRO_SIGN_VAL	0x03
#define LSM6DS3_LIR_ADDR			0x58
#define LSM6DS3_LIR_MASK			0x01
#define LSM6DS3_TIMER_EN_ADDR		0x58
#define LSM6DS3_TIMER_EN_MASK		0x80
#define LSM6DS3_PEDOMETER_EN_ADDR		0x58
#define LSM6DS3_PEDOMETER_EN_MASK		0x40
#define LSM6DS3_INT2_ON_INT1_ADDR		0x13
#define LSM6DS3_INT2_ON_INT1_MASK		0x20
#define LSM6DS3_MIN_DURATION_MS		1638
#define LSM6DS3_ROUNDING_ADDR		0x16
#define LSM6DS3_ROUNDING_MASK		0x04
#define LSM6DS3_FIFO_MODE_ADDR		0x0a
#define LSM6DS3_FIFO_MODE_MASK		0x07
#define LSM6DS3_FIFO_MODE_BYPASS		0x00
#define LSM6DS3_FIFO_MODE_CONTINUOS		0x06
#define LSM6DS3_FIFO_THRESHOLD_IRQ_MASK	0x08
#define LSM6DS3_FIFO_ODR_ADDR		0x0a
#define LSM6DS3_FIFO_ODR_MASK		0x78
#define LSM6DS3_FIFO_ODR_MAX			0x08
#define LSM6DS3_FIFO_ODR_OFF			0x00
#define LSM6DS3_FIFO_DECIMATOR_ADDR		0x08
#define LSM6DS3_FIFO_ACCEL_DECIMATOR_MASK	0x07
#define LSM6DS3_FIFO_GYRO_DECIMATOR_MASK	0x38
#define LSM6DS3_FIFO_THR_L_ADDR		0x06
#define LSM6DS3_FIFO_THR_H_ADDR		0x07
#define LSM6DS3_FIFO_THR_H_MASK		0x0f
#define LSM6DS3_FIFO_THR_IRQ_MASK		0x08

/* CUSTOM VALUES FOR ACCEL SENSOR */
#define LSM6DS3_ACCEL_ODR_ADDR		0x10
#define LSM6DS3_ACCEL_ODR_MASK		0xf0
#define LSM6DS3_ACCEL_FS_ADDR		0x10
#define LSM6DS3_ACCEL_FS_MASK		0x0c
#define LSM6DS3_ACCEL_FS_2G_VAL		0x00
#define LSM6DS3_ACCEL_FS_4G_VAL		0x02
#define LSM6DS3_ACCEL_FS_8G_VAL		0x03
#define LSM6DS3_ACCEL_FS_16G_VAL		0x01
#define LSM6DS3_ACCEL_FS_2G_GAIN		61
#define LSM6DS3_ACCEL_FS_4G_GAIN		122
#define LSM6DS3_ACCEL_FS_8G_GAIN		244
#define LSM6DS3_ACCEL_FS_16G_GAIN		488
#define LSM6DS3_ACCEL_OUT_X_L_ADDR		0x28
#define LSM6DS3_ACCEL_OUT_Y_L_ADDR		0x2a
#define LSM6DS3_ACCEL_OUT_Z_L_ADDR		0x2c
#define LSM6DS3_ACCEL_AXIS_EN_ADDR		0x18
#define LSM6DS3_ACCEL_DRDY_IRQ_MASK		0x01

/* CUSTOM VALUES FOR GYRO SENSOR */
#define LSM6DS3_GYRO_ODR_ADDR		0x11
#define LSM6DS3_GYRO_ODR_MASK		0xf0
#define LSM6DS3_GYRO_FS_ADDR			0x11
#define LSM6DS3_GYRO_FS_MASK			0x0c
#define LSM6DS3_GYRO_FS_245_VAL		0x00
#define LSM6DS3_GYRO_FS_500_VAL		0x01
#define LSM6DS3_GYRO_FS_1000_VAL		0x02
#define LSM6DS3_GYRO_FS_2000_VAL		0x03
#define LSM6DS3_GYRO_FS_245_GAIN		437
#define LSM6DS3_GYRO_FS_500_GAIN		875
#define LSM6DS3_GYRO_FS_1000_GAIN	1750
#define LSM6DS3_GYRO_FS_2000_GAIN	7000
#define LSM6DS3_GYRO_OUT_X_L_ADDR		0x22
#define LSM6DS3_GYRO_OUT_Y_L_ADDR		0x24
#define LSM6DS3_GYRO_OUT_Z_L_ADDR		0x26
#define LSM6DS3_GYRO_AXIS_EN_ADDR		0x19
#define LSM6DS3_GYRO_DRDY_IRQ_MASK		0x02

/* CUSTOM VALUES FOR SIGNIFICANT MOTION SENSOR */
#define LSM6DS3_SIGN_MOTION_EN_ADDR		0x19
#define LSM6DS3_SIGN_MOTION_EN_MASK		0x01
#define LSM6DS3_SIGN_MOTION_DRDY_IRQ_MASK	0x40

/* CUSTOM VALUES FOR STEP DETECTOR SENSOR */
#define LSM6DS3_STEP_DETECTOR_DRDY_IRQ_MASK	0x80

/* CUSTOM VALUES FOR STEP COUNTER SENSOR */
#define LSM6DS3_STEP_COUNTER_DRDY_IRQ_MASK	0x80
#define LSM6DS3_STEP_COUNTER_OUT_L_ADDR	0x4b
#define LSM6DS3_STEP_COUNTER_RES_ADDR	0x19
#define LSM6DS3_STEP_COUNTER_RES_MASK	0x06
#define LSM6DS3_STEP_COUNTER_RES_ALL_EN	0x03
#define LSM6DS3_STEP_COUNTER_RES_FUNC_EN	0x02
#define LSM6DS3_STEP_COUNTER_DURATION_ADDR	0x15

/* CUSTOM VALUES FOR TILT SENSOR */
#define LSM6DS3_TILT_EN_ADDR			0x58
#define LSM6DS3_TILT_EN_MASK			0x20
#define LSM6DS3_TILT_DRDY_IRQ_MASK		0x02

#define LSM6DS3_ENABLE_AXIS			0x07
#define LSM6DS3_FIFO_DIFF_L			0x3a
#define LSM6DS3_FIFO_DIFF_MASK		0x0fff
#define LSM6DS3_FIFO_DATA_OUT_L		0x3e
#define LSM6DS3_FIFO_ELEMENT_LEN_BYTE	6

#define LSM6DS3_SRC_FUNC_ADDR		0x53
#define LSM6DS3_FIFO_DATA_AVL_ADDR		0x3b

#define LSM6DS3_SRC_SIGN_MOTION_DATA_AVL	0x40
#define LSM6DS3_SRC_STEP_DETECTOR_DATA_AVL	0x10
#define LSM6DS3_SRC_TILT_DATA_AVL		0x20
#define LSM6DS3_SRC_STEP_COUNTER_DATA_AVL	0x80
#define LSM6DS3_FIFO_DATA_AVL		0x80

#define MSEC							(1000)
#define NSEC							(1000000000)
#define MSEC_TO_SEC(tm)					(tm / MSEC)
#define NSEC_TO_SEC(tn)					(tn / NSEC)
#define MSEC_TO_NSEC(tm)				(tm * NSEC / MSEC)
#define SEC_TO_HZ(s)					(1 / s)
#define MSEC_TO_HZ(ms)					(MSEC / ms)
#define NSEC_TO_HZ(ns)					(NSEC / ns)
#define HZ_TO_SEC(h)					(1 / h)
#define HZ_TO_MSEC(h)					(MSEC / h)
#define HZ_TO_NSEC(h)					(NSEC / h)

#ifndef MAX
#define MAX(a, b)				(((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b)				(((a) < (b)) ? (a) : (b))
#endif

static const struct lsm6ds3_sensor_name {
	const char *name;
	const char *description;
} lsm6ds3_sensor_name[LSM6DS3_SENSORS_NUMB] = {
	[LSM6DS3_ACCEL] = {
			.name = "accel",
			.description = "ST LSM6DS3 Accelerometer Sensor",
	},
	[LSM6DS3_GYRO] = {
			.name = "gyro",
			.description = "ST LSM6DS3 Accelerometer Sensor",
	},
	[LSM6DS3_SIGN_MOTION] = {
			.name = "sign_m",
			.description = "ST LSM6DS3 Significant Motion Sensor",
	},
	[LSM6DS3_STEP_COUNTER] = {
			.name = "step_c",
			.description = "ST LSM6DS3 Step Counter Sensor",
	},
	[LSM6DS3_STEP_DETECTOR] = {
			.name = "step_d",
			.description = "ST LSM6DS3 Step Detector Sensor",
	},
	[LSM6DS3_TILT] = {
			.name = "tilt",
			.description = "ST LSM6DS3 Tilt Sensor",
	},
};

struct lsm6ds3_odr_reg {
	u32 hz;
	u8 value;
};

static const struct lsm6ds3_odr_table {
	u8 addr[2];
	u8 mask[2];
	struct lsm6ds3_odr_reg odr_avl[6];
} lsm6ds3_odr_table = {
	.addr[LSM6DS3_ACCEL] = LSM6DS3_ACC_ODR_ADDR,
	.mask[LSM6DS3_ACCEL] = LSM6DS3_ACC_ODR_MASK,
	.addr[LSM6DS3_GYRO] = LSM6DS3_GYR_ODR_ADDR,
	.mask[LSM6DS3_GYRO] = LSM6DS3_GYR_ODR_MASK,
	.odr_avl[0] = { .hz = 26, .value = LSM6DS3_ODR_26HZ_VAL },
	.odr_avl[1] = { .hz = 52, .value = LSM6DS3_ODR_52HZ_VAL },
	.odr_avl[2] = { .hz = 104, .value = LSM6DS3_ODR_104HZ_VAL },
	.odr_avl[3] = { .hz = 208, .value = LSM6DS3_ODR_208HZ_VAL },
	.odr_avl[4] = { .hz = 416, .value = LSM6DS3_ODR_416HZ_VAL },
};

struct lsm6ds3_fs_reg {
	unsigned int gain;
	u8 value;
};

static struct st_lsm6ds3_fs_table {
	u8 addr;
	u8 mask;
	struct lsm6ds3_fs_reg fs_avl[LSM6DS3_FS_LIST_NUM];
} lsm6ds3_fs_table[LSM6DS3_SENSORS_NUMB] = {
	[LSM6DS3_ACCEL] = {
		.addr = LSM6DS3_ACCEL_FS_ADDR,
		.mask = LSM6DS3_ACCEL_FS_MASK,
		.fs_avl[0] = { .gain = LSM6DS3_ACCEL_FS_2G_GAIN,
					.value = LSM6DS3_ACCEL_FS_2G_VAL },
		.fs_avl[1] = { .gain = LSM6DS3_ACCEL_FS_4G_GAIN,
					.value = LSM6DS3_ACCEL_FS_4G_VAL },
		.fs_avl[2] = { .gain = LSM6DS3_ACCEL_FS_8G_GAIN,
					.value = LSM6DS3_ACCEL_FS_8G_VAL },
		.fs_avl[3] = { .gain = LSM6DS3_ACCEL_FS_16G_GAIN,
					.value = LSM6DS3_ACCEL_FS_16G_VAL },
	},
	[LSM6DS3_GYRO] = {
		.addr = LSM6DS3_GYRO_FS_ADDR,
		.mask = LSM6DS3_GYRO_FS_MASK,
		.fs_avl[0] = { .gain = LSM6DS3_GYRO_FS_245_GAIN,
					.value = LSM6DS3_GYRO_FS_245_VAL },
		.fs_avl[1] = { .gain = LSM6DS3_GYRO_FS_500_GAIN,
					.value = LSM6DS3_GYRO_FS_500_VAL },
		.fs_avl[2] = { .gain = LSM6DS3_GYRO_FS_1000_GAIN,
					.value = LSM6DS3_GYRO_FS_1000_VAL },
		.fs_avl[3] = { .gain = LSM6DS3_GYRO_FS_2000_GAIN,
					.value = LSM6DS3_GYRO_FS_2000_VAL },
	}
};

static struct workqueue_struct *lsm6ds3_workqueue = 0;

static inline void lsm6ds3_flush_works()
{
	flush_workqueue(lsm6ds3_workqueue);
}

static inline s64 lsm6ds3_get_time_ns(void)
{
	struct timespec ts;
	/*
	 * calls getnstimeofday.
	 * If hrtimers then up to ns accurate, if not microsecond.
	 */
	ktime_get_real_ts(&ts);

	return timespec_to_ns(&ts);
}

static int lsm6ds3_write_data_with_mask(struct lsm6ds3_data *cdata,
				u8 reg_addr, u8 mask, u8 data, bool b_lock)
{
	int err;
	u8 new_data = 0x00, old_data = 0x00;

	err = cdata->tf->read(cdata, reg_addr, 1, &old_data, b_lock);
	if (err < 0)
		return err;

	new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));

	if (new_data == old_data)
		return 1;

	return cdata->tf->write(cdata, reg_addr, 1, &new_data, b_lock);
}

static int lsm6ds3_input_init(struct lsm6ds3_sensor_data *sdata, u16 bustype,
															const char *description)
{
	int err = 0;

	sdata->input_dev = input_allocate_device();
	if (!sdata->input_dev) {
		dev_err(sdata->cdata->dev, "failed to allocate input device");
		return -ENOMEM;
	}

	sdata->input_dev->name = lsm6ds3_sensor_name[sdata->sindex].description;

	sdata->input_dev->id.bustype = bustype;
	sdata->input_dev->dev.parent = sdata->cdata->dev;
	sdata->input_dev->name = description;
	input_set_drvdata(sdata->input_dev, sdata);

	__set_bit(INPUT_EVENT_TYPE, sdata->input_dev->evbit );
	__set_bit(INPUT_EVENT_TIME_MSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_TIME_LSB, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_X, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_Y, sdata->input_dev->mscbit);
	__set_bit(INPUT_EVENT_Z, sdata->input_dev->mscbit);

	err = input_register_device(sdata->input_dev);
	if (err) {
		dev_err(sdata->cdata->dev, "unable to register sensor %s\n", sdata->name);
		input_free_device(sdata->input_dev);
	}

	return err;
}

static void lsm6ds3_input_cleanup(struct lsm6ds3_sensor_data *sdata)
{
	input_unregister_device(sdata->input_dev);
	input_free_device(sdata->input_dev);
}

static void lsm6ds3_report_3axes_event(struct lsm6ds3_sensor_data *sdata,
													s32 *xyz, int64_t timestamp)
{
	struct input_dev  * input  	= sdata->input_dev;

	if (!sdata->enabled)
		return;

	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_X, xyz[0]);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_Y, xyz[1]);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_Z, xyz[2]);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB, timestamp >> 32);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
														timestamp & 0xffffffff);
	input_sync(input);
}

static void lsm6ds3_report_single_event(struct lsm6ds3_sensor_data *sdata,
													s32 data, int64_t timestamp)
{
	struct input_dev  * input  	= sdata->input_dev;

	if (!sdata->enabled)
		return;

	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_X, data);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB, timestamp >> 32);
	input_event(input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
														timestamp & 0xffffffff);
	input_sync(input);
}

static void lsm6ds3_push_data_with_timestamp(struct lsm6ds3_sensor_data *sdata,
									u16 offset, int64_t timestamp)
{
	s32 data[3];

	data[0] = (s32)((s16)(sdata->cdata->fifo_data_buffer[offset] |
							(sdata->cdata->fifo_data_buffer[offset + 1] << 8)));
	data[1] = (s32)((s16)(sdata->cdata->fifo_data_buffer[offset + 2] |
							(sdata->cdata->fifo_data_buffer[offset + 3] << 8)));
	data[2] = (s32)((s16)(sdata->cdata->fifo_data_buffer[offset + 4] |
							(sdata->cdata->fifo_data_buffer[offset + 5] << 8)));

	data[0] *= sdata->c_gain;
	data[1] *= sdata->c_gain;
	data[2] *= sdata->c_gain;

	dev_info(sdata->cdata->dev, "push data %s\t%d\t[%d,\t%d,\t%d]\t\t timestamp = %ul",
						sdata->name, offset / 6, data[0], data[1], data[2], timestamp);
	lsm6ds3_report_3axes_event(sdata, data, timestamp);
}

static void lsm6ds3_parse_fifo_data(struct lsm6ds3_data *cdata, u16 read_len)
{
	u16 fifo_offset = 0;
	u8 gyro_sip, accel_sip;

	while (fifo_offset < read_len) {
		gyro_sip = cdata->gyro_samples_in_pattern;
		accel_sip = cdata->accel_samples_in_pattern;

		do {
			if (gyro_sip > 0) {
				if (cdata->fifo_reconfigured) {
					cdata->sensors[LSM6DS3_GYRO].timestamp = cdata->timestamp;
					cdata->fifo_reconfigured = false;
				} else
					cdata->sensors[LSM6DS3_GYRO].timestamp += cdata->gyro_deltatime;

				lsm6ds3_push_data_with_timestamp(&cdata->sensors[LSM6DS3_GYRO],
							fifo_offset, cdata->sensors[LSM6DS3_GYRO].timestamp);

				fifo_offset += LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
				gyro_sip--;
			}

			if (accel_sip > 0) {
				if (cdata->fifo_reconfigured) {
					cdata->sensors[LSM6DS3_ACCEL].timestamp  = cdata->timestamp;
					cdata->fifo_reconfigured = false;
				} else
					cdata->sensors[LSM6DS3_ACCEL].timestamp += cdata->accel_deltatime;

				lsm6ds3_push_data_with_timestamp(&cdata->sensors[LSM6DS3_ACCEL],
						fifo_offset, cdata->sensors[LSM6DS3_ACCEL].timestamp);

				fifo_offset += LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
				accel_sip--;
			}
		} while ((accel_sip > 0) || (gyro_sip > 0));
	}

	return;
}

void lsm6ds3_read_fifo(struct lsm6ds3_data *cdata, bool check_fifo_len)
{
	int err;
	u16 read_len;

	if (!cdata->fifo_data_buffer)
		return;

	if (check_fifo_len) {
		err = cdata->tf->read(cdata, LSM6DS3_FIFO_DIFF_L, 2, (u8 *)&read_len,
																	true);
		if (err < 0)
			return;

		read_len &= LSM6DS3_FIFO_DIFF_MASK;

		if (read_len > cdata->fifo_threshold)
			read_len = cdata->fifo_threshold;
	} else
		read_len = cdata->fifo_threshold;

	if (read_len == 0)
		return;

	err = cdata->tf->read(cdata, LSM6DS3_FIFO_DATA_OUT_L, read_len,
													cdata->fifo_data_buffer, true);
	if (err < 0)
		return;

	lsm6ds3_parse_fifo_data(cdata, read_len);
}

static int lsm6ds3_set_fifo_enable(struct lsm6ds3_data *cdata, bool status)
{
	u8 reg_value;

	if (status)
		reg_value = LSM6DS3_FIFO_ODR_MAX;
	else
		reg_value = LSM6DS3_FIFO_ODR_OFF;

	return lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_FIFO_ODR_ADDR,
					LSM6DS3_FIFO_ODR_MASK,
					reg_value, true);
}

int lsm6ds3_set_fifo_mode(struct lsm6ds3_data *cdata, enum fifo_mode fm)
{
	int err;
	u8 reg_value;
	bool enable_fifo;

	switch (fm) {
	case BYPASS:
		reg_value = LSM6DS3_FIFO_MODE_BYPASS;
		enable_fifo = false;
		break;
	case CONTINUOS:
		reg_value = LSM6DS3_FIFO_MODE_CONTINUOS;
		enable_fifo = true;
		break;
	default:
		return -EINVAL;
	}

	err = lsm6ds3_set_fifo_enable(cdata, enable_fifo);
	if (err < 0)
		return err;

	return lsm6ds3_write_data_with_mask(cdata, LSM6DS3_FIFO_MODE_ADDR,
				LSM6DS3_FIFO_MODE_MASK, reg_value, true);
}

int lsm6ds3_set_fifo_decimators_and_threshold(struct lsm6ds3_data *cdata)
{
	int err;
	unsigned int min_odr = 416, max_odr = 0;
	u8 accel_decimator = 0, gyro_decimator = 0;
	struct lsm6ds3_sensor_data *sdata_accel, *sdata_gyro;
	u16 fifo_len, fifo_threshold, fifo_len_accel = 0, fifo_len_gyro = 0;
	u16 num_pattern_accel = 0, num_pattern_gyro = 0, min_num_pattern;

	sdata_accel = &cdata->sensors[LSM6DS3_ACCEL];
	if (sdata_accel->enabled) {
		if (min_odr > sdata_accel->c_odr)
			min_odr = sdata_accel->c_odr;

		if (max_odr < sdata_accel->c_odr)
			max_odr = sdata_accel->c_odr;

		fifo_len_accel = (sdata_accel->fifo_length);
	}

	sdata_gyro = &cdata->sensors[LSM6DS3_GYRO];
	if (sdata_gyro->enabled) {
		if (min_odr > sdata_gyro->c_odr)
			min_odr = sdata_gyro->c_odr;

		if (max_odr < sdata_gyro->c_odr)
			max_odr = sdata_gyro->c_odr;

		fifo_len_gyro = (sdata_gyro->fifo_length);
	}

	if (sdata_accel->enabled) {
		cdata->accel_samples_in_pattern = (sdata_accel->c_odr / min_odr);
		num_pattern_accel = fifo_len_accel / cdata->accel_samples_in_pattern;
		cdata->accel_deltatime = (1000000000ULL / sdata_accel->c_odr);
		accel_decimator = max_odr / sdata_accel->c_odr;
	} else
		cdata->accel_samples_in_pattern = 0;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_FIFO_DECIMATOR_ADDR,
					LSM6DS3_FIFO_ACCEL_DECIMATOR_MASK,
					accel_decimator, true);
	if (err < 0)
		return err;

	if (sdata_gyro->enabled) {
		cdata->gyro_samples_in_pattern = (sdata_gyro->c_odr / min_odr);
		num_pattern_gyro = fifo_len_gyro / cdata->gyro_samples_in_pattern;
		cdata->gyro_deltatime = (1000000000ULL / sdata_gyro->c_odr);
		gyro_decimator = max_odr / sdata_gyro->c_odr;
	} else
		cdata->gyro_samples_in_pattern = 0;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_FIFO_DECIMATOR_ADDR,
					LSM6DS3_FIFO_GYRO_DECIMATOR_MASK,
					gyro_decimator, true);
	if (err < 0)
		return err;

	if ((num_pattern_gyro == 0) || (num_pattern_accel == 0))
		min_num_pattern = MAX(num_pattern_gyro, num_pattern_accel);
	else
		min_num_pattern = MIN(num_pattern_gyro, num_pattern_accel);

	fifo_len = (cdata->accel_samples_in_pattern +
					cdata->gyro_samples_in_pattern) * min_num_pattern *
					LSM6DS3_FIFO_ELEMENT_LEN_BYTE;

	if (fifo_len > 0) {
		fifo_threshold = fifo_len;

		err = cdata->tf->write(cdata, LSM6DS3_FIFO_THR_L_ADDR, 1,
												(u8 *)&fifo_threshold, true);
		if (err < 0)
			return err;

		err = lsm6ds3_write_data_with_mask(cdata,
										LSM6DS3_FIFO_THR_H_ADDR,
										LSM6DS3_FIFO_THR_H_MASK,
										*(((u8 *)&fifo_threshold) + 1), true);
		if (err < 0)
			return err;

		cdata->fifo_threshold = fifo_len;
	}
	if (cdata->fifo_data_buffer) {
		kfree(cdata->fifo_data_buffer);
		cdata->fifo_data_buffer = 0;
	}

	if (fifo_len > 0) {
		cdata->fifo_data_buffer = kmalloc(cdata->fifo_threshold, GFP_KERNEL);
		if (!cdata->fifo_data_buffer)
			return -ENOMEM;
	}

	cdata->fifo_reconfigured = true;

	return fifo_len;
}

int lsm6ds3_reconfigure_fifo(struct lsm6ds3_data *cdata,
													bool disable_irq_and_flush)
{
	int err, fifo_len;

	if (disable_irq_and_flush) {
		disable_irq(cdata->irq);
		lsm6ds3_flush_works();
	}

	mutex_lock(&cdata->fifo_lock);

	lsm6ds3_read_fifo(cdata, true);

	err = lsm6ds3_set_fifo_mode(cdata, BYPASS);
	if (err < 0)
		goto reconfigure_fifo_irq_restore;

	fifo_len = lsm6ds3_set_fifo_decimators_and_threshold(cdata);
	if (fifo_len < 0) {
		err = fifo_len;
		goto reconfigure_fifo_irq_restore;
	}

	if (fifo_len > 0) {
		err = lsm6ds3_set_fifo_mode(cdata, CONTINUOS);
		if (err < 0)
			goto reconfigure_fifo_irq_restore;
	}

reconfigure_fifo_irq_restore:
	mutex_unlock(&cdata->fifo_lock);

	if (disable_irq_and_flush)
		enable_irq(cdata->irq);

	return err;
}

int lsm6ds3_set_drdy_irq(struct lsm6ds3_sensor_data *sdata, bool state)
{
	u8 reg_addr, mask, value;

	if (state)
		value = LSM6DS3_EN_BIT;
	else
		value = LSM6DS3_DIS_BIT;

	switch (sdata->sindex) {
	case LSM6DS3_ACCEL:
		if (sdata->cdata->sensors[LSM6DS3_GYRO].enabled)
			return 0;

		reg_addr = LSM6DS3_INT1_ADDR;
		mask = LSM6DS3_FIFO_THR_IRQ_MASK;
		break;
	case LSM6DS3_GYRO:
		if (sdata->cdata->sensors[LSM6DS3_ACCEL].enabled)
			return 0;

		reg_addr = LSM6DS3_INT1_ADDR;
		mask = LSM6DS3_FIFO_THR_IRQ_MASK;
		break;
	case LSM6DS3_SIGN_MOTION:
		reg_addr = LSM6DS3_INT1_ADDR;
		mask = LSM6DS3_SIGN_MOTION_DRDY_IRQ_MASK;
		break;
	case LSM6DS3_STEP_COUNTER:
	case LSM6DS3_STEP_DETECTOR:
		reg_addr = LSM6DS3_INT1_ADDR;
		mask = LSM6DS3_STEP_DETECTOR_DRDY_IRQ_MASK;
		break;
	case LSM6DS3_TILT:
		reg_addr = LSM6DS3_MD1_ADDR;
		mask = LSM6DS3_TILT_DRDY_IRQ_MASK;
		break;
	default:
		return -EINVAL;
	}

	return lsm6ds3_write_data_with_mask(sdata->cdata, reg_addr, mask, value,
																		true);
}

static int lsm6ds3_set_fs(struct lsm6ds3_sensor_data *sdata, u32 gain)
{
	int err, i;

	for (i = 0; i < LSM6DS3_FS_LIST_NUM; i++) {
		if (lsm6ds3_fs_table[sdata->sindex].fs_avl[i].gain == gain)
			break;
	}

	if (i == LSM6DS3_FS_LIST_NUM)
		return -EINVAL;


	err = lsm6ds3_write_data_with_mask(sdata->cdata,
				lsm6ds3_fs_table[sdata->sindex].addr,
				lsm6ds3_fs_table[sdata->sindex].mask,
				lsm6ds3_fs_table[sdata->sindex].fs_avl[i].value, true);
	if (err < 0)
		return err;

	sdata->c_gain = gain;

	return 0;
}

irqreturn_t lsm6ds3_save_timestamp(int irq, void *private)
{
	struct lsm6ds3_data *cdata = private;

	cdata->timestamp = lsm6ds3_get_time_ns();
	queue_work(lsm6ds3_workqueue, &cdata->input_work);

	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

static void lsm6ds3_irq_management(struct work_struct *input_work)
{
	struct lsm6ds3_data *cdata;
	u8 src_value = 0x00, src_fifo = 0x00;

	cdata = container_of((struct work_struct *)input_work,
						struct lsm6ds3_data, input_work);

	cdata->tf->read(cdata, LSM6DS3_SRC_FUNC_ADDR, 1, &src_value, true);
	cdata->tf->read(cdata, LSM6DS3_FIFO_DATA_AVL_ADDR, 1, &src_fifo, true);

	printk("IRQ received 0x%x\n", src_fifo);
	if (src_fifo & LSM6DS3_FIFO_DATA_AVL)
		lsm6ds3_read_fifo(cdata, false);

	if (src_value & LSM6DS3_SRC_SIGN_MOTION_DATA_AVL) {
		printk("Significat Motion data available. Implement pushing event function!!!!\n\n");
	}

	if (src_value & LSM6DS3_SRC_STEP_DETECTOR_DATA_AVL) {
		printk("Step Detector data available. Implement pushing event function!!!!\n\n");
	}

	if (src_value & LSM6DS3_SRC_STEP_COUNTER_DATA_AVL) {
		printk("Step Counter data available. Implement pushing event function!!!!\n\n");
	}

	if (src_value & LSM6DS3_SRC_TILT_DATA_AVL) {
		printk("Tilt data available. Implement pushing event function!!!!\n\n");
	}

	enable_irq(cdata->irq);
	return;
}

int lsm6ds3_allocate_workqueue(struct lsm6ds3_data *cdata)
{
	int err;

	if (!lsm6ds3_workqueue)
		lsm6ds3_workqueue = create_workqueue(cdata->name);

	if (!lsm6ds3_workqueue)
		return -EINVAL;

	INIT_WORK(&cdata->input_work, lsm6ds3_irq_management);

	err = request_threaded_irq(cdata->irq, lsm6ds3_save_timestamp, NULL,
			IRQF_TRIGGER_RISING, cdata->name, cdata);
	if (err)
		return err;

	return 0;
}

static int lsm6ds3_set_extra_dependency(struct lsm6ds3_sensor_data *sdata,
								bool enable)
{
	int err;

	if (!(sdata->cdata->sensors[LSM6DS3_SIGN_MOTION].enabled |
				sdata->cdata->sensors[LSM6DS3_STEP_COUNTER].enabled |
				sdata->cdata->sensors[LSM6DS3_STEP_DETECTOR].enabled |
				sdata->cdata->sensors[LSM6DS3_TILT].enabled)) {
		if (enable) {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
								LSM6DS3_FUNC_EN_ADDR,
								LSM6DS3_FUNC_EN_MASK,
								LSM6DS3_EN_BIT, true);
			if (err < 0)
				return err;
		} else {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
								LSM6DS3_FUNC_EN_ADDR,
								LSM6DS3_FUNC_EN_MASK,
								LSM6DS3_DIS_BIT, true);
			if (err < 0)
				return err;
		}
	}

	if (!sdata->cdata->sensors[LSM6DS3_ACCEL].enabled) {
		if (enable) {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
							lsm6ds3_odr_table.addr[LSM6DS3_ACCEL],
							lsm6ds3_odr_table.mask[LSM6DS3_ACCEL],
							lsm6ds3_odr_table.odr_avl[0].value, true);
			if (err < 0)
				return err;
		} else {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
							lsm6ds3_odr_table.addr[LSM6DS3_ACCEL],
							lsm6ds3_odr_table.mask[LSM6DS3_ACCEL],
							LSM6DS3_ODR_POWER_OFF_VAL, true);
			if (err < 0)
				return err;
		}
	}

	return 0;
}

static int lsm6ds3_enable_pedometer(struct lsm6ds3_sensor_data *sdata,
								bool enable)
{
	u8 value = LSM6DS3_DIS_BIT;

	if (sdata->cdata->sensors[LSM6DS3_STEP_COUNTER].enabled &&
			sdata->cdata->sensors[LSM6DS3_STEP_DETECTOR].enabled)
		return 0;

	if (enable)
		value = LSM6DS3_EN_BIT;

	return lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_PEDOMETER_EN_ADDR,
						LSM6DS3_PEDOMETER_EN_MASK,
						value, true);

}

static int lsm6ds3_enable_sensors(struct lsm6ds3_sensor_data *sdata)
{
	int err, i;

	if (sdata->enabled)
		return 0;

	switch (sdata->sindex) {
	case LSM6DS3_ACCEL:
	case LSM6DS3_GYRO:
		for (i = 0; i < LSM6DS3_ODR_LIST_NUM; i++) {
			if (lsm6ds3_odr_table.odr_avl[i].hz == sdata->c_odr)
				break;
		}
		if (i == LSM6DS3_ODR_LIST_NUM)
			return -EINVAL;

		err = lsm6ds3_write_data_with_mask(sdata->cdata,
				lsm6ds3_odr_table.addr[sdata->sindex],
				lsm6ds3_odr_table.mask[sdata->sindex],
				lsm6ds3_odr_table.odr_avl[i].value, true);
		if (err < 0)
			return err;

		sdata->c_odr = lsm6ds3_odr_table.odr_avl[i].hz;

		if (sdata->sindex == LSM6DS3_GYRO)
			msleep(LSM6DS3_GYRO_READY_TIME_FROM_PD_MS);
		else
			msleep(LSM6DS3_ACCEL_READY_TIME_FROM_PD_MS);

		break;
	case LSM6DS3_SIGN_MOTION:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_SIGN_MOTION_EN_ADDR,
						LSM6DS3_SIGN_MOTION_EN_MASK,
						LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

		break;
	case LSM6DS3_STEP_COUNTER:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
							LSM6DS3_TIMER_EN_ADDR,
							LSM6DS3_TIMER_EN_MASK,
							LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

		break;
	case LSM6DS3_STEP_DETECTOR:
		err = lsm6ds3_enable_pedometer(sdata, true);
		if (err < 0)
			return err;

		break;
	case LSM6DS3_TILT:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
					LSM6DS3_TILT_EN_ADDR,
					LSM6DS3_TILT_EN_MASK,
					LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

		break;
	default:
		return -EINVAL;
	}

	err = lsm6ds3_set_extra_dependency(sdata, true);
	if (err < 0)
		return err;


	err = lsm6ds3_set_drdy_irq(sdata, true);
	if (err < 0)
		return err;

	sdata->enabled = true;

	err = lsm6ds3_reconfigure_fifo(sdata->cdata, true);
	if (err < 0)
		return err;

	return 0;
}

static int lsm6ds3_disable_sensors(struct lsm6ds3_sensor_data *sdata)
{
	int err;

	if (!sdata->enabled)
		return 0;

	switch (sdata->sindex) {
	case LSM6DS3_ACCEL:
		if (sdata->cdata->sensors[LSM6DS3_SIGN_MOTION].enabled |
						sdata->cdata->sensors[LSM6DS3_STEP_COUNTER].enabled |
						sdata->cdata->sensors[LSM6DS3_STEP_DETECTOR].enabled |
						sdata->cdata->sensors[LSM6DS3_TILT].enabled) {

			err = lsm6ds3_write_data_with_mask(sdata->cdata,
									lsm6ds3_odr_table.addr[LSM6DS3_ACCEL],
									lsm6ds3_odr_table.mask[LSM6DS3_ACCEL],
									lsm6ds3_odr_table.odr_avl[0].value, true);
		} else {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
									lsm6ds3_odr_table.addr[LSM6DS3_ACCEL],
									lsm6ds3_odr_table.mask[LSM6DS3_ACCEL],
									LSM6DS3_ODR_POWER_OFF_VAL, true);
		}
		if (err < 0)
			return err;

		break;
	case LSM6DS3_GYRO:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
									lsm6ds3_odr_table.addr[LSM6DS3_GYRO],
									lsm6ds3_odr_table.mask[LSM6DS3_GYRO],
									LSM6DS3_ODR_POWER_OFF_VAL, true);
		if (err < 0)
			return err;

		break;
	case LSM6DS3_SIGN_MOTION:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
									LSM6DS3_SIGN_MOTION_EN_ADDR,
									LSM6DS3_SIGN_MOTION_EN_MASK,
									LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;

		break;
	case LSM6DS3_STEP_COUNTER:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
							LSM6DS3_TIMER_EN_ADDR,
							LSM6DS3_TIMER_EN_MASK,
							LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;
	case LSM6DS3_STEP_DETECTOR:
		err = lsm6ds3_enable_pedometer(sdata, false);
		if (err < 0)
			return err;

		break;
	case LSM6DS3_TILT:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
							LSM6DS3_TILT_EN_ADDR,
							LSM6DS3_TILT_EN_MASK,
							LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;

		break;
	default:
		return -EINVAL;
	}

	err = lsm6ds3_set_extra_dependency(sdata, false);
	if (err < 0)
		return err;

	err = lsm6ds3_set_drdy_irq(sdata, false);
	if (err < 0)
		return err;

	sdata->enabled = false;

	err = lsm6ds3_reconfigure_fifo(sdata->cdata, true);
	if (err < 0)
		return err;

	return 0;
}

static int lsm6ds3_reset_steps(struct lsm6ds3_data *cdata)
{
	int err;
	u8 reg_value = 0x00;

	err = cdata->tf->read(cdata,
			LSM6DS3_STEP_COUNTER_RES_ADDR, 1, &reg_value, true);
	if (err < 0)
		return err;

	if (reg_value & LSM6DS3_FUNC_EN_MASK)
		reg_value = LSM6DS3_STEP_COUNTER_RES_FUNC_EN;
	else
		reg_value = LSM6DS3_DIS_BIT;

	err = lsm6ds3_write_data_with_mask(cdata,
				LSM6DS3_STEP_COUNTER_RES_ADDR,
				LSM6DS3_STEP_COUNTER_RES_MASK,
				LSM6DS3_STEP_COUNTER_RES_ALL_EN, true);
	if (err < 0)
		return err;

	err = lsm6ds3_write_data_with_mask(cdata,
				LSM6DS3_STEP_COUNTER_RES_ADDR,
				LSM6DS3_STEP_COUNTER_RES_MASK,
				reg_value, true);
	if (err < 0)
		return err;

	cdata->reset_steps = true;

	return 0;
}

static int lsm6ds3_init_sensors(struct lsm6ds3_data *cdata)
{
	int err, i;
	u8 default_reg_value = 0;
	struct lsm6ds3_sensor_data *sdata;

	mutex_init(&cdata->tb.buf_lock);

	for (i = 0; i < LSM6DS3_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];

		err = lsm6ds3_disable_sensors(sdata);
		if (err < 0)
			return err;

		if ((sdata->sindex == LSM6DS3_ACCEL) ||
				(sdata->sindex == LSM6DS3_GYRO)) {
			err = lsm6ds3_set_fs(sdata, sdata->c_gain);
			if (err < 0)
				return err;
		}
	}

	cdata->gyro_selftest_status = 0;
	cdata->accel_selftest_status = 0;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_LIR_ADDR,
					LSM6DS3_LIR_MASK,
					LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_BDU_ADDR,
					LSM6DS3_BDU_MASK,
					LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = lsm6ds3_set_fifo_enable(sdata->cdata, false);
	if (err < 0)
		return err;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_ROUNDING_ADDR,
					LSM6DS3_ROUNDING_MASK,
					LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_INT2_ON_INT1_ADDR,
					LSM6DS3_INT2_ON_INT1_MASK,
					LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = lsm6ds3_reset_steps(sdata->cdata);
	if (err < 0)
		return err;

	mutex_lock(&cdata->bank_registers_lock);
	err = lsm6ds3_write_data_with_mask(sdata->cdata,
					LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					LSM6DS3_FUNC_CFG_REG2_MASK,
					LSM6DS3_EN_BIT, false);
	if (err < 0)
		goto lsm6ds3_init_sensor_mutex_unlock;

	err = sdata->cdata->tf->write(sdata->cdata,
					LSM6DS3_STEP_COUNTER_DURATION_ADDR,
					1,
					&default_reg_value, false);
	if (err < 0)
		goto lsm6ds3_init_sensor_mutex_unlock;

	err = lsm6ds3_write_data_with_mask(sdata->cdata,
					LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					LSM6DS3_FUNC_CFG_REG2_MASK,
					LSM6DS3_DIS_BIT, false);
	if (err < 0)
		goto lsm6ds3_init_sensor_mutex_unlock;
	mutex_unlock(&cdata->bank_registers_lock);

	err = lsm6ds3_reconfigure_fifo(cdata, false);
	if (err < 0)
		return err;

	return 0;

lsm6ds3_init_sensor_mutex_unlock:
	mutex_unlock(&cdata->bank_registers_lock);
	return err;
}

static int lsm6ds3_set_odr(struct lsm6ds3_sensor_data *sdata, u32 odr)
{
	int err = 0, i;

	for (i = 0; i < LSM6DS3_ODR_LIST_NUM; i++) {
		if (lsm6ds3_odr_table.odr_avl[i].hz == odr)
			break;
	}
	if (i == LSM6DS3_ODR_LIST_NUM)
		return -EINVAL;

	if (sdata->enabled) {
		disable_irq(sdata->cdata->irq);
		lsm6ds3_flush_works();

		err = lsm6ds3_write_data_with_mask(sdata->cdata,
					lsm6ds3_odr_table.addr[sdata->sindex],
					lsm6ds3_odr_table.mask[sdata->sindex],
					lsm6ds3_odr_table.odr_avl[i].value, true);
		if (err < 0) {
			enable_irq(sdata->cdata->irq);

			return err;
		}

		sdata->c_odr = lsm6ds3_odr_table.odr_avl[i].hz;
		err = lsm6ds3_reconfigure_fifo(sdata->cdata, false);
		if (err < 0)
			return err;

		enable_irq(sdata->cdata->irq);
	} else
		sdata->c_odr = lsm6ds3_odr_table.odr_avl[i].hz;

	return err;
}

static ssize_t get_enable(struct device *dev, struct device_attribute *attr,
																	char *buf)
{
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->enabled);
}

static ssize_t set_enable(struct device *dev, struct device_attribute *attr,
												const char *buf, size_t count)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);
	unsigned long enable;

	if (strict_strtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		err = lsm6ds3_enable_sensors(sdata);
	else
		err = lsm6ds3_disable_sensors(sdata);

	return count;
}

static ssize_t get_polling_rate(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->c_odr);
}

static ssize_t set_polling_rate(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned int odr;
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &odr);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);
	err = lsm6ds3_set_odr(sdata, odr);
	mutex_unlock(&sdata->input_dev->mutex);

	return count;
}

static ssize_t get_fifo_length(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->fifo_length);
}

static ssize_t set_fifo_length(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned int fifo_length;
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &fifo_length);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);
	sdata->fifo_length = fifo_length;
	mutex_unlock(&sdata->input_dev->mutex);

	lsm6ds3_reconfigure_fifo(sdata->cdata, true);

	return count;
}

static ssize_t reset_steps(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned int reset;
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &reset);
	if (err < 0)
		return err;

	lsm6ds3_reset_steps(sdata->cdata);

	return count;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO, get_enable, set_enable);
static DEVICE_ATTR(polling_rate, S_IWUSR | S_IRUGO, get_polling_rate,
															set_polling_rate);
static DEVICE_ATTR(fifo_length, S_IWUSR | S_IRUGO, get_fifo_length,
															set_fifo_length);
static DEVICE_ATTR(reset_steps, S_IWUSR, NULL, reset_steps);

static struct attribute *lsm6ds3_accel_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_fifo_length.attr,
	NULL,
};

static struct attribute *lsm6ds3_gyro_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_polling_rate.attr,
	&dev_attr_fifo_length.attr,
	NULL,
};

static struct attribute *lsm6ds3_sign_m_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_fifo_length.attr,
	NULL,
};

static struct attribute *lsm6ds3_step_c_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_fifo_length.attr,
	&dev_attr_reset_steps.attr,
	NULL,
};

static struct attribute *lsm6ds3_step_d_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_fifo_length.attr,
	NULL,
};

static struct attribute *lsm6ds3_tilt_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_fifo_length.attr,
	NULL,
};

static const struct attribute_group lsm6ds3_attribute_groups[] = {
	[LSM6DS3_ACCEL] = {
		.attrs = lsm6ds3_accel_attribute,
		.name = "accel",
	},
	[LSM6DS3_GYRO] = {
		.attrs = lsm6ds3_gyro_attribute,
		.name = "gyro",
	},
	[LSM6DS3_SIGN_MOTION] = {
		.attrs = lsm6ds3_sign_m_attribute,
		.name = "sign_m",
	},
	[LSM6DS3_STEP_COUNTER] = {
		.attrs = lsm6ds3_step_c_attribute,
		.name = "step_c",
	},
	[LSM6DS3_STEP_DETECTOR] = {
		.attrs = lsm6ds3_step_d_attribute,
		.name = "step_d",
	},
	[LSM6DS3_TILT] = {
		.attrs = lsm6ds3_tilt_attribute,
		.name = "tilt",
	},
};

#ifdef CONFIG_OF
static const struct of_device_id lsm6ds3_dt_id[] = {
	{.compatible = "st,lsm6ds3",},
	{},
};
MODULE_DEVICE_TABLE(of, lsm6ds3_dt_id);

static u32 lsm6ds3_parse_dt(struct lsm6ds3_data *cdata)
{
	u32 val;
	struct device_node *np;

	np = cdata->dev->of_node;
	if (!np)
		return -EINVAL;

	if (!of_property_read_u32(np, "st,drdy-int-pin", &val) &&
							(val <= 2) && (val > 0))
		cdata->drdy_int_pin = (u8)val;
	else
		cdata->drdy_int_pin = 1;

	return 0;
}

#else
#endif

int lsm6ds3_common_probe(struct lsm6ds3_data *cdata, int irq, u16 bustype)
{
	/* TODO: add errors management */
	int32_t err, i;
	u8 wai =0x00;
	struct lsm6ds3_sensor_data *sdata;

	mutex_init(&cdata->bank_registers_lock);
	mutex_init(&cdata->fifo_lock);
	cdata->fifo_data_buffer = 0;

	err = cdata->tf->read(cdata, LSM6DS3_WAI_ADDRESS, 1, &wai, true);
	if (err < 0) {
		dev_err(cdata->dev, "failed to read Who-Am-I register.\n");
		return err;
	}
	if (wai != LSM6DS3_WAI_EXP) {
		dev_err(cdata->dev, "Who-Am-I value not valid.\n");
		return -ENODEV;
	}

	mutex_init(&cdata->lock);

	if (irq > 0) {
#ifdef CONFIG_OF
		err = lsm6ds3_parse_dt(cdata);
		if (err < 0)
			return err;
#else /* CONFIG_OF */
		if (cdata->dev->platform_data) {
			cdata->drdy_int_pin = ((struct lsm6ds3_platform_data *)
								cdata->dev->platform_data)->drdy_int_pin;

			if ((cdata->drdy_int_pin > 2) || (cdata->drdy_int_pin < 1))
				cdata->drdy_int_pin = 1;
		} else
			cdata->drdy_int_pin = 1;
#endif /* CONFIG_OF */

		dev_info(cdata->dev, "driver use DRDY int pin %d\n",
														cdata->drdy_int_pin);
	}

	for (i = 0; i < LSM6DS3_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];
		sdata->enabled = false;
		sdata->cdata = cdata;
		sdata->sindex = i;
		sdata->name = lsm6ds3_sensor_name[i].name;
		sdata->fifo_length = 1;
		if ((i == LSM6DS3_ACCEL) || (i == LSM6DS3_GYRO)) {
			sdata->c_odr = lsm6ds3_odr_table.odr_avl[0].hz;
			sdata->c_gain = lsm6ds3_fs_table[i].fs_avl[0].gain;
		}

		lsm6ds3_input_init(sdata, bustype, lsm6ds3_sensor_name[i].description);
		if (sysfs_create_group(&sdata->input_dev->dev.kobj, &lsm6ds3_attribute_groups[i])) {
			dev_err(cdata->dev, "failed to create sysfs group for sensor %s", sdata->name);
			input_unregister_device(sdata->input_dev);
			sdata->input_dev = NULL;
		}
	}

	if(lsm6ds3_workqueue == 0)
		lsm6ds3_workqueue =
			create_workqueue("lsm6ds3_workqueue");

	err = lsm6ds3_init_sensors(cdata);
	if (err < 0)
		return err;
	if (irq > 0) {
		cdata->irq = irq;
		dev_info(cdata->dev, "driver use DRDY int pin 1\n");
	}

	if (irq > 0) {
		err = lsm6ds3_allocate_workqueue(cdata);
		if (err < 0)
			return err;
	}

	cdata->fifo_data_buffer = kmalloc(FIFO_SIZE_BYTE, GFP_KERNEL);
	if (!cdata->fifo_data_buffer)
			return -ENOMEM;

	cdata->fifo_data_size = FIFO_SIZE_BYTE;

	dev_info(cdata->dev, "%s: probed\n", LSM6DS3_ACC_GYR_DEV_NAME);
	return 0;
}

void lsm6ds3_common_remove(struct lsm6ds3_data *cdata, int irq)
{
	u8 i;

	for (i = 0; i < LSM6DS3_SENSORS_NUMB; i++) {
		lsm6ds3_disable_sensors(&cdata->sensors[i]);
		lsm6ds3_input_cleanup(&cdata->sensors[i]);
	}

	//remove_sysfs_interfaces(cdata->dev);

	if(!lsm6ds3_workqueue) {
		flush_workqueue(lsm6ds3_workqueue);
		destroy_workqueue(lsm6ds3_workqueue);
	}

	kfree(cdata->fifo_data_buffer);
}

#ifdef CONFIG_PM
int lsm6ds3_common_suspend(struct lsm6ds3_data *cdata)
{
	return 0;
}
EXPORT_SYMBOL(lsm6ds3_common_suspend);

int lsm6ds3_common_resume(struct lsm6ds3_data *cdata)
{
	return 0;
}
EXPORT_SYMBOL(lsm6ds3_common_resume);
#endif /* CONFIG_PM */

