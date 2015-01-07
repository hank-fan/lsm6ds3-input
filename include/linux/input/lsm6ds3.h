/*
 * STMicroelectronics lsm6ds3 driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef __LSM6DS3_H__
#define __LSM6DS3_H__

//#define CONFIG_OF  /* TODO: remove */

/************************************************/
/* 	Accelerometer section defines	 	*/
/************************************************/

/* Accelerometer Sensor Full Scale */
#define LSM6DS3_ACC_FS_MASK	(0x0c)
#define LSM6DS3_ACC_FS_2G 	(0x00)	/* Full scale 2g */
#define LSM6DS3_ACC_FS_4G 	(0x02)	/* Full scale 4g */
#define LSM6DS3_ACC_FS_8G 	(0x03)	/* Full scale 8g */
#define LSM6DS3_ACC_FS_16G	(0x01)	/* Full scale 16g */

/* Accelerometer Anti-Aliasing Filter */
#define LSM6DS3_ACC_AAFBW_MASK	(0x03)
#define LSM6DS3_ACC_AAFBW_400	(0x00)
#define LSM6DS3_ACC_AAFBW_200	(0x01)
#define LSM6DS3_ACC_AAFBW_100	(0x02)
#define LSM6DS3_ACC_AAFBW_50	(0x03)

#define LSM6DS3_ACC_DFLT_POLL_INTERVAL_USEC	(1000000)	// 1000000[usec]=> 1  [Hz]
#define LSM6DS3_ACC_MIN_POLL_PERIOD_US		(2000)		//    2000[usec]=> 500[Hz]

/************************************************/
/* 	Gyroscope section defines	 	*/
/************************************************/

/* Gyroscope Sensor Full Scale */
#define LSM6DS3_GYR_FS_MASK	(0x0e)
#define LSM6DS3_GYR_FS_125DPS	(0x01)		/* Full scale  125dps */
#define LSM6DS3_GYR_FS_245DPS	(0x00)		/* Full scale  245dps */
#define LSM6DS3_GYR_FS_500DPS	((0x01) << 1)	/* Full scale  500dps */
#define LSM6DS3_GYR_FS_1000DPS	((0x02) << 1)	/* Full scale 1000dps */
#define LSM6DS3_GYR_FS_2000DPS	((0x03) << 1)	/* Full scale 2000dps */

#define LSM6DS3_GYR_DFLT_POLL_INTERVAL_USEC	(1000000)	// 1000000[usec]=> 1  [Hz]
#define LSM6DS3_GYR_MIN_POLL_PERIOD_US		(2000)		//    2000[usec]=> 500[Hz]


#define LSM6DS3_ACC_GYR_DEV_NAME		"lsm6ds3"
#define LSM6DS3_ACC_INPUT_DEV_NAME	"lsm6ds3_acc"
#define LSM6DS3_GYR_INPUT_DEV_NAME	"lsm6ds3_gyr"


#define LSM6DS3_RX_MAX_LENGTH	(500)
#define LSM6DS3_TX_MAX_LENGTH	(500)

#define LSM6DS3_INT1_GPIO_DEF	(-EINVAL)
#define LSM6DS3_INT2_GPIO_DEF	(-EINVAL)

struct lsm6ds3_platform_data {
	u8 drdy_int_pin;
};

#endif /* __LSM6DS3_H__ */
