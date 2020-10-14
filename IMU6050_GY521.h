/*
 * IMU6050_GY521.h
 *
 *  Created on: Okt 10, 2020
 *      Author: Christian
 */

#ifndef INC_IMU6050_GY521_H_
#define INC_IMU6050_GY521_H_

#include "stm32l4xx_hal.h"
//#import <stdint.h>




/**
 * Variablen
 *
 * TODO Welche sind global wichtig?
 * Welche können in die IMU...c verschoben werden?
 */
#define IMU6050_ADDR 0xD0

/* @brief Define I2C Register Addresses */
#define IMU6050_SMPLRT_DIV_REG 	0x19
#define IMU6050_GYRO_CONFIG_REG 0x1B
#define IMU6050_ACCL_CONFIG_REG 0x1C
#define IMU6050_ACCL_XOUT_H_REG 0x3B
#define IMU6050_TEMP_OUT_H_REG 	0x41
#define IMU6050_GYRO_XOUT_H_REG 0x43
#define IMU6050_PWR_MGMT_1_REG	0x6B
#define IMU6050_WHO_AM_I_REG 	0x75
/* define Device ID Check (WHO_AM_I_REG default value) */
#define IMU6050_I_AM 	0x68
/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENS_250		((float)1 / ((float) 131))
#define MPU6050_GYRO_SENS_500		((float)1 / ((float) 65.5))
#define MPU6050_GYRO_SENS_1000		((float)1 / ((float) 32.8))
#define MPU6050_GYRO_SENS_2000		((float)1 / ((float) 16.4))
/* Acce sensitivities in g/s */
#define MPU6050_ACCL_SENS_2			((float)1 / ((float) 16384.0))
#define MPU6050_ACCL_SENS_4			((float)1 / ((float) 8192.0))
#define MPU6050_ACCL_SENS_8			((float)1 / ((float) 4096.0))
#define MPU6050_ACCL_SENS_16		((float)1 / ((float) 2048.0))
/* @brief Define IMU Output variables */

// Define possible Sample rates
#define MPU6050_SampleRate_8KHz       0x00   /*!< Sample rate set to 8 kHz */
#define MPU6050_SampleRate_4KHz       0x01   /*!< Sample rate set to 4 kHz */
#define MPU6050_SampleRate_2KHz       0x03   /*!< Sample rate set to 2 kHz */
#define MPU6050_SampleRate_1KHz       0x07   /*!< Sample rate set to 1 kHz */
#define MPU6050_SampleRate_500Hz      0x0F  /*!< Sample rate set to 500 Hz */
#define MPU6050_SampleRate_250Hz      0x1F  /*!< Sample rate set to 250 Hz */
#define MPU6050_SampleRate_125Hz      0x3F  /*!< Sample rate set to 125 Hz */
#define MPU6050_SampleRate_100Hz      0x4F  /*!< Sample rate set to 100 Hz */

// Define possible number of calibration Measurements
#define MPU6050_Calibration_short		100
#define MPU6050_Calibration_long		500
#define MPU6050_Calibration_extended	1000

typedef struct {
	int16_t Gyro_X;
	int16_t Gyro_Y;
	int16_t Gyro_Z;
	int16_t	Gx_offset;
	int16_t	Gy_offset;
	int16_t	Gz_offset;
	float	Gyro_mult;
	float 	Gyro_time;
	int16_t Accl_X;
	int16_t Accl_Y;
	int16_t Accl_Z;
	float	Accl_mult;
	float 	Accl_time;
}IMU6050_Data;

typedef enum{
	MPU6050_Status_OK= 0x00,
	MPU6050_Read_OK,
	MPU6050_Status_Error,
	MPU6050_Read_Error,
	MPU6050_Status_Notconnected,
	MPU6050_Status_Invalid
}MPU6050_STAUTS;


// Set Accl_Res to 0, 1, 2, 3 for following resolution:
typedef enum {
	MPU6050_ACCL_2G = 0x00,		// 0 ±2g  16384 LSB/g
	MPU6050_ACCL_4G,			// 1 ±4g   8192 LSB/g
	MPU6050_ACCL_8G,			// 2 ±8g   4096 LSB/g
	MPU6050_ACCL_16G			// 3 ±16g  2048 LSB/g
} MPU6050_ACCL_RES;

// Set Gyro_Res to 0, 1, 2, 3 for following resolution:
typedef enum IMU6050_GYRO_RES_ENUM{
	MPU6050_GYRO_250deg = 0x00,		// 0 ±  250 °/s 131   LSB/°/s
	MPU6050_GYRO_500deg,			// 1 ±  500 °/s  65.5 LSB/°/s
	MPU6050_GYRO_1000deg,			// 2 ± 1000 °/s  32.8 LSB/°/s
	MPU6050_GYRO_2000deg			// 3 ± 2000 °/s  16.4 LSB/°/s
} MPU6050_GYRO_RES;


/**
 * Functions
 */
MPU6050_STAUTS IMU6050_init(I2C_HandleTypeDef* __hi2c, IMU6050_Data* DataStruct, MPU6050_ACCL_RES accl_resolution, MPU6050_GYRO_RES gyro_resolution, uint8_t samplerate);
MPU6050_STAUTS IMU6050_Read_Accl(I2C_HandleTypeDef* __hi2c,IMU6050_Data* DataStruct);
MPU6050_STAUTS IMU6050_Read_Gyro(I2C_HandleTypeDef* __hi2c,IMU6050_Data* DataStruct);
MPU6050_STAUTS IMU6050_Calibrate_Gyro(I2C_HandleTypeDef* __hi2c, IMU6050_Data* DataStruct, uint8_t n_measurements);

#endif /* INC_IMU6050_GY521_H_ */
