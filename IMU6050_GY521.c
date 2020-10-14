/*
 * IMU.c
 *
 *  Created on: Oct 05, 2020
 *      Author: Christsian
 */

#include "IMU6050_GY521.h"


MPU6050_STAUTS IMU6050_init(I2C_HandleTypeDef* __hi2c, IMU6050_Data* DataStruct, MPU6050_ACCL_RES accl_resolution, MPU6050_GYRO_RES gyro_resolution, uint8_t samplerate){

	uint8_t Data=0x00;

	//Check if device is ready: Let LED blink 3 times if yes
	if(HAL_I2C_IsDeviceReady(__hi2c, IMU6050_ADDR, 2, 20) == HAL_OK){
    	int t = 0;
		while(t < 6){
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
			HAL_Delay(200);
			t++;
		}
    }
    else
    {
    	return MPU6050_Status_Notconnected;
    }


	//check WHO_AM_I: If answer is I_AM toggle LED
	HAL_I2C_Mem_Read(__hi2c, IMU6050_ADDR, IMU6050_WHO_AM_I_REG, 1, &Data, 1, 1000);

	if (Data == IMU6050_I_AM) // if-statement is 1 if IMU is present
	{
		// Toggle LED LD2
		 HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	}
	else
	{
		return MPU6050_Status_Invalid;
	}



	// wake sensor
	Data=0x00;
	if(HAL_I2C_Mem_Write(__hi2c, IMU6050_ADDR, IMU6050_PWR_MGMT_1_REG, 1, &Data, 1, 1000)!=HAL_OK)
	{
		return MPU6050_Status_Invalid;
	}


	// Config range/resolution Gyroscope
	Data = 0xE0 | (gyro_resolution<<3);
	if(HAL_I2C_Mem_Write(__hi2c, IMU6050_ADDR, IMU6050_GYRO_CONFIG_REG, 1, &Data, 1, 1000) != HAL_OK){
		return MPU6050_Status_Error;
	}
	//for normalization of Gyroscope range
	switch (gyro_resolution){
		case MPU6050_GYRO_250deg:
			DataStruct->Gyro_mult = MPU6050_GYRO_SENS_250;
			break;
		case MPU6050_GYRO_500deg:
			DataStruct->Gyro_mult = MPU6050_GYRO_SENS_500;
			break;
		case MPU6050_GYRO_1000deg:
			DataStruct->Gyro_mult = MPU6050_GYRO_SENS_1000;
			break;
		case MPU6050_GYRO_2000deg:
			DataStruct->Gyro_mult = MPU6050_GYRO_SENS_2000;
			break;
		}


	// Config range/resolution of Accelerometer
	Data = 0xE0 | (accl_resolution<<3);
	if(HAL_I2C_Mem_Write(__hi2c, IMU6050_ADDR, IMU6050_ACCL_CONFIG_REG, 1, &Data, 1, 1000) != HAL_OK){
		return MPU6050_Status_Error;
	}

	switch(accl_resolution){
	case MPU6050_ACCL_2G:
		DataStruct->Accl_mult = MPU6050_ACCL_SENS_2;
			break;
	case MPU6050_ACCL_4G:
		DataStruct->Accl_mult = MPU6050_ACCL_SENS_4;
			break;
	case MPU6050_ACCL_8G:
		DataStruct->Accl_mult = MPU6050_ACCL_SENS_8;
			break;
	case MPU6050_ACCL_16G:
		DataStruct->Accl_mult = MPU6050_ACCL_SENS_16;
			break;
	}


	// Config Sample Rate: Sample Rate=8/(1+Data)
	Data=samplerate;
	if(HAL_I2C_Mem_Write(__hi2c, IMU6050_ADDR, IMU6050_SMPLRT_DIV_REG, 1, &Data, 1, 1000) != HAL_OK){
		return MPU6050_Status_Error;
	}

	//Set Gyroscope offset to zero
	DataStruct->Gx_offset = 0;
	DataStruct->Gy_offset = 0;
	DataStruct->Gz_offset = 0;



	return MPU6050_Status_OK;
}




MPU6050_STAUTS IMU6050_Read_Accl(I2C_HandleTypeDef* __hi2c,IMU6050_Data* DataStruct){

	uint8_t Rec_Data[6];

	// Read 6Bytes (2byte per axis) starting from ACCL_XOUT_H register
	if(HAL_I2C_Mem_Read (__hi2c, IMU6050_ADDR, IMU6050_ACCL_XOUT_H_REG, 1, Rec_Data, 6, 1000) != HAL_OK){
		return MPU6050_Status_Error;
	}

	// Data for all axis comes as 6Byte array, in the following it is filled into one 16-bit array for each axis
	DataStruct->Accl_X = (int16_t)(Rec_Data[0]<<8 | Rec_Data[1]);
	DataStruct->Accl_Y = (int16_t)(Rec_Data[2]<<8 | Rec_Data[3]);
	DataStruct->Accl_Z = (int16_t)(Rec_Data[4]<<8 | Rec_Data[5]);

	// Get time of Measurement
	DataStruct->Accl_time = HAL_GetTick();

	return MPU6050_Read_OK;
}


MPU6050_STAUTS IMU6050_Read_Gyro(I2C_HandleTypeDef* __hi2c,IMU6050_Data* DataStruct){

	uint8_t Rec_Data[6];

	// Read 6Bytes (2byte per axis) starting from GYRO_XOUT_H register
	if(HAL_I2C_Mem_Read (__hi2c, IMU6050_ADDR, IMU6050_GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000) != HAL_OK){
		return MPU6050_Status_Error;
	}

	// Data for all axis comes as 6Byte array, in the following it is filled into one 16-bit array for each axis
	DataStruct->Gyro_X = (int16_t)(Rec_Data[0]<<8 | Rec_Data[1])-DataStruct->Gx_offset;
	DataStruct->Gyro_Y = (int16_t)(Rec_Data[2]<<8 | Rec_Data[3])-DataStruct->Gy_offset;
	DataStruct->Gyro_Z = (int16_t)(Rec_Data[4]<<8 | Rec_Data[5])-DataStruct->Gz_offset;
	// Get time of Measurement
	DataStruct->Gyro_time = HAL_GetTick();

	return MPU6050_Read_OK;
}


MPU6050_STAUTS IMU6050_Calibrate_Gyro(I2C_HandleTypeDef* __hi2c, IMU6050_Data* DataStruct, uint8_t n_measurements){
	int32_t Gx_offset_temp=0;
	int32_t Gy_offset_temp=0;
	int32_t Gz_offset_temp=0;
	for (int i=0; i < n_measurements; i++){
		IMU6050_Read_Gyro(__hi2c,DataStruct);
		int16_t Gx = DataStruct->Gyro_X;
		int16_t Gy = DataStruct->Gyro_Y;
		int16_t Gz = DataStruct->Gyro_Z;
		Gx_offset_temp+=Gx;
		Gy_offset_temp+=Gy;
		Gz_offset_temp+=Gz;
	}
	DataStruct->Gx_offset=Gx_offset_temp/n_measurements;
	DataStruct->Gy_offset=Gy_offset_temp/n_measurements;
	DataStruct->Gz_offset=Gz_offset_temp/n_measurements;

	return MPU6050_Read_OK;
}
