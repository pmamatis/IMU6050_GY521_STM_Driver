

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

	//Set Accelerometer offset to zero
	DataStruct->Ax_offset = 0;
	DataStruct->Ay_offset = 0;
	DataStruct->Az_offset = 0;


	return MPU6050_Status_OK;
}




MPU6050_STAUTS IMU6050_Read_Accl(I2C_HandleTypeDef* __hi2c,IMU6050_Data* DataStruct){

	uint8_t Rec_Data[6];

	// Read 6Bytes (2byte per axis) starting from ACCL_XOUT_H register
	if(HAL_I2C_Mem_Read (__hi2c, IMU6050_ADDR, IMU6050_ACCL_XOUT_H_REG, 1, Rec_Data, 6, 1000) != HAL_OK){
		return MPU6050_Status_Error;
	}

	// Data for all axis comes as 6Byte array, in the following it is filled into one 16-bit array for each axis
	DataStruct->Accl_X = (int16_t)(Rec_Data[0]<<8 | Rec_Data[1]) - DataStruct->Ax_offset;
	DataStruct->Accl_Y = (int16_t)(Rec_Data[2]<<8 | Rec_Data[3]) - DataStruct->Ay_offset;
	DataStruct->Accl_Z = (int16_t)(Rec_Data[4]<<8 | Rec_Data[5]) - DataStruct->Az_offset;

	return MPU6050_Read_OK;
}


MPU6050_STAUTS IMU6050_Read_Gyro(I2C_HandleTypeDef* __hi2c,IMU6050_Data* DataStruct){

	uint8_t Rec_Data[6];

	// Read 6Bytes (2byte per axis) starting from GYRO_XOUT_H register
	if(HAL_I2C_Mem_Read (__hi2c, IMU6050_ADDR, IMU6050_GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000) != HAL_OK){
		return MPU6050_Status_Error;
	}

	// Get time of new Measurement and safe time of last measurement
	DataStruct->Gyro_time	=	DWT->CYCCNT;
	DWT->CYCCNT				=	0;

	// Data for all axis comes as 6Byte array, in the following it is filled into one 16-bit array for each axis
	DataStruct->Gyro_X = (int16_t)(Rec_Data[0]<<8 | Rec_Data[1])-DataStruct->Gx_offset;
	DataStruct->Gyro_Y = (int16_t)(Rec_Data[2]<<8 | Rec_Data[3])-DataStruct->Gy_offset;
	DataStruct->Gyro_Z = (int16_t)(Rec_Data[4]<<8 | Rec_Data[5])-DataStruct->Gz_offset;


	return MPU6050_Read_OK;
}

MPU6050_STAUTS IMU6050_Read_Sensor(I2C_HandleTypeDef* __hi2c,IMU6050_Data* DataStruct){

	uint8_t Rec_Data[14];

		// Read 6Bytes (2byte per axis) starting from ACCL_XOUT_H register
		HAL_I2C_Mem_Read (__hi2c, IMU6050_ADDR, IMU6050_ACCL_XOUT_H_REG, 1, Rec_Data, 14, 1000);
		// Get time of new Measurement and safe time of last measurement
		DataStruct->Gyro_time	=	DWT->CYCCNT;
		DWT->CYCCNT				=	0;
		// Data for all axis comes as 6Byte array, in the following it is filled into one 16-bit array for each axis
		DataStruct->Accl_X = (int16_t)(Rec_Data[0]<<8 | Rec_Data[1]) - DataStruct->Ax_offset;
		DataStruct->Accl_Y = (int16_t)(Rec_Data[2]<<8 | Rec_Data[3]) - DataStruct->Ay_offset;
		DataStruct->Accl_Z = (int16_t)(Rec_Data[4]<<8 | Rec_Data[5]) - DataStruct->Az_offset;
		// Data for all axis comes as 6Byte array, in the following it is filled into one 16-bit array for each axis
		DataStruct->Gyro_X = (int16_t)(Rec_Data[8]<<8 | Rec_Data[9]) - DataStruct->Gx_offset;
		DataStruct->Gyro_Y = (int16_t)(Rec_Data[10]<<8 | Rec_Data[11]) - DataStruct->Gy_offset;
		DataStruct->Gyro_Z = (int16_t)(Rec_Data[12]<<8 | Rec_Data[13]) - DataStruct->Gz_offset;

		return MPU6050_Read_OK;
}


MPU6050_STAUTS IMU6050_Calculate_Mean(I2C_HandleTypeDef* __hi2c, IMU6050_Data* DataStruct){
	// Temp memory to sum up values for mean calculation
	// Gyroscope
	int32_t Gx_mean_tempbuffer	=	0;
	int32_t Gy_mean_tempbuffer	=	0;
	int32_t Gz_mean_tempbuffer	=	0;
	// Accelerometer
	int32_t Ax_mean_tempbuffer	=	0;
	int32_t Ay_mean_tempbuffer	=	0;
	int32_t Az_mean_tempbuffer	=	0;
	// First measurements have to be skipped
	int n_skipping		=	100;
	int n_measurements 	=	1000;

	for (int i=n_skipping; i < n_measurements + n_skipping; i++){
		// Read Data
		// from Gyroscope
		IMU6050_Read_Sensor(__hi2c,DataStruct);
		int16_t Gx = DataStruct	->	Gyro_X;
		int16_t Gy = DataStruct	->	Gyro_Y;
		int16_t Gz = DataStruct	->	Gyro_Z;
		// from Accelerometer
		IMU6050_Read_Accl(__hi2c,DataStruct);
		int16_t Ax = DataStruct	->	Accl_X;
		int16_t Ay = DataStruct	->	Accl_Y;
		int16_t Az = DataStruct	->	Accl_Z;
		// sum it up
		Gx_mean_tempbuffer	+=	Gx;
		Gy_mean_tempbuffer	+=	Gy;
		Gz_mean_tempbuffer	+=	Gz;
		Ax_mean_tempbuffer	+=	Ax;
		Ay_mean_tempbuffer	+=	Ay;
		Az_mean_tempbuffer	+=	Az;
		// Wait to prevent double measurements
		HAL_Delay(2);
	}

	//Save offset to Datastruct
	//Gyroscope offset
	DataStruct->Gx_mean=Gx_mean_tempbuffer/n_measurements;
	DataStruct->Gy_mean=Gy_mean_tempbuffer/n_measurements;
	DataStruct->Gz_mean=Gz_mean_tempbuffer/n_measurements;
	//Accelerometer offset
	DataStruct->Ax_mean=Ax_mean_tempbuffer/n_measurements;
	DataStruct->Ay_mean=Ay_mean_tempbuffer/n_measurements;
	DataStruct->Az_mean=Az_mean_tempbuffer/n_measurements;

	return MPU6050_Read_OK;
}

MPU6050_STAUTS IMU6050_Calibrate(I2C_HandleTypeDef* __hi2c, IMU6050_Data* DataStruct){

	// gravitational constant depending on accelerometer resolution
	int16_t gravitation	= (int16_t)((float)1 / DataStruct -> Accl_mult);
	// allowed deviation from mean for raw data from sensors
	int8_t	gyro_tolerance  = 2;
	int8_t	accl_tolerance  = 8;

	// Set offset Initially (offset is subtracted from future measurements)
	DataStruct -> Gx_offset		=	DataStruct -> Gx_mean;
	DataStruct -> Gy_offset		=	DataStruct -> Gy_mean;
	DataStruct -> Gz_offset		=	DataStruct -> Gz_mean;
	DataStruct -> Ax_offset		=	DataStruct -> Ax_mean;
	DataStruct -> Ay_offset		=	DataStruct -> Ay_mean;
	DataStruct -> Az_offset		=	(DataStruct -> Az_mean) + gravitation;

	while(1){
	// Calculate means of sensors
	IMU6050_Calculate_Mean(__hi2c, DataStruct);
	// Indicator for correct calibration of all axis
	int calibrated = 0;
	// Validate or Update Calibration
	// Gyroscope X Y Z
	if (abs(DataStruct -> Gx_mean)<gyro_tolerance) calibrated++;
	else DataStruct -> Gx_offset		=	(DataStruct -> Gx_offset)	+	(DataStruct -> Gx_mean);

	if (abs(DataStruct -> Gy_mean)<gyro_tolerance) calibrated++;
	else DataStruct -> Gy_offset		=	(DataStruct -> Gy_offset)	+	(DataStruct -> Gy_mean);

	if (abs(DataStruct -> Gz_mean)<gyro_tolerance) calibrated++;
	else DataStruct -> Gz_offset		=	(DataStruct -> Gz_offset)	+	(DataStruct -> Gz_mean);
	// Accelerometer X Y Z
	if (abs(DataStruct -> Ax_mean)<accl_tolerance) calibrated++;
	else DataStruct -> Ax_offset		=	(DataStruct -> Ax_offset)	+	(DataStruct -> Ax_mean);

	if (abs(DataStruct -> Ay_mean)<accl_tolerance) calibrated++;
	else DataStruct -> Ay_offset		=	(DataStruct -> Ay_offset)	+	(DataStruct -> Ay_mean);

	if ((abs((DataStruct -> Az_mean) - gravitation)) <accl_tolerance) calibrated++;
	else DataStruct -> Az_offset		=	(DataStruct -> Az_offset)	+	((DataStruct -> Az_mean) - gravitation);

	if (calibrated == 6){
		break;
	}
	}

	return MPU6050_Read_OK;
}

void getAcclFilt(IMU6050_Data* DataStruct){

}

void getGyroRoll(IMU6050_Data* DataStruct){

	// time of measurement for integration
	float cycletime = 80000000;

	// Get Roll
	// RAW Gyrodata: Gyro_X; transformation to Deg/s: Gyro_mult; time since last measurement: Gyro_time; cycletime per second: cycletime
	DataStruct -> Gx_deg =  (float)( ((DataStruct -> Gyro_X) * DataStruct -> Gyro_mult ) * (DataStruct -> Gyro_time)) / cycletime;
	DataStruct -> Gy_deg =  (float)( ((DataStruct -> Gyro_Y) * DataStruct -> Gyro_mult ) * (DataStruct -> Gyro_time)) / cycletime;
	DataStruct -> Gz_deg =  (float)( ((DataStruct -> Gyro_Z) * DataStruct -> Gyro_mult ) * (DataStruct -> Gyro_time)) / cycletime;

	// Take into account that +-360 degrees is the same as 0 degrees
	if (DataStruct -> Gx_deg >=180) DataStruct -> Gx_deg = DataStruct -> Gx_deg - 360;
	if (DataStruct -> Gx_deg <=-180) DataStruct -> Gx_deg = DataStruct -> Gx_deg + 360;

	if (DataStruct -> Gy_deg >=180) DataStruct -> Gy_deg = DataStruct -> Gy_deg - 360;
	if (DataStruct -> Gy_deg <=-180) DataStruct -> Gy_deg = DataStruct -> Gy_deg + 360;

	if (DataStruct -> Gy_deg >=180) DataStruct -> Gz_deg = DataStruct -> Gz_deg - 360;
	if (DataStruct -> Gy_deg <=-180) DataStruct -> Gz_deg = DataStruct -> Gz_deg + 360;
}


void getAcclRoll(IMU6050_Data* DataStruct){

	//Parameter for stability in Regions where X and Z or Y and Z are near zero
	int16_t mu = 0.01;


	int16_t X = DataStruct -> Accl_X;
	int16_t Y = DataStruct -> Accl_Y;
	int16_t Z = DataStruct -> Accl_Z;

	// parameter for correct orientation
	int16_t 	sign;
	if (Z>0)	sign = 1;
	else 		sign = -1;

	//77transform radians to degree
	float RAD_TO_DEG = 180/M_PI;

	// Get Roll
	// RAW Gyrodata: Gyro_X; transformation to Deg/s: Gyro_mult; time since last measurement: Gyro_time; cycletime per second: cycletime
	DataStruct -> Ax_deg = atan2( Y ,   sign * sqrt(Z*Z+ mu*X*X)) * RAD_TO_DEG;
	DataStruct -> Ay_deg = atan2(-X, sqrt(Y*Y + Z*Z)) * RAD_TO_DEG;

}

void getAngleRoll(I2C_HandleTypeDef* __hi2c, IMU6050_Data* DataStruct){

	IMU6050_Read_Sensor(__hi2c, DataStruct);
	getAcclRoll(DataStruct);
	getGyroRoll(DataStruct);

	float X = DataStruct -> X_deg;
	float Y = DataStruct -> Y_deg;

	DataStruct -> X_deg =  0.98 * (X + DataStruct -> Gx_deg) + (0.02 * (DataStruct -> Ax_deg));
	DataStruct -> Y_deg =  0.98 * (Y + DataStruct -> Gy_deg) + (0.02 * (DataStruct -> Ay_deg));
	DataStruct -> Z_deg = DataStruct -> Z_deg + DataStruct -> Gz_deg;
}
