/*
 * sd_hal_mpu6050.c
 *
 *  Created on: Feb 19, 2016
 *      Author: Sina Darvishi
 */

/**
 * |----------------------------------------------------------------------
 * | Copyright (C) Sina Darvishi,2016
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */

#include "mpu6050.h"
#include <math.h>

#define RAD_TO_DEG 57.295779513082320876798154814105

// Setup MPU6050
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

uint32_t timer;

Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};

MPU6050_Result MPU6050_Write_Byte(I2C_HandleTypeDef* I2Cx,uint8_t registers,uint8_t data)
{
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t d[2];
	
	/* Format I2C address */
	d[0] = registers;
	d[1] = data;
	
	/* Try to transmit via I2C */
	if(HAL_I2C_Master_Transmit(Handle,(uint16_t)MPU6050_I2C_ADDR , (uint8_t *)d, 2, 1000) != HAL_OK)
		{
					return SD_MPU6050_Result_Error;
		}
	/* Return OK */
	return SD_MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_Init(I2C_HandleTypeDef* I2Cx,MPU6050_t* DataStruct, SD_MPU6050_Device DeviceNumber, SD_MPU6050_Accelerometer AccelerometerSensitivity, SD_MPU6050_Gyroscope GyroscopeSensitivity)
{
	uint8_t WHO_AM_I = (uint8_t)MPU6050_WHO_AM_I;
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
//	uint8_t d[2];


	/* Format I2C address */
	DataStruct->Address = MPU6050_I2C_ADDR | (uint8_t)DeviceNumber;
	uint8_t address = DataStruct->Address;

	/* Check if device is connected */
	if(HAL_I2C_IsDeviceReady(Handle,address,2,5)!=HAL_OK)
	{
				return SD_MPU6050_Result_Error;
	}
	/* Check who am I */
	//------------------
		/* Send address */
		if(HAL_I2C_Master_Transmit(Handle, address, &WHO_AM_I, 1, 1000) != HAL_OK)
		{
			return SD_MPU6050_Result_Error;
		}

		/* Receive multiple byte */
		if(HAL_I2C_Master_Receive(Handle, address, &temp, 1, 1000) != HAL_OK)
		{
			return SD_MPU6050_Result_Error;
		}

		/* Checking */
		while(temp != MPU6050_I_AM)
		{
				/* Return error */
				return SD_MPU6050_Result_DeviceInvalid;
		}
	//------------------

	/* Wakeup MPU6050 */
	//------------------
		MPU6050_Write_Byte(Handle,MPU6050_PWR_MGMT_1,0x00);
	//------------------

	/* Set sample rate to 1kHz */
	SD_MPU6050_SetDataRate(I2Cx,DataStruct, SD_MPU6050_DataRate_1KHz);

	/* Config accelerometer */
	SD_MPU6050_SetAccelerometer(I2Cx,DataStruct, AccelerometerSensitivity);

	/* Config Gyroscope */
	SD_MPU6050_SetGyroscope(I2Cx,DataStruct, GyroscopeSensitivity);
		
	/* Close Interrupt */
	//-------------------
		MPU6050_Write_Byte(Handle,MPU6050_INT_ENABLE,0x00);
	//-------------------
	
	/* I2C main mode off */
	//---------------------
		MPU6050_Write_Byte(Handle,MPU6050_USER_CTRL,0x00);
	//---------------------
		
	/* Close FIFO */
	//--------------
		MPU6050_Write_Byte(Handle,MPU6050_FIFO_EN_REG,0x00);
	//--------------
	
	/* INT Valid Low */
	//-----------------
		MPU6050_Write_Byte(Handle,MPU6050_INT_PIN_CFG,0x80);
	//-----------------
		
	/* Set x reference */
	//-------------------
		MPU6050_Write_Byte(Handle,MPU6050_PWR_MGMT_1,0x01);
		MPU6050_Write_Byte(Handle,MPU6050_PWR_MGMT_2,0x00);
	//-------------------

	/* Return OK */
	return SD_MPU6050_Result_Ok;
}

MPU6050_Result SD_MPU6050_SetDataRate(I2C_HandleTypeDef* I2Cx,MPU6050_t* DataStruct, uint8_t rate)
{
	uint8_t d[2];
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;
	/* Format array to send */
	d[0] = MPU6050_SMPLRT_DIV;
	d[1] = rate;

	/* Set data sample rate */
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)address,(uint8_t *)d,2,1000)!=HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/

	/* Return OK */
	return SD_MPU6050_Result_Ok;
}

MPU6050_Result SD_MPU6050_SetAccelerometer(I2C_HandleTypeDef* I2Cx,MPU6050_t* DataStruct, SD_MPU6050_Accelerometer AccelerometerSensitivity)
{
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;
	uint8_t regAdd =(uint8_t )MPU6050_ACCEL_CONFIG;

	/* Config accelerometer */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,&regAdd, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/
	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,&temp, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/

	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case SD_MPU6050_Accelerometer_2G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_2;
			break;
		case SD_MPU6050_Accelerometer_4G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_4;
			break;
		case SD_MPU6050_Accelerometer_8G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8;
			break;
		case SD_MPU6050_Accelerometer_16G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_16;
			break;
		default:
			break;
		}

	/* Return OK */
	return SD_MPU6050_Result_Ok;
}

MPU6050_Result SD_MPU6050_SetGyroscope(I2C_HandleTypeDef* I2Cx,MPU6050_t* DataStruct, SD_MPU6050_Gyroscope GyroscopeSensitivity)
{
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;
	uint8_t regAdd =(uint8_t )MPU6050_GYRO_CONFIG;

	/* Config gyroscope */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,&regAdd, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/
	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,&temp, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/

	switch (GyroscopeSensitivity) {
			case SD_MPU6050_Gyroscope_250s:
				DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250;
				break;
			case SD_MPU6050_Gyroscope_500s:
				DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500;
				break;
			case SD_MPU6050_Gyroscope_1000s:
				DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000;
				break;
			case SD_MPU6050_Gyroscope_2000s:
				DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000;
				break;
			default:
				break;
		}
	/* Return OK */
	return SD_MPU6050_Result_Ok;
}

//MPU6050_Result SD_MPU6050_ReadAccelerometer(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct)
//{
//	uint8_t data[6];
//	uint8_t reg = MPU6050_ACCEL_XOUT_H;
//	I2C_HandleTypeDef* Handle = I2Cx;
//	uint8_t address = DataStruct->Address;

//	/* Read accelerometer data */
//	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

//	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 6, 1000) != HAL_OK);

//	/* Format */
//	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
//	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
//	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

//	/* Return OK */
//	return SD_MPU6050_Result_Ok;
//}
//MPU6050_Result SD_MPU6050_ReadGyroscope(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct)
//{
//	uint8_t data[6];
//	uint8_t reg = MPU6050_GYRO_XOUT_H;
//	I2C_HandleTypeDef* Handle = I2Cx;
//	uint8_t address = DataStruct->Address;

//	/* Read gyroscope data */
//	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

//	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 6, 1000) != HAL_OK);

//	/* Format */
//	DataStruct->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
//	DataStruct->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
//	DataStruct->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);

//	/* Return OK */
//	return SD_MPU6050_Result_Ok;
//}
//MPU6050_Result SD_MPU6050_ReadTemperature(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct)
//{
//	uint8_t data[2];
//	int16_t temp;
//	uint8_t reg = MPU6050_TEMP_OUT_H;
//	I2C_HandleTypeDef* Handle = I2Cx;
//	uint8_t address = DataStruct->Address;

//	/* Read temperature */
//	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

//	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 2, 1000) != HAL_OK);

//	/* Format temperature */
//	temp = (data[0] << 8 | data[1]);
//	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

//	/* Return OK */
//	return SD_MPU6050_Result_Ok;
//}
//MPU6050_Result SD_MPU6050_ReadAll(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct)
//{
//	uint8_t data[14];
//	int16_t temp;
//	uint8_t reg = MPU6050_ACCEL_XOUT_H;
//	I2C_HandleTypeDef* Handle = I2Cx;
//	uint8_t address = DataStruct->Address;

//	/* Read full raw data, 14bytes */
//	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

//	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 14, 1000) != HAL_OK);

//	/* Format accelerometer data */
//	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
//	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
//	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

//	/* Format temperature */
//	temp = (data[6] << 8 | data[7]);
//	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);

//	/* Format gyroscope data */
//	DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
//	DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
//	DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

//	/* Return OK */
//	return SD_MPU6050_Result_Ok;
//}
MPU6050_Result SD_MPU6050_EnableInterrupts(I2C_HandleTypeDef* I2Cx,MPU6050_t* DataStruct)
{
	uint8_t temp;
	uint8_t reg[2] = {MPU6050_INT_ENABLE,0x21};
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Enable interrupts for data ready and motion detect */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, reg, 2, 1000) != HAL_OK);

	uint8_t mpu_reg= MPU6050_INT_PIN_CFG;
	/* Clear IRQ flag on any read operation */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &mpu_reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 14, 1000) != HAL_OK);
	temp |= 0x10;
	reg[0] = MPU6050_INT_PIN_CFG;
	reg[1] = temp;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, reg, 2, 1000) != HAL_OK);

	/* Return OK */
	return SD_MPU6050_Result_Ok;
}
MPU6050_Result SD_MPU6050_DisableInterrupts(I2C_HandleTypeDef* I2Cx,MPU6050_t* DataStruct)
{
	uint8_t reg[2] = {MPU6050_INT_ENABLE,0x00};
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Disable interrupts */
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)address,reg,2,1000)!=HAL_OK);
	/* Return OK */
	return SD_MPU6050_Result_Ok;
}
MPU6050_Result SD_MPU6050_ReadInterrupts(I2C_HandleTypeDef* I2Cx,MPU6050_t* DataStruct, SD_MPU6050_Interrupt* InterruptsStruct)
{
	uint8_t read;

	/* Reset structure */
	InterruptsStruct->Status = 0;
	uint8_t reg = MPU6050_INT_STATUS;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &read, 14, 1000) != HAL_OK);

	/* Fill value */
	InterruptsStruct->Status = read;
	/* Return OK */
	return SD_MPU6050_Result_Ok;
}



void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}


void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (?/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, MPU6050_TEMP_OUT_H, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, Rec_Data, 14, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    double dt = (double) (HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
            DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0) {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    } else {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    } else {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
   	DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);

}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};

