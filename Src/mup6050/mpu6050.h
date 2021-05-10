/*
 * sd_hal_mpu6050.h
 *
 *  Created on: Feb 19, 2016
 *      Author: Sina Darvishi
 */

#ifndef DRIVERS_MYLIB_MPU6050_H_
#define DRIVERS_MYLIB_MPU6050_H_

/*
 C++ detection
#ifdef __cplusplus
extern "C" {
#endif
*/

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"

/**
 * @defgroup SD_MPU6050_Macros
 * @brief    Library defines
 * @{
 */

/* Default I2C used */
//#ifndef MPU6050_I2C
//#define	MPU6050_I2C                    I2C1              /*!< Default I2C */
//#define MPU6050_I2C_PINSPACK           SD_I2C_PinsPack_1 /*!< Default I2C pinspack. Check @ref SD_I2C for more information */
//#endif

/* Default I2C clock */
#ifndef MPU6050_I2C_CLOCK
#define MPU6050_I2C_CLOCK              400000            /*!< Default I2C clock speed */
#endif

/* Default I2C address */
#define MPU6050_I2C_ADDR			0xD0

/* Who I am register value */
#define MPU6050_I_AM				0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SELF_TESTX_REG		0x0D	//自检寄存器X
#define MPU6050_SELF_TESTY_REG		0x0E	//自检寄存器Y
#define MPU6050_SELF_TESTZ_REG		0x0F	//自检寄存器Z
#define MPU6050_SELF_TESTA_REG		0x10	//自检寄存器A
#define MPU6050_SMPLRT_DIV			0x19	//采样频率分频器
#define MPU6050_CONFIG				0x1A	//配置寄存器
#define MPU6050_GYRO_CONFIG			0x1B	//陀螺仪配置寄存器
#define MPU6050_ACCEL_CONFIG		0x1C	//加速度计配置寄存器
#define MPU6050_MOTION_THRESH		0x1F	//运动检测阀值设置寄存器
#define MPU6050_FIFO_EN_REG			0x23	//FIFO使能寄存器
#define MPU6050_I2CMST_CTRL_REG		0x24	//IIC主机控制寄存器
#define MPU6050_I2CSLV0_ADDR_REG	0x25	//IIC从机0器件地址寄存器
#define MPU6050_I2CSLV0_REG			0x26	//IIC从机0数据地址寄存器
#define MPU6050_I2CSLV0_CTRL_REG	0x27	//IIC从机0控制寄存器
#define MPU6050_I2CSLV1_ADDR_REG	0x28	//IIC从机1器件地址寄存器
#define MPU6050_I2CSLV1_REG			0x29	//IIC从机1数据地址寄存器
#define MPU6050_I2CSLV1_CTRL_REG	0x2A	//IIC从机1控制寄存器
#define MPU6050_I2CSLV2_ADDR_REG	0x2B	//IIC从机2器件地址寄存器
#define MPU6050_I2CSLV2_REG			0x2C	//IIC从机2数据地址寄存器
#define MPU6050_I2CSLV2_CTRL_REG	0x2D	//IIC从机2控制寄存器
#define MPU6050_I2CSLV3_ADDR_REG	0x2E	//IIC从机3器件地址寄存器
#define MPU6050_I2CSLV3_REG			0x2F	//IIC从机3数据地址寄存器
#define MPU6050_I2CSLV3_CTRL_REG	0x30	//IIC从机3控制寄存器
#define MPU6050_I2CSLV4_ADDR_REG	0x31	//IIC从机4器件地址寄存器
#define MPU6050_I2CSLV4_REG			0x32	//IIC从机4数据地址寄存器
#define MPU6050_I2CSLV4_DO_REG		0x33	//IIC从机4写数据寄存器
#define MPU6050_I2CSLV4_CTRL_REG	0x34	//IIC从机4控制寄存器
#define MPU6050_I2CSLV4_DI_REG		0x35	//IIC从机4读数据寄存器

#define MPU6050_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU6050_INT_PIN_CFG			0x37	//中断/旁路设置寄存器
#define MPU6050_INT_ENABLE			0x38	//中断使能寄存器
#define MPU6050_INT_STATUS			0x3A	//中断状态寄存器

#define MPU6050_ACCEL_XOUT_H		0x3B	//加速度值,X轴高8位寄存器
#define MPU6050_ACCEL_XOUT_L		0x3C	//加速度值,X轴低8位寄存器
#define MPU6050_ACCEL_YOUT_H		0x3D	//加速度值,Y轴高8位寄存器
#define MPU6050_ACCEL_YOUT_L		0x3E	//加速度值,Y轴低8位寄存器
#define MPU6050_ACCEL_ZOUT_H		0x3F	//加速度值,Z轴高8位寄存器
#define MPU6050_ACCEL_ZOUT_L		0x40	//加速度值,Z轴低8位寄存器

#define MPU6050_TEMP_OUT_H			0x41	//温度值高八位寄存器
#define MPU6050_TEMP_OUT_L			0x42	//温度值低8位寄存器

#define MPU6050_GYRO_XOUT_H			0x43	//陀螺仪值,X轴高8位寄存器
#define MPU6050_GYRO_XOUT_L			0x44	//陀螺仪值,X轴低8位寄存器
#define MPU6050_GYRO_YOUT_H			0x45	//陀螺仪值,Y轴高8位寄存器
#define MPU6050_GYRO_YOUT_L			0x46	//陀螺仪值,Y轴低8位寄存器
#define MPU6050_GYRO_ZOUT_H			0x47	//陀螺仪值,Z轴高8位寄存器
#define MPU6050_GYRO_ZOUT_L			0x48	//陀螺仪值,Z轴低8位寄存器

#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_I2CSLV0_DO_REG		0x63	//IIC从机0数据寄存器
#define MPU6050_I2CSLV1_DO_REG		0x64	//IIC从机1数据寄存器
#define MPU6050_I2CSLV2_DO_REG		0x65	//IIC从机2数据寄存器
#define MPU6050_I2CSLV3_DO_REG		0x66	//IIC从机3数据寄存器

#define MPU6050_I2CMST_DELAY_REG	0x67	//IIC主机延时管理寄存器
#define MPU6050_SIGNAL_PATH_RESET	0x68	//信号通道复位寄存器
#define MPU6050_MOT_DETECT_CTRL		0x69	//运动检测控制寄存器
#define MPU6050_USER_CTRL			0x6A	//用户控制寄存器
#define MPU6050_PWR_MGMT_1			0x6B	//电源管理寄存器1
#define MPU6050_PWR_MGMT_2			0x6C	//电源管理寄存器2 
#define MPU6050_FIFO_COUNTH			0x72	//FIFO计数寄存器高八位
#define MPU6050_FIFO_COUNTL			0x73	//FIFO计数寄存器低八位
#define MPU6050_FIFO_R_W			0x74	//FIFO读写寄存器
#define MPU6050_WHO_AM_I			0x75	//器件ID寄存器

/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g/s */
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)

/**
 * @brief  Data rates predefined constants
 * @{
 */
#define SD_MPU6050_DataRate_8KHz       0   /*!< Sample rate set to 8 kHz */
#define SD_MPU6050_DataRate_4KHz       1   /*!< Sample rate set to 4 kHz */
#define SD_MPU6050_DataRate_2KHz       3   /*!< Sample rate set to 2 kHz */
#define SD_MPU6050_DataRate_1KHz       7   /*!< Sample rate set to 1 kHz */
#define SD_MPU6050_DataRate_500Hz      15  /*!< Sample rate set to 500 Hz */
#define SD_MPU6050_DataRate_250Hz      31  /*!< Sample rate set to 250 Hz */
#define SD_MPU6050_DataRate_125Hz      63  /*!< Sample rate set to 125 Hz */
#define SD_MPU6050_DataRate_100Hz      79  /*!< Sample rate set to 100 Hz */


// MPU6050 structure
typedef struct {
	
	uint8_t Address;         /*!< I2C address of device. */
	float Gyro_Mult;         /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
	float Acce_Mult;         /*!< Accelerometer corrector from raw data to "g". Only for private use */

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
} MPU6050_t;


// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;



void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
/**
 * @}
 */

/**
 * @}
 */

/**
 * @defgroup SD_MPU6050_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  MPU6050 can have 2 different slave addresses, depends on it's input AD0 pin
 *         This feature allows you to use 2 different sensors with this library at the same time
 */
typedef enum  {
	SD_MPU6050_Device_0 = 0x00, /*!< AD0 pin is set to low */
	SD_MPU6050_Device_1 = 0x02  /*!< AD0 pin is set to high */
} SD_MPU6050_Device;

/**
 * @brief  MPU6050 result enumeration
 */
typedef enum  {
	SD_MPU6050_Result_Ok = 0x00,          /*!< Everything OK */
	SD_MPU6050_Result_Error,              /*!< Unknown error */
	SD_MPU6050_Result_DeviceNotConnected, /*!< There is no device with valid slave address */
	SD_MPU6050_Result_DeviceInvalid       /*!< Connected device with address is not MPU6050 */
} MPU6050_Result;

/**
 * @brief  Parameters for accelerometer range
 */
typedef enum  {
	SD_MPU6050_Accelerometer_2G = 0x00, /*!< Range is +- 2G */
	SD_MPU6050_Accelerometer_4G = 0x01, /*!< Range is +- 4G */
	SD_MPU6050_Accelerometer_8G = 0x02, /*!< Range is +- 8G */
	SD_MPU6050_Accelerometer_16G = 0x03 /*!< Range is +- 16G */
} SD_MPU6050_Accelerometer;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum {
	SD_MPU6050_Gyroscope_250s = 0x00,  /*!< Range is +- 250 degrees/s */
	SD_MPU6050_Gyroscope_500s = 0x01,  /*!< Range is +- 500 degrees/s */
	SD_MPU6050_Gyroscope_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
	SD_MPU6050_Gyroscope_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} SD_MPU6050_Gyroscope;

/**
 * @brief  Main MPU6050 structure
 */
//typedef struct  {
//	/* Private */
//	uint8_t Address;         /*!< I2C address of device. */
//	float Gyro_Mult;         /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
//	float Acce_Mult;         /*!< Accelerometer corrector from raw data to "g". Only for private use */
//	/* Public */
//	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
//	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
//	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
//	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
//	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
//	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
//	float   Temperature;       /*!< Temperature in degrees */
//	//I2C_HandleTypeDef* I2Cx;
//} SD_MPU6050;

/**
 * @brief  Interrupts union and structure
 */
typedef union {
	struct {
		uint8_t DataReady:1;       /*!< Data ready interrupt */
		uint8_t reserved2:2;       /*!< Reserved bits */
		uint8_t Master:1;          /*!< Master interrupt. Not enabled with library */
		uint8_t FifoOverflow:1;    /*!< FIFO overflow interrupt. Not enabled with library */
		uint8_t reserved1:1;       /*!< Reserved bit */
		uint8_t MotionDetection:1; /*!< Motion detected interrupt */
		uint8_t reserved0:1;       /*!< Reserved bit */
	} F;
	uint8_t Status;
} SD_MPU6050_Interrupt;


MPU6050_Result MPU6050_Write_Byte(I2C_HandleTypeDef* I2Cx,uint8_t registers,uint8_t data);	//寄存器写入函数


/**
 * @}
 */

/**
 * @defgroup SD_MPU6050_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes MPU6050 and I2C peripheral
 * @param  *DataStruct: Pointer to empty @ref SD_MPU6050_t structure
 * @param  DeviceNumber: MPU6050 has one pin, AD0 which can be used to set address of device.
 *          This feature allows you to use 2 different sensors on the same board with same library.
 *          If you set AD0 pin to low, then this parameter should be SD_MPU6050_Device_0,
 *          but if AD0 pin is high, then you should use SD_MPU6050_Device_1
 *
 *          Parameter can be a value of @ref SD_MPU6050_Device_t enumeration
 * @param  AccelerometerSensitivity: Set accelerometer sensitivity. This parameter can be a value of @ref SD_MPU6050_Accelerometer_t enumeration
 * @param  GyroscopeSensitivity: Set gyroscope sensitivity. This parameter can be a value of @ref SD_MPU6050_Gyroscope_t enumeration
 * @retval Initialization status:
 *            - SD_MPU6050_Result_t: Everything OK
 *            - Other member: in other cases
 */
MPU6050_Result MPU6050_Init(I2C_HandleTypeDef* I2Cx,MPU6050_t* DataStruct, SD_MPU6050_Device DeviceNumber, SD_MPU6050_Accelerometer AccelerometerSensitivity, SD_MPU6050_Gyroscope GyroscopeSensitivity);

/**
 * @brief  Sets gyroscope sensitivity
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @param  GyroscopeSensitivity: Gyro sensitivity value. This parameter can be a value of @ref SD_MPU6050_Gyroscope_t enumeration
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
MPU6050_Result SD_MPU6050_SetGyroscope(I2C_HandleTypeDef* I2Cx,MPU6050_t* DataStruct, SD_MPU6050_Gyroscope GyroscopeSensitivity);

/**
 * @brief  Sets accelerometer sensitivity
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @param  AccelerometerSensitivity: Gyro sensitivity value. This parameter can be a value of @ref SD_MPU6050_Accelerometer_t enumeration
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
MPU6050_Result SD_MPU6050_SetAccelerometer(I2C_HandleTypeDef* I2Cx,MPU6050_t* DataStruct, SD_MPU6050_Accelerometer AccelerometerSensitivity);

/**
 * @brief  Sets output data rate
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @param  rate: Data rate value. An 8-bit value for prescaler value
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
MPU6050_Result SD_MPU6050_SetDataRate(I2C_HandleTypeDef* I2Cx,MPU6050_t* DataStruct, uint8_t rate);


/**
 * @brief  Enables interrupts
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
MPU6050_Result SD_MPU6050_EnableInterrupts(I2C_HandleTypeDef* I2Cx,MPU6050_t* DataStruct);

/**
 * @brief  Disables interrupts
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
MPU6050_Result SD_MPU6050_DisableInterrupts(I2C_HandleTypeDef* I2Cx,MPU6050_t* DataStruct);

/**
 * @brief  Reads and clears interrupts
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @param  *InterruptsStruct: Pointer to @ref SD_MPU6050_Interrupt_t structure to store status in
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
MPU6050_Result SD_MPU6050_ReadInterrupts(I2C_HandleTypeDef* I2Cx,MPU6050_t* DataStruct, SD_MPU6050_Interrupt* InterruptsStruct);

/**
 * @brief  Reads accelerometer data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure to store data to
 * @retval Member of @ref SD_MPU6050_Result_t:
 *            - SD_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
//MPU6050_Result SD_MPU6050_ReadAccelerometer(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct);

/**
 * @brief  Reads gyroscope data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure to store data to
 * @retval Member of @ref SD_MPU6050_Result_t:
 *            - SD_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
//MPU6050_Result SD_MPU6050_ReadGyroscope(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct);

/**
 * @brief  Reads temperature data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure to store data to
 * @retval Member of @ref SD_MPU6050_Result_t:
 *            - SD_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
//MPU6050_Result SD_MPU6050_ReadTemperature(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct);

/**
 * @brief  Reads accelerometer, gyroscope and temperature data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure to store data to
 * @retval Member of @ref SD_MPU6050_Result_t:
 *            - SD_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
//MPU6050_Result SD_MPU6050_ReadAll(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */


#endif /* DRIVERS_MYLIB_SD_HAL_MPU6050_H_ */
