/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mytype.h"
#include "pid.h"
#include "ps2.h"
#include "fonts.h"
#include "ssd1306.h"
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
  
	if(SD_MPU6050_Init(&hi2c2,&MPU6050,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_2000s) !=SD_MPU6050_Result_Ok)
	{
		Error_Handler();
	}
	HAL_Delay(500);

	if (ssd1306_Init(&hi2c1) != 0) 
	{
		Error_Handler();
	}
	HAL_Delay(1000);

	ssd1306_Fill(Black);
//	OLED_Clear(&hi2c1);
	ssd1306_UpdateScreen(&hi2c1);

	HAL_Delay(1000);
  
    // Write data to local screenbuffer
	ssd1306_WriteString(17,0,"Sophia",Font_11x18,White);

	ssd1306_WriteString(29,20,"Li_Zii",Font_7x10,White);
  
  // Draw rectangle on screen
	for (uint8_t i=0; i<28; i++) {
		for (uint8_t j=0; j<64; j++) {
			ssd1306_DrawPixel(100+i,0+j,White);
		}
	}

  // Copy all data from local screenbuffer to the screen
	ssd1306_UpdateScreen(&hi2c1);
	HAL_Delay(2000);

	
	ssd1306_Fill(Black);
	
	ssd1306_WriteString(10,25,"Angle_x:",Font_7x10,White);
	ssd1306_WriteString(10,SSD1306_HEIGHT-22,"Angle_y:",Font_7x10,White);
	
	ssd1306_UpdateScreen(&hi2c1);//清屏，准备进入主界面
	
	ssd1306_WriteString(31,0,"Sophia",Font_11x18,White);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  MPU6050_Read_All(&hi2c2,&MPU6050);
	  
	  double g_x = MPU6050.Gx;
	  double g_y = MPU6050.Gy;
	  double g_z = MPU6050.Gz;
	  
	  double a_x = MPU6050.Ax;
	  double a_y = MPU6050.Ay;
	  double a_z = MPU6050.Az;
	
	  double angle_x = MPU6050.KalmanAngleX;
	  double angle_y = MPU6050.KalmanAngleY;
	  
	  ssd1306_Clear_Area(68,25,100,10,Black);
	  ssd1306_Clear_Area(68,SSD1306_HEIGHT-22,100,10,Black);
	  ssd1306_ShowPiontNum(68,25,angle_x,4,Font_7x10,White);
	  ssd1306_ShowPiontNum(68,SSD1306_HEIGHT-22,angle_y,4,Font_7x10,White);
	  
	  printf("g_x:%f	g_y:%f	g_z:%f\r\n",g_x,g_y,g_z);
	  printf("a_x:%f	a_y:%f	a_z:%f\r\n",a_x,a_y,a_z);
	  printf("angle_x:%f	angle_y:%f\r\n",angle_x,angle_y);
	  printf("------------------------\r\n");
	  
	  ssd1306_UpdateScreen(&hi2c1);
	  
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
