/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP280_Adress (0x76<<1)
#define calibreStarting_Adresss (0x88)
#define BMP280_REG_TEMP_XLSB   0xFC /* bits: 7-4 */
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_PRESS_XLSB  0xF9 /* bits: 7-4 */
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CONFIG      0xF5 /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
#define BMP280_REG_CTRL        0xF4 /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
#define BMP280_REG_STATUS      0xF3 /* bits: 3 measuring; 0 im_update */
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_ID          0xD0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned short dig_T1,dig_P1;
signed short dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
signed long temperature_raw, pressure_raw;
float temperature, pressure, altitude;
uint8_t Data;
uint8_t check;

void BMP280_Init (void)
{
	BMP280_Calibrating();

	HAL_I2C_Mem_Read(&hi2c1, BMP280_Adress, BMP280_REG_ID, 1, &check, 1, 1000);		//Read sensor id and control it.
	if(check==88)
	{
		Data= 0x00;
		HAL_I2C_Mem_Write(&hi2c1, BMP280_Adress, BMP280_REG_RESET, 1, &Data, 1, 1000);

		Data= 0x00;
		HAL_I2C_Mem_Write(&hi2c1, BMP280_Adress, BMP280_REG_STATUS, 1, &Data, 1, 1000);

		Data= 0x00;
		HAL_I2C_Mem_Write(&hi2c1, BMP280_Adress, BMP280_REG_CTRL, 1, &Data, 1, 1000);

		Data= 0x00;
		HAL_I2C_Mem_Write(&hi2c1, BMP280_Adress, BMP280_REG_CONFIG, 1, &Data, 1, 1000);

		Data= 0x80;
		HAL_I2C_Mem_Write(&hi2c1, BMP280_Adress, BMP280_REG_PRESS_MSB, 1, &Data, 1, 1000);

		Data= 0x00;
		HAL_I2C_Mem_Write(&hi2c1, BMP280_Adress, BMP280_REG_PRESS_LSB, 1, &Data, 1, 1000);

		Data= 0x00;
		HAL_I2C_Mem_Write(&hi2c1, BMP280_Adress, BMP280_REG_PRESS_XLSB, 1, &Data, 1, 1000);

		Data= 0x80;
		HAL_I2C_Mem_Write(&hi2c1, BMP280_Adress, BMP280_REG_TEMP_MSB, 1, &Data, 1, 1000);

		Data= 0x00;
		HAL_I2C_Mem_Write(&hi2c1, BMP280_Adress, BMP280_REG_TEMP_LSB, 1, &Data, 1, 1000);

		Data= 0x00;
		HAL_I2C_Mem_Write(&hi2c1, BMP280_Adress, BMP280_REG_TEMP_XLSB, 1, &Data, 1, 1000);
	}

}

void BMP280_Calibrating (void)
{
	uint8_t Rec_Data[24];
	//Read 24 bytes of data from bmp280
	HAL_I2C_Mem_Read(&hi2c1, BMP280_Adress, calibreStarting_Adresss, 1, Rec_Data, 24, 1000);

	dig_T1=(Rec_Data[0])+(Rec_Data[1]<<8);
	dig_T2=(Rec_Data[2])+(Rec_Data[3]<<8);
	dig_T3=(Rec_Data[4])+(Rec_Data[5]<<8);
	dig_P1=(Rec_Data[6])+(Rec_Data[7]<<8);
	dig_P2=(Rec_Data[8])+(Rec_Data[9]<<8);
	dig_P3=(Rec_Data[10])+(Rec_Data[11]<<8);
	dig_P4=(Rec_Data[12])+(Rec_Data[13]<<8);
	dig_P5=(Rec_Data[14])+(Rec_Data[15]<<8);
	dig_P6=(Rec_Data[16])+(Rec_Data[17]<<8);
	dig_P7=(Rec_Data[18])+(Rec_Data[19]<<8);
	dig_P8=(Rec_Data[20])+(Rec_Data[21]<<8);
	dig_P9=(Rec_Data[22])+(Rec_Data[23]<<8);
}

void BMP280_Calculating(void)
{
	uint8_t Rec_Data[6];
	uint8_t buffer[3];

	HAL_I2C_Mem_Read(&hi2c1, BMP280_Adress, BMP280_REG_PRESS_MSB, 1, Rec_Data, 6, 1000);
	buffer[2]=Rec_Data[3];
	buffer[1]=Rec_Data[4];
	buffer[0]=Rec_Data[5];

	temperature_raw= ((buffer[2]<< 12) | (buffer[1] << 4) | (buffer[0] >>4));

	buffer[2]=Rec_Data[0];
	buffer[1]=Rec_Data[1];
	buffer[0]=Rec_Data[2];

	pressure_raw= ((buffer[2]<< 12) | (buffer[1] << 4) | (buffer[0] >>4));

	double var1, var2;

	var1=(((double)temperature_raw)/16384.0-((double)dig_T1)/1024.0)*((double)dig_T2);
	var2=((((double)temperature_raw)/131072.0-((double)dig_T1)/8192.0)*(((double)temperature_raw)/131072.0-((double)dig_T1)/8192.0))*((double)dig_T3);
	double t_fine = (int32_t)(var1+var2);
volatile  float T = (var1+var2)/5120.0;

	var1=((double)t_fine/2.0)-64000.0;
	var2=var1*var1*((double)dig_P6)/32768.0;
	var2=var2+var1*((double)dig_P5)*2.0;
	var2=(var2/4.0)+(((double)dig_P4)*65536.0);
	var1=(((double)dig_P3)*var1*var1/524288.0+((double)dig_P2)*var1)/524288.0;
	var1=(1.0+var1/32768.0)*((double)dig_P1);
volatile	double p=1048576.0-(double)pressure_raw;

	p=(p-(var2/4096.0))*6250.0/var1;
	var1=((double)dig_P9)*p*p/2147483648.0;
	var2=p*((double)dig_P8)/32768.0;
	p=p+(var1+var2+((double)dig_P7))/16.0;

	temperature=T;
	pressure=p;
	altitude=44330.0f*(1-powf(pressure/101325.0f,1.0f/5.255f)); //altitude=((powf(101325.0/pressure, 1/5.257f)-1)*(temperature+273.15f))/0.0065f;
}
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  BMP280_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  BMP280_Calculating();
	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
	__I2C1_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x30A0A7FB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
