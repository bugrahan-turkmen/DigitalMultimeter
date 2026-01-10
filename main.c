/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_resol (3.3f / 4095.0f)
// ADC resolution for a 12-bit ADC (voltage per LSB)

#define RREF_100   135.0
#define RREF_1K    1035.0
#define RREF_10K   10035.0
#define RREF_100K  100035.0


//LCD icin
#define LCD_ADDR (0x27 << 1)

#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04
#define LCD_RS        0x01

extern I2C_HandleTypeDef hi2c1;
// Calibrated reference resistor values (measured, not nominal)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

uint16_t adc_buf[100];
double Calculated_mean[4];
uint16_t Vref_buf[100];

double Calculated_ohm[4];
double Calculated_Vref[4];
uint8_t stop_flag=0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//    ADCBUFFER[0] = hedef
//		adcbuffer[1] = referans

uint16_t adcbuffer[2];
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*hadc)
{
	if(hadc->Instance==ADC1){
	//	HAL_ADC_Start_DMA(&hadc1,adcbuffer,2);

	}
}




void LCD_Send(uint8_t data, uint8_t rs)
{
    uint8_t high = (data & 0xF0) | LCD_BACKLIGHT | rs;
    uint8_t low  = ((data << 4) & 0xF0) | LCD_BACKLIGHT | rs;

    uint8_t buf[4];
    buf[0] = high | LCD_ENABLE;
    buf[1] = high;
    buf[2] = low | LCD_ENABLE;
    buf[3] = low;

    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, buf, 4, 100);
}

void LCD_Cmd(uint8_t cmd)
{
    LCD_Send(cmd, 0);
}

void LCD_Data(uint8_t data)
{
    LCD_Send(data, LCD_RS);
}

void LCD_Init(void)
{
    HAL_Delay(50);

    LCD_Cmd(0x30);
    HAL_Delay(5);
    LCD_Cmd(0x30);
    HAL_Delay(1);
    LCD_Cmd(0x30);
    LCD_Cmd(0x20);   // 4 bit

    LCD_Cmd(0x28);   // 2 satır
    LCD_Cmd(0x08);   // display off
    LCD_Cmd(0x01);   // clear
    HAL_Delay(2);
    LCD_Cmd(0x06);   // cursor move
    LCD_Cmd(0x0C);   // display on
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? 0x80 : 0xC0;
    LCD_Cmd(addr + col);
}

void LCD_Print(char *str)
{
    while (*str)
        LCD_Data(*str++);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int i;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  double sum;
  double mean;
  double Vref_mean;
  double Vref_sum;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, SET);
  LCD_Init();

  LCD_SetCursor(0,0);
  LCD_Print("Direnc Degeri");
  HAL_ADC_Start_DMA(&hadc1,adcbuffer,2);
 // LCD_SetCursor(1,0);
 // LCD_Print("800 ohm");
 // uint8_t flag3=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  for (int j = 0 ; j < 4 ; j++ ){
		    // Each iteration selects a different reference resistor range
		    // using GPIO-controlled switches
		  sum = 0;
		  mean = 0;
		  Vref_mean = 0;
		  Vref_sum = 0;
		  if(j==0){
			  HAL_GPIO_WritePin(GPIOA, ohm_100_Pin, GPIO_PIN_SET);
			  HAL_Delay(5);

			  for (i = 0; i < 100 ; i++){


					Vref_sum = Vref_sum + ADC_resol * adcbuffer[1];
				    HAL_Delay(1);


					adc_buf[i] = adcbuffer[0];
					sum = sum + ADC_resol * adc_buf[i];

			  }

			  // Multiple ADC samples are taken and averaged
			  // to reduce noise and improve measurement stability

			  mean = sum / i;
			  Vref_mean = Vref_sum / i;

			  Calculated_Vref[j] = Vref_mean;
			  Calculated_mean[j] = mean;

			  double ratio = mean / Vref_mean;

			  // Limit ratio to avoid division by zero and saturation effects
			  if(ratio < 0.01) ratio = 0.01;
			  if(ratio > 0.99) ratio = 0.99;

			  // Unknown resistance calculation based on voltage divider equation
			  Calculated_ohm[j] = (RREF_100 * ratio) / (1 - ratio);
			  if(mean>0.1){
				  LCD_SetCursor(1,0);
				  LCD_Print("20 ohm");
			  }
			  HAL_GPIO_WritePin(GPIOA, ohm_100_Pin, GPIO_PIN_RESET);
			  HAL_Delay(5);
		  }
		  if(j==1){
			  HAL_GPIO_WritePin(GPIOA,ohm_1k_Pin, GPIO_PIN_SET);
			  HAL_Delay(5);

			  for (i = 0; i < 100 ; i++){

					Vref_buf[i] = adcbuffer[1];
					Vref_sum = Vref_sum + ADC_resol * Vref_buf[i];


					adc_buf[i] = adcbuffer[0];
					sum = sum + ADC_resol * adc_buf[i];
			  }
			  mean = sum / i;
			  Vref_mean = Vref_sum / i;
			  Calculated_Vref[j] = Vref_mean;
			  Calculated_mean[j] = mean;

			  double ratio = mean / Vref_mean;



			  Calculated_ohm[j] = (RREF_1K * (ratio)) / (1 - (ratio));
			  if(mean>0.3){
				  LCD_SetCursor(1,0);
				  LCD_Print("200 ohm");
			  }
			  HAL_GPIO_WritePin(GPIOA,ohm_1k_Pin, GPIO_PIN_RESET);
			  HAL_Delay(5);
		  }
		  if(j==2){
			  HAL_GPIO_WritePin(GPIOA,ohm_10k_Pin, GPIO_PIN_SET);
			  HAL_Delay(5);

			  for (i = 0; i < 200 ; i++){

					Vref_buf[i] = adcbuffer[1];
					Vref_sum = Vref_sum + ADC_resol * Vref_buf[i];


					adc_buf[i] = adcbuffer[0];
					sum = sum + ADC_resol * adc_buf[i];
			  }
			  mean = sum / i;
			  Vref_mean = Vref_sum / i;
			  Calculated_Vref[j] = Vref_mean;
			  Calculated_mean[j] = mean;

			  double ratio = mean / Vref_mean;



			  Calculated_ohm[j] = (RREF_10K * (ratio)) / (1 - (ratio));
			  if(mean>0.3){
				  LCD_SetCursor(1,0);
				  LCD_Print("2k ohm");
			  }
			  HAL_GPIO_WritePin(GPIOA,ohm_10k_Pin, GPIO_PIN_RESET);
			  HAL_Delay(5);

		  }
		  if(j==3){
			  HAL_GPIO_WritePin(GPIOA,ohm_100k_Pin, GPIO_PIN_SET);
			  HAL_Delay(5);



			  for (i = 0; i < 100 ; i++){

					Vref_buf[i] = adcbuffer[1];
					Vref_sum = Vref_sum + ADC_resol * Vref_buf[i];


					adc_buf[i] = adcbuffer[0];
					sum = sum + ADC_resol * adc_buf[i];
			  }
			  mean = sum / i;
			  Vref_mean = Vref_sum / i;
			  Calculated_Vref[j] = Vref_mean;

			  Calculated_mean[j] = mean;

			  double ratio = Vref_mean / mean;

			  Calculated_ohm[j] = (RREF_100K) / ((ratio) - 1);

			  if(mean>0.3){
				  LCD_SetCursor(1,0);
				  LCD_Print("20k ohm");
			  }

			  HAL_GPIO_WritePin(GPIOA,ohm_100k_Pin, GPIO_PIN_RESET);
			  HAL_Delay(5);
			  stop_flag=1;
		  }
	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  hdma_adc1.Instance = DMA1_Channel1;
      hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
      hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
      hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
      hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
      hdma_adc1.Init.Mode = DMA_CIRCULAR;  // Dairesel mod ekleyin
      hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
      if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
          Error_Handler();
      }

      __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ohm_1k_Pin|ohm_10k_Pin|ohm_100k_Pin|ohm_100_Pin
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : ohm_1k_Pin ohm_10k_Pin ohm_100k_Pin ohm_100_Pin
                           PA9 */
  GPIO_InitStruct.Pin = ohm_1k_Pin|ohm_10k_Pin|ohm_100k_Pin|ohm_100_Pin
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
