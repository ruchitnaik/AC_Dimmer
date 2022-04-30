/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdbool.h>
#include <stdio.h>
#include "retarget.h"

#include "MY_ILI9341.h"
#include "TSC2046.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TOUCH_CTRL		0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PRPTOTYPE int __io_getchar(void)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile bool f_StartPWM = false;
volatile bool f_ChangePWM = false;
static int DutyCycle = 0;				//Duty cycle default set to 100
uint8_t set_dutycycle = 0;
uint8_t backupDutyCycle = 0;

TS_TOUCH_DATA_Def myTS_Handle;
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart2);
  printf("Starting at 0%% duty cycle by default\r\n");
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  TIM2->CCR1 = DutyCycle;

#if (TOUCH_CTRL == 1)
  /** Touch and LCD code below **/
	uint8_t dutycycle = 0;
	char str[10];
	uint8_t sw = 0;

	ILI9341_Init(&hspi1, LCD_CS_GPIO_Port, LCD_CS_Pin, LCD_DC_GPIO_Port, LCD_DC_Pin, LCD_RESET_GPIO_Port, LCD_RESET_Pin);
	ILI9341_setRotation(2);
	ILI9341_Fill(COLOR_YELLOW);

	TSC2046_Begin(&hspi2, TS_CS_GPIO_Port, TS_CS_Pin);
	TSC2046_Calibrate();
	ILI9341_Fill(COLOR_NAVY);

	ILI9341_printText("INTENSITY CONTROL", 10, 10, COLOR_RED, COLOR_NAVY, 3);

	ILI9341_Fill_Rect(40, 60, 110 ,130, COLOR_WHITE );	//up button white
	ILI9341_Fill_Rect(45, 65, 105 ,125, COLOR_BLACK );	//up button black

	ILI9341_Fill_Rect(200, 60, 270 ,130, COLOR_WHITE );	//down button white
	ILI9341_Fill_Rect(205, 65, 265 ,125, COLOR_BLACK );	//down button black

	//ILI9341_drawTriangle(75, 80, 65, 100, 85, 100, COLOR_RED);
	ILI9341_fillTriangle(75, 80, 65, 100, 85, 100, COLOR_RED); //up button triangle
	ILI9341_fillTriangle(225, 80, 245, 80, 235, 100, COLOR_RED); //down button triangle

	ILI9341_Fill_Rect(10, 180, 110, 230, COLOR_BLACK); //on-off button
	ILI9341_Fill_Rect(180, 180, 310, 230, COLOR_BLACK); //set duty cycle button

	ILI9341_printText("UP", 65, 140, COLOR_RED, COLOR_NAVY, 2);
	ILI9341_printText("DOWN", 215, 140, COLOR_RED, COLOR_NAVY, 2);

	ILI9341_printText("DUTY", 130, 70, COLOR_RED, COLOR_NAVY, 2);
	ILI9341_printText("CYCLE", 125, 95, COLOR_RED, COLOR_NAVY, 2);

	ILI9341_printText("ON/OFF", 25, 200, COLOR_WHITE, COLOR_BLACK, 2);
	//ILI9341_printText("OFF", 25, 200, COLOR_WHITE, COLOR_BLACK, 2);
	ILI9341_printText("SET", 230, 185, COLOR_WHITE, COLOR_BLACK, 2);
	ILI9341_printText("DUTY CYCLE", 185, 205, COLOR_WHITE, COLOR_BLACK, 2);

	sprintf(str, " %d ", dutycycle);
	ILI9341_printText(str, 120, 140, COLOR_RED, COLOR_NAVY, 4);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /** LCD and Touch control code STARTS **/
#if (TOUCH_CTRL == 1)
    myTS_Handle = TSC2046_GetTouchData();
	  if(myTS_Handle.isPressed)
	  {
		  //UP button
		  if(myTS_Handle.X >=40 && myTS_Handle.X<=110 && myTS_Handle.Y>=60 && myTS_Handle.Y<=130)
		  {
			  if((sw % 2)!=0)
			  {
				  if(dutycycle == 100)
				  {
					  continue;
				  }
				  else
				  {
					  dutycycle += 10;
					  sprintf(str, " %d ", dutycycle);
					  if(dutycycle == 100)
					  {
						  ILI9341_printText(str, 95, 140, COLOR_RED, COLOR_NAVY, 4);
						  ILI9341_printText("DOWN", 215, 140, COLOR_RED, COLOR_NAVY, 2);
					  }
					  else if(dutycycle > 0 || dutycycle < 100 )
					  {
						  ILI9341_printText(str, 110, 140, COLOR_RED, COLOR_NAVY, 4);
						  ILI9341_printText("DOWN", 215, 140, COLOR_RED, COLOR_NAVY, 2);
					  }
					  else
					  {
						  ILI9341_printText(str, 120, 140, COLOR_RED, COLOR_NAVY, 4);
						  ILI9341_printText("DOWN", 215, 140, COLOR_RED, COLOR_NAVY, 2);
					  }
				  }
			  }
		  }
		  //DOWN button
		  if(myTS_Handle.X >=200 && myTS_Handle.X<=270 && myTS_Handle.Y>=60 && myTS_Handle.Y<=130)
		  {
			  if(dutycycle == 0)
			  {
				  continue;
			  }
			  else
			  {
				  dutycycle -= 10;
				  sprintf(str, " %d ", dutycycle);
				  if(dutycycle == 0)
				  {
					  ILI9341_printText(str, 120, 140, COLOR_RED, COLOR_NAVY, 4);
					  ILI9341_printText("DOWN", 215, 140, COLOR_RED, COLOR_NAVY, 2);
				  }
				  else if(dutycycle > 0 || dutycycle < 100 )
				  {
					  ILI9341_printText(str, 110, 140, COLOR_RED, COLOR_NAVY, 4);
					  ILI9341_printText("DOWN", 215, 140, COLOR_RED, COLOR_NAVY, 2);
				  }
				  else
				  {
					  ILI9341_printText(str, 95, 140, COLOR_RED, COLOR_NAVY, 4);
					  ILI9341_printText("DOWN", 215, 140, COLOR_RED, COLOR_NAVY, 2);
				  }
			  }
		  }

		  //ON/OFF button
		  if(myTS_Handle.X >=10 && myTS_Handle.X<=110 && myTS_Handle.Y>=180 && myTS_Handle.Y<=230)
		  {
			  sw++;

			  if((sw % 2) != 0)
			  {
				  set_dutycycle = backupDutyCycle;
				  dutycycle = set_dutycycle;
				  sprintf(str, " %d ", dutycycle);
				  ILI9341_printText(str, 110, 140, COLOR_RED, COLOR_NAVY, 4);
				  ILI9341_printText("DOWN", 215, 140, COLOR_RED, COLOR_NAVY, 2);
				  ILI9341_Fill_Rect(10, 180, 110, 230, COLOR_WHITE);
				  ILI9341_printText("ON", 45, 200, COLOR_BLACK, COLOR_WHITE, 2);
			  }
			  else
			  {
				  backupDutyCycle = set_dutycycle;
				  dutycycle = 0;
				  set_dutycycle = 0;
				  sprintf(str, " %d ", dutycycle);
				  ILI9341_printText(str, 120, 140, COLOR_RED, COLOR_NAVY, 4);
				  ILI9341_printText("DOWN", 215, 140, COLOR_RED, COLOR_NAVY, 2);
				  ILI9341_Fill_Rect(10, 180, 110, 230, COLOR_BLACK);
				  ILI9341_printText("OFF", 40, 200, COLOR_WHITE, COLOR_BLACK, 2);
			  }
		  }

		  //set dutycycle button
		  if(myTS_Handle.X >=180 && myTS_Handle.X<=310 && myTS_Handle.Y>=180 && myTS_Handle.Y<=230)
		  {
			  set_dutycycle = dutycycle; //use set_dutycycle further
			  printf("Duty Cycle: %d\r\n", set_dutycycle);
		  }
	  }

	/** TRIAC Control code STARTS **/
#else
	if(f_ChangePWM){
		//Handling PWM change on USR button press
		f_ChangePWM = false;
		printf("Enter New Duty Cycle\r\n");
		while(1){
			scanf("%d", &DutyCycle);
			if((DutyCycle >= 0) && (DutyCycle <= 100)){
				set_dutycycle = DutyCycle;
				printf("%d\r\n", DutyCycle);
				break;
			}
			else{
				printf("Enter Duty Cycle value between 0 to 100. \r\n Try Again\r\n");
			}
		}
	}
#endif
	if(f_StartPWM){										//ZCD interrupt handling
		f_StartPWM = false;
		//Code to start PWM based on Received input value
//    	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//    	TIM2->CCR1 = DutyCycle;
		HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
	}
	/** TRIAC Control code ENDS **/
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 14000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TS_CS_GPIO_Port, TS_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LCD_CS_Pin|LCD_RESET_Pin|LCD_DC_Pin|LD4_Pin
                          |LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TS_CS_Pin */
  GPIO_InitStruct.Pin = TS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TS_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_RESET_Pin LCD_DC_Pin LD4_Pin
                           LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_RESET_Pin|LCD_DC_Pin|LD4_Pin
                          |LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_MCK_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_MCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ZCR_INT_Pin */
  GPIO_InitStruct.Pin = ZCR_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ZCR_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
#if TOUCH_CTRL == 0
	if(GPIO_Pin == B1_Pin){
		f_ChangePWM = true;
	}
#endif
	if(GPIO_Pin == ZCR_INT_Pin){
		f_StartPWM = true;
		TIM2->CCR1 = set_dutycycle;
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
	}
}
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
