
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */

#define ledLevelsFromEncoderTest
//#define allLedsTransitionTest
//#define knightRiderTest
//#define scrollColorTest
//#define allLedsWhiteTest

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

LED_PTR led_array[NUM_LEDS];
TLC5947 tlc5947;
BUTTON button;
ENCODER encoder;
DEBUG_MSG debugOutput;

uint8_t serialDebugRxChar[2]; 

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_CAN1_Init(void);

/* USER CODE BEGIN PFP */

/* Private function prototypes -----------------------------------------------*/
static void mapLedPointersToPwmOutputs(TLC5947 * tlc5947, LED_PTR led_array[], unsigned char numLeds);

extern unsigned int rgbTransitionLEDColor(COLOR color, LED_PTR lednum);
extern void rgbSetLEDColorAndLevel(COLOR color, LED_PTR lednum);
extern void rgbCalculateLedLevels(unsigned char zoneLevel, LED_PTR led_array[], unsigned int num_leds);
extern COLOR colors[MAX_COLOR_STRUCTS];

extern void tlc5947Init(TLC5947 * tlc5947, SPI_HandleTypeDef * spi, GPIO_TypeDef * latchGpioGroup, uint16_t latchGpioPin);
extern void tlc5947SetPWMOutputsFromBuffer(TLC5947 * tlc5947);

extern void buttonInit(BUTTON * button, GPIO_TypeDef * gpioGroup, uint16_t gpioPin);
extern void encoderInit(ENCODER * encoder, GPIO_TypeDef * encAGpioGroup, GPIO_TypeDef * encBGpioGroup, uint16_t encAGpioPin, uint16_t encBGpioPin);

extern void serialDebugRead(UART_HandleTypeDef * uart);

extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

extern int strncmp(const char *s1, const char *s2, size_t n);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_SPI2_Init();
	MX_USART1_UART_Init();
	MX_USART6_UART_Init();
	MX_CAN1_Init();
	
	/* USER CODE BEGIN 2 */
	
	CAN_FilterTypeDef  sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}
	
	
	tlc5947Init(&tlc5947, &hspi2, GPIOB, GPIO_PIN_14);
	buttonInit(&button, GPIOI, GPIO_PIN_3);
	encoderInit(&encoder, GPIOG, GPIOG, GPIO_PIN_6, GPIO_PIN_7);
	
	HAL_StatusTypeDef status;
	
	mapLedPointersToPwmOutputs(&tlc5947, led_array, NUM_LEDS);
	
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	
	while (1)
	{

	#ifdef allLedsWhiteTest
		int i;
		for (i = 0; i < PWM_BUFLEN; i++)
		{
			tlc5947.pwmBuffer.buf[i] = 0xff;
		}
		tlc5947SetPWMOutputsFromBuffer(&tlc5947);
		HAL_Delay(1000);
	  
		for (i = 0; i < PWM_BUFLEN; i++)
		{
			tlc5947.pwmBuffer.buf[i] = 0x00;
		}
		tlc5947SetPWMOutputsFromBuffer(&tlc5947);
		HAL_Delay(1000);
		#endif
	
		#ifdef scrollColorTest
		static int color = 0;
		static int led = 0;
		for (int i = 0; i < PWM_BUFLEN; i++)
		{
			tlc5947.pwmBuffer.buf[i] = 0;    //comment out for different effect
		}						
	
		led_array[led].level = MAX_LED_LEVEL;
		rgbSetLEDColorAndLevel(colors[color], led_array[led]);
		led++;
		if (led >= NUM_LEDS)
		{
			led = 0;
			color++;
			if (color >= MAX_COLOR_STRUCTS) color = 0;
		}
		tlc5947SetPWMOutputsFromBuffer(&tlc5947);
		HAL_Delay(125);
		#endif
	  
		#ifdef knightRiderTest
		static int color = 0;
		static int led = 0;
		static int direction = 1;
		for (int i = 0; i < PWM_BUFLEN; i++)
		{
			tlc5947.pwmBuffer.buf[i] = 0;
		}	
		rgbCalculateLedLevels(MAX_LED_LEVEL, led_array, NUM_LEDS);
	
		if (led < NUM_LEDS && led >= 0)
			rgbSetLEDColorAndLevel(colors[color], led_array[led]);
		if (led + 1 < NUM_LEDS)
			rgbSetLEDColorAndLevel(colors[color], led_array[led + 1]);
		if (led - 1 >= 0)
			rgbSetLEDColorAndLevel(colors[color], led_array[led - 1]);
		if (direction == 1) led++;
		else if (direction == -1) led--;
	  
		if (led >= NUM_LEDS)
		{
			direction = -1;
		}
	  
		if (led < 0)
		{
			direction = 1;
			color++;
			if (color >= MAX_COLOR_STRUCTS) color = 0;
		}
		tlc5947SetPWMOutputsFromBuffer(&tlc5947);
		HAL_Delay(75);
		#endif
	  
		#ifdef allLedsTransitionTest
		static int color_struct = 0;
		///*
		for(int i = 0 ; i < NUM_LEDS - 1 ; i++) //do all but last led here...
		{
			rgbTransitionLEDColor(colors[color_struct], /*i*/led_array[i]);
		}
		//*/
		//transitionLEDColor(colors[color_struct], 1);
		if(rgbTransitionLEDColor(colors[color_struct], /*NUM_LEDS - 1*/led_array[NUM_LEDS - 1])) //...do final led and check for transition
		{
			color_struct++;
			HAL_Delay(500);     //pause on this color for a bit
		}
		if (color_struct == MAX_COLOR_STRUCTS)
		{
			color_struct = 0;
		}
		//tlc5947SetPWMOutputsFromBuffer(pwmOutputs, REAL_PWM_LEN);
		tlc5947SetPWMOutputsFromBuffer(&tlc5947);
	 
		//HAL_Delay(10); //debug
		#endif
	  
		#ifdef	ledLevelsFromEncoderTest
	  
		rgbCalculateLedLevels(encoder.position, led_array, NUM_LEDS);
	  
		for (int i = 0; i < NUM_LEDS; i++)
		{
			if (button.muted == 1)
				rgbSetLEDColorAndLevel(colors[RED], led_array[i]);
			else
				rgbSetLEDColorAndLevel(colors[GREEN], led_array[i]);
		}
		tlc5947SetPWMOutputsFromBuffer(&tlc5947);  
		#endif
	  		
		HAL_UART_Receive_IT(&huart6, serialDebugRxChar, 1); 		//interrupt every character! callback will decide what to do with it
		
		if(debugOutput.response[0] != 0) //if there is anything in the debug output buffer, print it
		{
			HAL_UART_Transmit(&huart6, debugOutput.response, sizeof(debugOutput.response), HAL_MAX_DELAY);  
			for (int i = 0; i < MSG_RESPONSE_LEN; i++) debugOutput.response[i] = 0;
		}
		
		/*##-4- Start the Transmission process #####################################*/
		/*
		TxHeader.StdId = 0x11;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.DLC = 2;
		TxHeader.TransmitGlobalTime = DISABLE;
		TxData[0] = 0xCA;
		TxData[1] = 0xFE;
  
		uint32_t timeNow;
		static uint32_t timeThen = 0;
		static int okay;
		timeNow = HAL_GetTick();
		if (timeNow - timeThen > 20)
		{
			if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
			{
				uint32_t free;
				free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
				
				Error_Handler();
			}
			else
			{
				okay++;
			}
			timeThen = timeNow;
		}
		*/
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
					
		/* USER CODE END 3 */

	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART6;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PI3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PG7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
static void mapLedPointersToPwmOutputs(TLC5947 * tlc5947, LED_PTR led_array[], unsigned char numLeds)
{
	for (int i = 0; i < numLeds; i++)
	{
		for (int j = 0; j < LED_BYTES_PER_COLOR; j++)
		{
			led_array[i].r[j] = &tlc5947->pwmBuffer.buf[(LED_TOTAL_BYTE_WIDTH *(i + 1)) + j];     //(i + 1) instead of 1 because PWM outputs in reverse order, and last three pwm outputs have no LED connection (assuming 7 leds)
			led_array[i].g[j] = &tlc5947->pwmBuffer.buf[(LED_TOTAL_BYTE_WIDTH *(i + 1)) + j + LED_BYTES_PER_COLOR];
			led_array[i].b[j] = &tlc5947->pwmBuffer.buf[(LED_TOTAL_BYTE_WIDTH *(i + 1)) + j + (LED_BYTES_PER_COLOR * 2)];
		}				
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	if(GPIO_Pin == button.gpioPin)
	{
		buttonRead(&button);
	}
	else if(GPIO_Pin == encoder.aGpioPin)
	{
		encoderRead(&encoder);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart6)
	{
		serialDebugRead(huart);
	}
	
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
