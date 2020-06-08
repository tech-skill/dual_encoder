/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 *
 * STM32f103 Cube MX project under FreeRTOS
 * DUAL_Encoder
 * Add result of a glass scale with a capacitive quill encoder result
 * and send it as quadrature A/B signal to a DRO
 *
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NROFSCALEBYTES 9	// Number of bytes sent by Quill-scale
#define ENCODERRESOLUTION 5	// 5 micrometer  resolution

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define sign(cnt) 	((cnt<0)?-1:1)
#define abs(cnt)	((cnt<0)?-1*cnt:cnt)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId mainTaskHandle;
osThreadId task_100msHandle;
osMessageQId advanceQueueHandle;
/* USER CODE BEGIN PV */

int32_t previousQuillHeight = 0;
int32_t previousZHeight = 0;

_Bool A[4] = { pdFALSE, pdTRUE, pdTRUE, pdFALSE };
_Bool B[4] = { pdFALSE, pdFALSE, pdTRUE, pdTRUE };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void StartMainTask(void const *argument);
void Starttask_100ms(void const *argument);

/* USER CODE BEGIN PFP */

int32_t quill2Micro();
int32_t metric2Micro();
int32_t imperial2Micro();
void Quadrature(int32_t advance);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t QuillScale_data[NROFSCALEBYTES];

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* definition and creation of advanceQueue */
	osMessageQDef(advanceQueue, 10, uint16_t);
	advanceQueueHandle = osMessageCreate(osMessageQ(advanceQueue), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of mainTask */
	osThreadDef(mainTask, StartMainTask, osPriorityNormal, 0, 128);
	mainTaskHandle = osThreadCreate(osThread(mainTask), NULL);

	/* definition and creation of task_100ms */
	osThreadDef(task_100ms, Starttask_100ms, osPriorityIdle, 0, 128);
	task_100msHandle = osThreadCreate(osThread(task_100ms), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	HAL_UART_Receive(&huart1, QuillScale_data, sizeof(QuillScale_data), 100);
	HAL_UART_Receive_DMA(&huart1, QuillScale_data, sizeof(QuillScale_data));
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 4800;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, DRO_A_Pin | DRO_B_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : GREEN_LED_Pin */
	GPIO_InitStruct.Pin = GREEN_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DRO_A_Pin DRO_B_Pin */
	GPIO_InitStruct.Pin = DRO_A_Pin | DRO_B_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// QuillScale  message complete.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart1) {
	int32_t QuillHeight;

	QuillHeight = quill2Micro();
	int32_t Diff = previousQuillHeight - QuillHeight;

	osMessagePut(advanceQueueHandle, Diff, 0);
	previousQuillHeight = QuillHeight;
	HAL_UART_Receive_DMA(huart1, QuillScale_data, sizeof(QuillScale_data));

}

int32_t metric2Micro() {
	int32_t Q = (QuillScale_data[1] - '0') * 1000000;
	Q += (QuillScale_data[2] - '0') * 100000;
	Q += (QuillScale_data[3] - '0') * 10000;
	Q += (QuillScale_data[4] - '0') * 1000;
	Q += (QuillScale_data[6] - '0') * 100;
	Q += (QuillScale_data[7] - '0') * 10;
	Q *= (QuillScale_data[0] == '-') ? -1 : 1;
	return Q;

}

int32_t imperial2Micro() {
	int32_t Q = (QuillScale_data[1] - '0') * 10000000;
	Q += (QuillScale_data[2] - '0') * 1000000;
	Q += (QuillScale_data[4] - '0') * 100000;
	Q += (QuillScale_data[5] - '0') * 10000;
	Q += (QuillScale_data[6] - '0') * 1000;
	Q += (QuillScale_data[7] - '0') * 100;
	Q *= ((QuillScale_data[0] == '-') ? -1 : 1) * 254;
	Q /= 10000;
	return Q;

}

int32_t quill2Micro() {
	if (QuillScale_data[5] == '.')
		return metric2Micro();
	if (QuillScale_data[3] == '.')
		return imperial2Micro();
	else
		return previousQuillHeight;

}

void Quadrature(int32_t count) {
	static uint8_t index = 0;
	int8_t dir = sign(count);
	// send AB signals
	for (int32_t counter = 0; counter < abs(count); counter++) {
		index += dir;
		index &= 0x3;
		HAL_GPIO_WritePin(GPIOA, DRO_A_Pin,
				A[index] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, DRO_B_Pin,
				B[index] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	__NOP();
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMainTask */
/**
 * @brief  Function implementing the mainTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osEvent Event = osMessageGet(advanceQueueHandle, 0);
		int32_t Advance = (int32_t) (void*) Event.value.p;
		Quadrature(Advance);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask_100ms */
/**
 * @brief Function implementing the task_100ms thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_100ms */
void Starttask_100ms(void const *argument) {
	/* USER CODE BEGIN StartTask_100ms */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	/* Infinite loop */
	for (;;) {
		int16_t ZHeight = TIM2->CNT * ENCODERRESOLUTION;
		int32_t Diff = previousZHeight - ZHeight;

		osMessagePut(advanceQueueHandle, Diff, 0);
		previousZHeight = ZHeight;
		osDelay(100);
	}
	/* USER CODE END StartTask_100ms */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
