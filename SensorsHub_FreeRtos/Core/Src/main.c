/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <math.h>
#include <stdio.h>
#include"lcd_txt.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VIBRATION_SENSOR GPIO_PIN_1
#define MOTION_SENSOR GPIO_PIN_2

#define SERIESRESISTOR 10000          //Value of the series resistor in ohms
#define THERMISTORNOMINAL 10000       //Nominal resistance of the thermistor at 25°C in ohms
#define TEMPERATURENOMINAL 25           //Nominal temperature for the thermistor resistance in degrees Celsius
#define BCOEFFICIENT 3950

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lcdDispTask */
osThreadId_t lcdDispTaskHandle;
const osThreadAttr_t lcdDispTask_attributes = {
  .name = "lcdDispTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AdcReadingTa */
osThreadId_t AdcReadingTaHandle;
const osThreadAttr_t AdcReadingTa_attributes = {
  .name = "AdcReadingTa",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for temperatureValues */
osMessageQueueId_t temperatureValuesHandle;
const osMessageQueueAttr_t temperatureValues_attributes = {
  .name = "temperatureValues"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void lcdDisplayTask(void *argument);
void AdcReadingTask(void *argument);

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of temperatureValues */
  temperatureValuesHandle = osMessageQueueNew (10, sizeof(float), &temperatureValues_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of lcdDispTask */
  lcdDispTaskHandle = osThreadNew(lcdDisplayTask, NULL, &lcdDispTask_attributes);

  /* creation of AdcReadingTa */
  AdcReadingTaHandle = osThreadNew(AdcReadingTask, NULL, &AdcReadingTa_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
	case VIBRATION_SENSOR:
		osThreadFlagsSet (lcdDispTaskHandle, 0x000F);
		break;
	case MOTION_SENSOR:
		osThreadFlagsSet (lcdDispTaskHandle, 0x001F);
		break;

	}
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_lcdDisplayTask */
/**
 * @brief Function implementing the lcdDispTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_lcdDisplayTask */
void lcdDisplayTask(void *argument)
{
  /* USER CODE BEGIN lcdDisplayTask */

	float tempValue = 0;
	char msg[30];
	/* Infinite loop */
	for(;;)
	{

		uint32_t notifValue = osThreadFlagsWait (0x000F, osFlagsWaitAll, osWaitForever);
		switch (notifValue)
		{
		case 0x000F:
			lcd_puts((uint8_t*)" vibration detected" , strlen (" vibration detected"));

			break;
		case 0x001F:
			lcd_puts((uint8_t*)" motion detected" , strlen (" motion detected"));

		case 0x002F:

			osMessageQueueGet (temperatureValuesHandle, &tempValue, 0, osWaitForever);
			sprintf(msg, " LM35: %.2f C\r\n",tempValue);
			lcd_puts((uint8_t*)msg, strlen (msg));
		case 0x003F:

			osMessageQueueGet (temperatureValuesHandle, &tempValue, 0, osWaitForever);
			sprintf(msg, " NTC: %.2f *C\r\n", tempValue);  // Format the temperature value for UART output
			lcd_puts((uint8_t*)msg, strlen (msg));


		}
		HAL_Delay(500);
		lcd_clear();


		osDelay(100);
	}
  /* USER CODE END lcdDisplayTask */
}

/* USER CODE BEGIN Header_AdcReadingTask */
/**
 * @brief Function implementing the AdcReadingTa thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_AdcReadingTask */
void AdcReadingTask(void *argument)
{
  /* USER CODE BEGIN AdcReadingTask */

	float temp = 0;
	float steinhart=0;
	/* Infinite loop */
	for(;;)
	{

		for(int i=0; i<2; i++)
		{

			HAL_ADC_Start(&hadc1); // Start ADC Conversion
			HAL_ADC_PollForConversion(&hadc1, 1); // Poll ADC1 Peripheral & TimeOut = 1mSec
			uint16_t AD_RES = HAL_ADC_GetValue(&hadc1); // Read ADC Conversion Result
			switch(i)
			{
			case 0:

				temp = ((AD_RES * 5000 /4096) + 5)/10;


				osMessageQueuePut (temperatureValuesHandle, &temp, 0, osWaitForever);
				osThreadFlagsSet (lcdDispTaskHandle, 0x002F);

				break;
			case 1:
				temp = AD_RES;
				if (temp > 0) {  // Ensure the average is not zero to avoid division errors
					temp = 4094 / temp - 1;  // Convert ADC reading to resistance using 12-bit ADC resolution
					temp = SERIESRESISTOR / temp;  // Calculate the thermistor resistance

					// Apply the Steinhart-Hart equation to convert resistance to temperature
					steinhart = temp / THERMISTORNOMINAL;  //Compute the ratio of thermistor resistance to nominal resistance
					steinhart = log(steinhart);  //Compute the natural logarithm of the resistance ratio
					steinhart /= BCOEFFICIENT;  //Divide by the thermistor's Beta coefficient
					steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);  //Add the reciprocal of the nominal temperature in Kelvin
					steinhart = 1.0 / steinhart;  //Compute the reciprocal to get the temperature in Kelvin
					steinhart -= 273.15;  //Convert the temperature from Kelvin to Celsius

					osMessageQueuePut (temperatureValuesHandle, &steinhart, 0, osWaitForever);
					osThreadFlagsSet (lcdDispTaskHandle, 0x003F);


				}
				break;

			}
			osDelay(5000);
		}
	}
  /* USER CODE END AdcReadingTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
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
