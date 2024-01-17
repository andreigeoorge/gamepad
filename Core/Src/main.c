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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct Joystick {
	uint8_t up;
	uint8_t down;
	uint8_t left;
	uint8_t right;
} JoystickTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define	GAMEPAD_BUFF_SIZE_UC	3U
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

uint32_t 	DBG_SIZE_PIN_STATE;
GPIO_PinState		DBG_VAL_PIN_STATE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void JoystickToLeds(uint8_t ucAdcXIn, uint8_t ucAdcYIn, uint8_t *ucPosUpOut,
		uint8_t *ucPosDownOut, uint8_t *ucPosLeftOut, uint8_t *ucPosRightOut);

void JoystickToUsb(uint8_t ucAdcXIn, uint8_t ucAdcYIn, uint8_t *scUsbXOut,
		uint8_t *scUsbYOut);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	uint32_t uiAdc1Val = 0U;
	uint32_t uiAdc2Val = 0U;
	GPIO_PinState BtnState0, BtnState1, BtnState2, BtnState3;
	JoystickTypeDef Joystick_t;
	uint8_t ucGamePadBuffer[GAMEPAD_BUFF_SIZE_UC];
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
	MX_USB_DEVICE_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	ucGamePadBuffer[0] = 0U; /*X movement (0: not moving, +ve: Right movement, -ve: Left movement)*/
	ucGamePadBuffer[1] = 0U; /*Y movement (0: not moving, +ve: Down movement, -ve: Up movement)*/
	ucGamePadBuffer[2] = 0x90U; /*Right and Left clicks (0: None, 1: Left Click, 2: Right click)*/

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/*Get the ADC1 value*/
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 5U) == HAL_OK) {
			uiAdc1Val = HAL_ADC_GetValue(&hadc1);
		}
		HAL_ADC_Stop(&hadc1);

		/*Get the ADC2 value*/
		HAL_ADC_Start(&hadc2);
		if (HAL_ADC_PollForConversion(&hadc2, 5U) == HAL_OK) {
			uiAdc2Val = HAL_ADC_GetValue(&hadc2);
		}
		HAL_ADC_Stop(&hadc2);

		JoystickToLeds(uiAdc1Val, uiAdc2Val, &Joystick_t.up, &Joystick_t.down,
				&Joystick_t.left, &Joystick_t.right);

		/*Modify PWM duty cycle according to ADCs values*/
		htim4.Instance->CCR1 = (uint32_t) Joystick_t.down;
		htim4.Instance->CCR2 = (uint32_t) Joystick_t.right;
		htim4.Instance->CCR3 = (uint32_t) Joystick_t.up;
		htim4.Instance->CCR4 = (uint32_t) Joystick_t.left;

		/*Read GPIO Buttons state and update the GamePad signal*/
		BtnState0 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);
		BtnState1 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
		BtnState2 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
		BtnState3 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5);

		ucGamePadBuffer[2] = (((uint8_t)BtnState0 << 0U) | ((uint8_t)BtnState1 << 1U)
				| ((uint8_t)BtnState2 << 2U) | ((uint8_t)BtnState3 << 3U) | 0x90U);

		/*Convert Joystick reads into Mouse signals*/
		JoystickToUsb(uiAdc1Val, uiAdc2Val, &ucGamePadBuffer[0],
				&ucGamePadBuffer[1]);

		ucGamePadBuffer[2] 	= 0x90U;
		DBG_SIZE_PIN_STATE	= sizeof(GPIO_PinState);
		DBG_VAL_PIN_STATE	= BtnState0;

		/*Send the Mouse signals through USB*/
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, ucGamePadBuffer,
				GAMEPAD_BUFF_SIZE_UC);
		HAL_Delay(100U);

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_8B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION_8B;
	hadc2.Init.ScanConvMode = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 16;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 100;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 100;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pins : PE2 PE3 PE4 PE5 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void JoystickToLeds(uint8_t ucAdcXIn, uint8_t ucAdcYIn, uint8_t *ucPosUpOut,
		uint8_t *ucPosDownOut, uint8_t *ucPosLeftOut, uint8_t *ucPosRightOut) {
	/*X AXIS*/
	if (ucAdcXIn > 123U && ucAdcXIn < 135U) /*MIDDLE*/
	{
		*ucPosUpOut = 0U;
		*ucPosDownOut = 0U;
	} else if (ucAdcXIn >= 135U) /*UP*/
	{
		*ucPosUpOut = (128U + (int8_t) ucAdcXIn) / 2U;
		*ucPosDownOut = 0U;
	} else if (ucAdcXIn <= 123U) /*DOWN*/
	{
		*ucPosUpOut = 0U;
		*ucPosDownOut = (128U - (int8_t) ucAdcXIn) / 2U;
	}

	/*Y AXIS*/
	if (ucAdcYIn > 123U && ucAdcYIn < 135U) /*MIDDLE*/
	{
		*ucPosLeftOut = 0U;
		*ucPosRightOut = 0U;
	} else if (ucAdcYIn >= 135U) /*RIGHT*/
	{
		*ucPosLeftOut = 0U;
		*ucPosRightOut = (128U + (int8_t) ucAdcYIn) / 2U;
	} else if (ucAdcYIn <= 123U) /*LEFT*/
	{
		*ucPosLeftOut = (128U - (int8_t) ucAdcYIn) / 2U;
		*ucPosRightOut = 0U;
	}
}

void JoystickToUsb(uint8_t ucAdcXIn, uint8_t ucAdcYIn, uint8_t *scUsbXOut,
		uint8_t *scUsbYOut) {
	/*X AXIS*/
	if (ucAdcXIn > 123U && ucAdcXIn < 135U) /*MIDDLE*/
	{
		*scUsbXOut = 0U;
	} else if (ucAdcXIn >= 135U) /*UP*/
	{
		*scUsbXOut = (ucAdcXIn - 127U);
	} else if (ucAdcXIn >= 0 && ucAdcXIn <= 127U) /*DOWN*/
	{
		*scUsbXOut = (ucAdcXIn - 127U);
	}

	/*Y AXIS*/
	if (ucAdcYIn > 123U && ucAdcYIn < 135U) /*MIDDLE*/
	{
		*scUsbYOut = 0U;
	} else if (ucAdcYIn >= 135U) /*RIGHT*/
	{
		*scUsbYOut = -(ucAdcYIn - 127U);
	} else if (ucAdcYIn >= 0 && ucAdcYIn <= 127U) /*LEFT*/
	{
		*scUsbYOut = -(ucAdcYIn - 127U);
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
