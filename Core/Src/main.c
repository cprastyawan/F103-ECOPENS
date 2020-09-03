/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	PHASE_A, PHASE_B, PHASE_C
} PHASE;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define map(x,in_min,in_max,out_min,out_max) ( (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min )
#define constrain(nilaix,bawah,atas) ( (nilaix)<(bawah) ? (bawah) : ( (nilaix)>(atas) ? (atas) : (nilaix) ) )
#define FALLING 0
#define RISING 1
#define delaycomp for(int i=0; i<100; i++)
#define ADC_PHASE_A 1
#define ADC_PHASE_B 2
#define ADC_PHASE_C 3
#define ADC_NEUTRAL_POINT 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t ADCVal[5];
uint32_t offsetValue = 2500;
//uint32_t integralBEMFA = 0, integralBEMFB = 0, integralBEMFC = 0;
static uint32_t integralBEMF = 0;
static uint8_t step = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void outPWM(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t dutyCycle){
	dutyCycle = map(dutyCycle, 0, 0xFFFF, 0, 4800 - 1);
	__HAL_TIM_SET_COMPARE(htim, channel, dutyCycle);
}

void startMotor(){
	//Align Mode
	outPWM(&htim1, TIM_CHANNEL_1, 0xBFFF);
	//phase a floating, phase b gnd, phase c pwm
	//CH->BL
	HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_SET); //INH_C is HIGH, IC Active Mode
	HAL_GPIO_WritePin(INH_A_GPIO_Port, INH_A_Pin, GPIO_PIN_RESET); //INH_A connected to ground, IC A Sleep Mode
	HAL_GPIO_WritePin(IN_B_GPIO_Port, IN_B_Pin, GPIO_PIN_SET); //IN_B connected to ground
	HAL_GPIO_WritePin(INH_B_GPIO_Port, INH_B_Pin, GPIO_PIN_SET); //INH_B is high, IC B Active Mode
	HAL_Delay(250);

	//phase a pwm, phase b gnd, phase c floating
	//AH->BL
	HAL_GPIO_WritePin(IN_A_GPIO_Port, IN_A_Pin, GPIO_PIN_RESET); //IN_A connected to ground, PWM Mode
	HAL_GPIO_WritePin(INH_A_GPIO_Port, INH_A_Pin, GPIO_PIN_SET); //INH_A is HIGH, IC A Active Mode
	HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_RESET); //INH_C connected to ground, IC C Sleep Mode
	HAL_Delay(250);
}
void detectZeroCross(PHASE phase, bool mode){
	//Initialize pin for external interrupt from comparator output
	static GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Mode = (mode == RISING ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING); //Select Interrupt for back emf, rising or falling
	GPIO_InitStruct.Pin = (phase == PHASE_A ? COMP_PHASEA_Pin : phase == PHASE_B ? COMP_PHASEB_Pin : COMP_PHASEC_Pin);
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_ADC_Stop_IT(&hadc1);
	static ADC_ChannelConfTypeDef sConfig;

	switch(phase){
	case PHASE_A:
		sConfig.Channel = ADC_CHANNEL_4;
		HAL_NVIC_DisableIRQ(COMP_PHASEB_EXTI_IRQn);
		HAL_NVIC_DisableIRQ(COMP_PHASEC_EXTI_IRQn);
		delaycomp;
		HAL_NVIC_EnableIRQ(COMP_PHASEA_EXTI_IRQn);
		break;

	case PHASE_B:
		sConfig.Channel = ADC_CHANNEL_5;
		HAL_NVIC_DisableIRQ(COMP_PHASEA_EXTI_IRQn);
		HAL_NVIC_DisableIRQ(COMP_PHASEC_EXTI_IRQn);
		delaycomp;
		HAL_NVIC_EnableIRQ(COMP_PHASEB_EXTI_IRQn);
		break;

	case PHASE_C:
		sConfig.Channel = ADC_CHANNEL_6;
		HAL_NVIC_DisableIRQ(COMP_PHASEA_EXTI_IRQn);
		HAL_NVIC_DisableIRQ(COMP_PHASEB_EXTI_IRQn);
		delaycomp;
		HAL_NVIC_EnableIRQ(COMP_PHASEC_EXTI_IRQn);
		break;
	}
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}
void phase(int step){
	switch(step){
	//INH_X use open drain, SET = GND, RESET = FLOATING
	//IN_X use push pull, SET = HIGH, RESET = LOW
	case 1:
		//phase b floating, phase c ground, phase a pwm
		//AH->CL, B READ BACK EMF (RISING)
		HAL_GPIO_WritePin(INH_B_GPIO_Port, INH_B_Pin, GPIO_PIN_RESET); //INH_B connected to ground, IC B Sleep Mode
		HAL_GPIO_WritePin(IN_C_GPIO_Port, IN_C_Pin, GPIO_PIN_SET); //IN_C connected to ground
		HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_SET); //INH_C is high, IC C Active Mode
		detectZeroCross(PHASE_B, RISING);
		break;

	case 2:
		//phase a floating, phase b pwm, phase c ground
		//BH->CL, A READ BACK EMF (FALLING)
		HAL_GPIO_WritePin(INH_A_GPIO_Port, INH_A_Pin, GPIO_PIN_RESET); //INH_A connnected to ground, IC A Sleep Mode
		HAL_GPIO_WritePin(IN_B_GPIO_Port, IN_B_Pin, GPIO_PIN_RESET); //IN_B PWM Mode
		HAL_GPIO_WritePin(IN_B_GPIO_Port, INH_B_Pin, GPIO_PIN_SET); //INH_B high, IC B Active Mode
		detectZeroCross(PHASE_A, FALLING);
		break;

	case 3:
		//phase a gnd, phase b pwm, phase c floating
		//BH->AL, C READ BACK EMF (RISING)
		HAL_GPIO_WritePin(IN_A_GPIO_Port, IN_A_Pin, GPIO_PIN_SET); //IN_A connected to ground
		HAL_GPIO_WritePin(IN_A_GPIO_Port, INH_A_Pin, GPIO_PIN_SET); //INH_A is high, IC A Active Mode
		HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_RESET); //INH_C connected to ground. IC C Sleep Mode
		detectZeroCross(PHASE_C, RISING);
		break;

	case 4:
		//phase a gnd, phase b floating, phase c pwm
		//CH->AL, B READ BACK EMF (FALLING)
		HAL_GPIO_WritePin(INH_B_GPIO_Port, INH_B_Pin, GPIO_PIN_RESET); //INH_B connected to ground, IC B Sleep Mode
		HAL_GPIO_WritePin(IN_C_GPIO_Port, IN_C_Pin, GPIO_PIN_RESET); //IN_C floating, pwm mode
		HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_SET); //INH_C is high, IC C Active Mode
		detectZeroCross(PHASE_B, FALLING);
		break;
	case 5:
		//phase a floating, phase b gnd, phase c pwm
		//CH->BL, A READ BACK EMF (RISING)
		HAL_GPIO_WritePin(INH_A_GPIO_Port, INH_A_Pin, GPIO_PIN_RESET); //INH_A connected to ground, IC A Sleep Mode
		HAL_GPIO_WritePin(IN_B_GPIO_Port, IN_B_Pin, GPIO_PIN_SET); //IN_B connected to ground
		HAL_GPIO_WritePin(INH_B_GPIO_Port, INH_B_Pin, GPIO_PIN_SET); //INH_B is high, IC B Active Mode
		detectZeroCross(PHASE_A, RISING);
		break;

	case 6:
		//phase a pwm, phase b gnd, phase c floating
		//AH->BL, C READ BACK EMF (FALLING)
		HAL_GPIO_WritePin(IN_A_GPIO_Port, IN_A_Pin, GPIO_PIN_RESET); //IN_A connected to ground, PWM Mode
		HAL_GPIO_WritePin(INH_A_GPIO_Port, INH_A_Pin, GPIO_PIN_SET); //INH_A is HIGH, IC A Active Mode
		HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_RESET); //INH_C connected to ground, IC C Sleep Mode
		detectZeroCross(PHASE_C, FALLING);
		break;

	}
}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_DisableIRQ(COMP_PHASEA_EXTI_IRQn);
  HAL_NVIC_DisableIRQ(COMP_PHASEB_EXTI_IRQn);
  HAL_NVIC_DisableIRQ(COMP_PHASEC_EXTI_IRQn);
  HAL_UART_Transmit(&huart1, "Mulai\r\n", 7, 10);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_ADCEx_Calibration_Start(&hadc1);
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCVal, 5);
  startMotor();
  phase(step);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(integralBEMF >= offsetValue){
		  integralBEMF = 0;
		  step += 1;
		  if(step > 6) step = 1;
		  phase(step);
	  }
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4800 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
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
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, INH_A_Pin|IN_A_Pin|INH_B_Pin|IN_B_Pin
                          |INH_C_Pin|IN_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : COMP_PHASEA_Pin COMP_PHASEB_Pin COMP_PHASEC_Pin */
  GPIO_InitStruct.Pin = COMP_PHASEA_Pin|COMP_PHASEB_Pin|COMP_PHASEC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INH_A_Pin INH_B_Pin INH_C_Pin */
  GPIO_InitStruct.Pin = INH_A_Pin|INH_B_Pin|INH_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_A_Pin IN_B_Pin IN_C_Pin */
  GPIO_InitStruct.Pin = IN_A_Pin|IN_B_Pin|IN_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	/*if(GPIO_Pin == COMP_PHASEA_Pin){
		//HAL_ADC_Start_DMA(&hadc1, (uint32_t)&ADCVal[ADC_PHASE_A], 1);

	}
	else if(GPIO_Pin == COMP_PHASEB_Pin){
		//HAL_ADC_Start_DMA(&hadc1, (uint32_t)&ADCVal[ADC_PHASE_B], 1);
	}
	else if(GPIO_Pin == COMP_PHASEC_Pin){
		//HAL_ADC_Start_DMA(&hadc1, (uint32_t)&ADCVal[ADC_PHASE_C], 1);
	}*/

	HAL_ADC_Start_IT(&hadc1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//integralBEMFA += HAL_ADC_GetValue(&hadc1);
	//(bemf_phase == PHASE_A ? integralBEMFA : bemf_phase == PHASE_B ? integralBEMFB : integralBEMFC) += HAL_ADC_GetValue(&hadc1);
	integralBEMF += HAL_ADC_GetValue(&hadc1);
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
