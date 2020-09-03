/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define COMP_PHASEA_Pin GPIO_PIN_0
#define COMP_PHASEA_GPIO_Port GPIOA
#define COMP_PHASEA_EXTI_IRQn EXTI0_IRQn
#define COMP_PHASEB_Pin GPIO_PIN_1
#define COMP_PHASEB_GPIO_Port GPIOA
#define COMP_PHASEB_EXTI_IRQn EXTI1_IRQn
#define COMP_PHASEC_Pin GPIO_PIN_2
#define COMP_PHASEC_GPIO_Port GPIOA
#define COMP_PHASEC_EXTI_IRQn EXTI2_IRQn
#define ADC_PHASEA_Pin GPIO_PIN_4
#define ADC_PHASEA_GPIO_Port GPIOA
#define ADC_PHASEB_Pin GPIO_PIN_5
#define ADC_PHASEB_GPIO_Port GPIOA
#define ADC_PHASEC_Pin GPIO_PIN_6
#define ADC_PHASEC_GPIO_Port GPIOA
#define ADC_PHASEN_Pin GPIO_PIN_7
#define ADC_PHASEN_GPIO_Port GPIOA
#define INH_A_Pin GPIO_PIN_3
#define INH_A_GPIO_Port GPIOB
#define IN_A_Pin GPIO_PIN_4
#define IN_A_GPIO_Port GPIOB
#define INH_B_Pin GPIO_PIN_5
#define INH_B_GPIO_Port GPIOB
#define IN_B_Pin GPIO_PIN_6
#define IN_B_GPIO_Port GPIOB
#define INH_C_Pin GPIO_PIN_7
#define INH_C_GPIO_Port GPIOB
#define IN_C_Pin GPIO_PIN_8
#define IN_C_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
