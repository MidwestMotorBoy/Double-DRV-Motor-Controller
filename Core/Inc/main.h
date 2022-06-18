/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
//	extern uint32_t adc1[4];
//	extern uint32_t adc2[3];
//	extern uint32_t adc3[2];
	extern ADC_HandleTypeDef hadc1;
	extern ADC_HandleTypeDef hadc2;
	extern ADC_HandleTypeDef hadc3;
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
#define FAULT_LED_Pin GPIO_PIN_1
#define FAULT_LED_GPIO_Port GPIOA
#define BRD_EN_Pin GPIO_PIN_2
#define BRD_EN_GPIO_Port GPIOA
#define FAULT1_Pin GPIO_PIN_8
#define FAULT1_GPIO_Port GPIOD
#define DRV1_EN_Pin GPIO_PIN_9
#define DRV1_EN_GPIO_Port GPIOD
#define INV2_EN_Pin GPIO_PIN_15
#define INV2_EN_GPIO_Port GPIOD
#define INV1_EN_Pin GPIO_PIN_9
#define INV1_EN_GPIO_Port GPIOC
#define nFault2_Pin GPIO_PIN_2
#define nFault2_GPIO_Port GPIOD
#define DRV2_EN_Pin GPIO_PIN_3
#define DRV2_EN_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */
//Resolution_out = Resolution_in * FOSR^FORD,range of sensor 150*-3 -> 150*3
//150*6/(2*32^3)=0.0137
#define CURRENT_A_SCALER 0.01373291015f
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
