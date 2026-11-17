/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define ULTRA_EN_Pin GPIO_PIN_13
#define ULTRA_EN_GPIO_Port GPIOC
#define XSHUT_2_Pin GPIO_PIN_15
#define XSHUT_2_GPIO_Port GPIOC
#define ROS_TX_Pin GPIO_PIN_2
#define ROS_TX_GPIO_Port GPIOA
#define ROS_RX_Pin GPIO_PIN_3
#define ROS_RX_GPIO_Port GPIOA
#define CURRENT_Pin GPIO_PIN_4
#define CURRENT_GPIO_Port GPIOA
#define OCV_Pin GPIO_PIN_5
#define OCV_GPIO_Port GPIOA
#define ENC_DRIVE_R_B_Pin GPIO_PIN_6
#define ENC_DRIVE_R_B_GPIO_Port GPIOA
#define ENC_DRIVE_R_A_Pin GPIO_PIN_7
#define ENC_DRIVE_R_A_GPIO_Port GPIOA
#define CHARGING_PINS_Pin GPIO_PIN_0
#define CHARGING_PINS_GPIO_Port GPIOB
#define BLDC_VAC_Pin GPIO_PIN_1
#define BLDC_VAC_GPIO_Port GPIOB
#define XSHUT_1_Pin GPIO_PIN_10
#define XSHUT_1_GPIO_Port GPIOB
#define DIR_L_Pin GPIO_PIN_12
#define DIR_L_GPIO_Port GPIOB
#define DIR_R_Pin GPIO_PIN_13
#define DIR_R_GPIO_Port GPIOB
#define S_BRUSH_RELAY1_Pin GPIO_PIN_14
#define S_BRUSH_RELAY1_GPIO_Port GPIOB
#define S_BRUSH_RELAY2_Pin GPIO_PIN_15
#define S_BRUSH_RELAY2_GPIO_Port GPIOB
#define PWM_DRIVE_R_Pin GPIO_PIN_8
#define PWM_DRIVE_R_GPIO_Port GPIOA
#define PWM_DRIVE_L_Pin GPIO_PIN_9
#define PWM_DRIVE_L_GPIO_Port GPIOA
#define PWM_VACUUM_Pin GPIO_PIN_10
#define PWM_VACUUM_GPIO_Port GPIOA
#define SR05_TX_Pin GPIO_PIN_11
#define SR05_TX_GPIO_Port GPIOA
#define SR05_RX_Pin GPIO_PIN_12
#define SR05_RX_GPIO_Port GPIOA
#define VAC_EN_Pin GPIO_PIN_15
#define VAC_EN_GPIO_Port GPIOA
#define XSHUT_3_Pin GPIO_PIN_4
#define XSHUT_3_GPIO_Port GPIOB
#define XSHUT_4_Pin GPIO_PIN_5
#define XSHUT_4_GPIO_Port GPIOB
#define ENC_DRIVE_L_A_Pin GPIO_PIN_6
#define ENC_DRIVE_L_A_GPIO_Port GPIOB
#define ENC_DRIVE_L_B_Pin GPIO_PIN_7
#define ENC_DRIVE_L_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
