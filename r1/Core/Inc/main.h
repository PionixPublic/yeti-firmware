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
#include "stm32g4xx_hal.h"

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
#define RCD_TEST_Pin GPIO_PIN_13
#define RCD_TEST_GPIO_Port GPIOC
#define RCD_DC_ERROR_Pin GPIO_PIN_14
#define RCD_DC_ERROR_GPIO_Port GPIOC
#define RCD_DC_ERROR_EXTI_IRQn EXTI15_10_IRQn
#define CP_3V3_Pin GPIO_PIN_0
#define CP_3V3_GPIO_Port GPIOC
#define ADC_12V_Pin GPIO_PIN_1
#define ADC_12V_GPIO_Port GPIOC
#define ADC_N12V_Pin GPIO_PIN_2
#define ADC_N12V_GPIO_Port GPIOC
#define PM_IRQ0_Pin GPIO_PIN_3
#define PM_IRQ0_GPIO_Port GPIOC
#define PP_3V3_Pin GPIO_PIN_0
#define PP_3V3_GPIO_Port GPIOA
#define EVSE_CP_IN_ADC_Pin GPIO_PIN_1
#define EVSE_CP_IN_ADC_GPIO_Port GPIOA
#define SLEEP_Pin GPIO_PIN_4
#define SLEEP_GPIO_Port GPIOA
#define FRAM_CS_Pin GPIO_PIN_0
#define FRAM_CS_GPIO_Port GPIOB
#define METER_CS_Pin GPIO_PIN_1
#define METER_CS_GPIO_Port GPIOB
#define PM_IRQ1_Pin GPIO_PIN_2
#define PM_IRQ1_GPIO_Port GPIOB
#define PM_IRQ1_EXTI_IRQn EXTI2_IRQn
#define MIRROR_L2L3_Pin GPIO_PIN_12
#define MIRROR_L2L3_GPIO_Port GPIOA
#define RCD_PWM_IN_Pin GPIO_PIN_6
#define RCD_PWM_IN_GPIO_Port GPIOC
#define POWERSWITCH_L2L3_Pin GPIO_PIN_9
#define POWERSWITCH_L2L3_GPIO_Port GPIOA
#define CFG_Pin GPIO_PIN_8
#define CFG_GPIO_Port GPIOC
#define GPIO2_Pin GPIO_PIN_8
#define GPIO2_GPIO_Port GPIOA
#define DISPLAY_RESET_LED3_Pin GPIO_PIN_11
#define DISPLAY_RESET_LED3_GPIO_Port GPIOA
#define DISPLAY_CS_LED1_Pin GPIO_PIN_12
#define DISPLAY_CS_LED1_GPIO_Port GPIOA
#define POWERSWITCH_L1_Pin GPIO_PIN_15
#define POWERSWITCH_L1_GPIO_Port GPIOA
#define GPIO4_Pin GPIO_PIN_10
#define GPIO4_GPIO_Port GPIOC
#define GPIO3_Pin GPIO_PIN_11
#define GPIO3_GPIO_Port GPIOC
#define MIRROR_L1_Pin GPIO_PIN_10
#define MIRROR_L1_GPIO_Port GPIOA
#define GPS_1PPS_Pin GPIO_PIN_4
#define GPS_1PPS_GPIO_Port GPIOB
#define GPIO1_Pin GPIO_PIN_5
#define GPIO1_GPIO_Port GPIOB
#define LOCK_F_Pin GPIO_PIN_6
#define LOCK_F_GPIO_Port GPIOB
#define LOCK_R_Pin GPIO_PIN_7
#define LOCK_R_GPIO_Port GPIOB
#define CP_ENABLE_Pin GPIO_PIN_9
#define CP_ENABLE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
