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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define UART1_TX_Pin GPIO_PIN_14
#define UART1_TX_GPIO_Port GPIOG
#define UART2_RX_Pin GPIO_PIN_7
#define UART2_RX_GPIO_Port GPIOB
#define DBUS_RX_Pin GPIO_PIN_11
#define DBUS_RX_GPIO_Port GPIOC
#define SIGNAL_SELECT_Pin GPIO_PIN_7
#define SIGNAL_SELECT_GPIO_Port GPIOI
#define PWM_6_Pin GPIO_PIN_6
#define PWM_6_GPIO_Port GPIOI
#define UART1_RX_Pin GPIO_PIN_9
#define UART1_RX_GPIO_Port GPIOG
#define UART2_TX_Pin GPIO_PIN_9
#define UART2_TX_GPIO_Port GPIOA
#define IST_SDA_Pin GPIO_PIN_9
#define IST_SDA_GPIO_Port GPIOC
#define IST_SCL_Pin GPIO_PIN_8
#define IST_SCL_GPIO_Port GPIOA
#define LASER_Pin GPIO_PIN_8
#define LASER_GPIO_Port GPIOC
#define PWM_5_Pin GPIO_PIN_6
#define PWM_5_GPIO_Port GPIOC
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_EXTI_IRQn EXTI3_IRQn
#define ADC_VIN_Pin GPIO_PIN_10
#define ADC_VIN_GPIO_Port GPIOF
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define BUZZER_Pin GPIO_PIN_14
#define BUZZER_GPIO_Port GPIOD
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define KEY_EXTI_IRQn EXTI0_IRQn
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define PWM_3_Pin GPIO_PIN_13
#define PWM_3_GPIO_Port GPIOE
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define PWM_1_Pin GPIO_PIN_9
#define PWM_1_GPIO_Port GPIOE
#define PWM_2_Pin GPIO_PIN_11
#define PWM_2_GPIO_Port GPIOE
#define PWM_4_Pin GPIO_PIN_14
#define PWM_4_GPIO_Port GPIOE
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
