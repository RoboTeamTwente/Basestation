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
#include "stm32f7xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define APB1 108
#define APB2 216
#define SX_RX_IRQ_Pin GPIO_PIN_3
#define SX_RX_IRQ_GPIO_Port GPIOE
#define SX_RX_IRQ_EXTI_IRQn EXTI3_IRQn
#define SX_RX_BUSY_Pin GPIO_PIN_4
#define SX_RX_BUSY_GPIO_Port GPIOE
#define SX_RX_RST_Pin GPIO_PIN_14
#define SX_RX_RST_GPIO_Port GPIOC
#define LED_RX_Pin GPIO_PIN_15
#define LED_RX_GPIO_Port GPIOC
#define QUADSPI_IRQ_Pin GPIO_PIN_4
#define QUADSPI_IRQ_GPIO_Port GPIOF
#define QUADSPI_IRQ_EXTI_IRQn EXTI4_IRQn
#define USB_RST_Pin GPIO_PIN_4
#define USB_RST_GPIO_Port GPIOA
#define LED_TX_Pin GPIO_PIN_7
#define LED_TX_GPIO_Port GPIOC
#define SX_TX_CS_Pin GPIO_PIN_8
#define SX_TX_CS_GPIO_Port GPIOA
#define SX_TX_RST_Pin GPIO_PIN_10
#define SX_TX_RST_GPIO_Port GPIOA
#define SX_TX_BUSY_Pin GPIO_PIN_11
#define SX_TX_BUSY_GPIO_Port GPIOA
#define SX_TX_IRQ_Pin GPIO_PIN_12
#define SX_TX_IRQ_GPIO_Port GPIOA
#define SX_TX_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define LED_LED3_Pin GPIO_PIN_9
#define LED_LED3_GPIO_Port GPIOG
#define LED_LED2_Pin GPIO_PIN_10
#define LED_LED2_GPIO_Port GPIOG
#define LED_LED1_Pin GPIO_PIN_11
#define LED_LED1_GPIO_Port GPIOG
#define LED_USB_Pin GPIO_PIN_12
#define LED_USB_GPIO_Port GPIOG
#define LED_ACTIVE_Pin GPIO_PIN_13
#define LED_ACTIVE_GPIO_Port GPIOG
#define SW4_Pin GPIO_PIN_7
#define SW4_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_8
#define SW3_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_9
#define SW2_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_0
#define SW1_GPIO_Port GPIOE
#define SX_RX_CS_Pin GPIO_PIN_1
#define SX_RX_CS_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
