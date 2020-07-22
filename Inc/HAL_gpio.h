/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define GPIO_SX1276_RX_PIN                      GPIO_PIN_8  //VC1
#define GPIO_SX1276_RX_PORT                     GPIOB
#define GPIO_SX1276_RX_CLK_ENABLE()             __HAL_RCC_GPIOB_CLK_ENABLE()
                                              
#define GPIO_SX1276_TX_PIN                      GPIO_PIN_13  //VC2
#define GPIO_SX1276_TX_PORT                     GPIOC
#define GPIO_SX1276_TX_CLK_ENABLE()             __HAL_RCC_GPIOC_CLK_ENABLE()

#define GPIO_SX1276_RESET_PIN                   GPIO_PIN_2
#define GPIO_SX1276_RESET_PORT                  GPIOB
#define GPIO_SX1276_RESET_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()

#define GPIO_SX1276_PWR_PIN                     GPIO_PIN_9
#define GPIO_SX1276_PWR_PORT                    GPIOB
#define GPIO_SX1276_PWR_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()

#define GPIO_SX1276_DIO0_PIN                    GPIO_PIN_0
#define GPIO_SX1276_DIO0_PORT                   GPIOA
#define GPIO_SX1276_DIO0_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define GPIO_SX1276_DIO0_IRQn                   EXTI0_IRQn

#define GPIO_SX1276_DIO1_PIN                    GPIO_PIN_1
#define GPIO_SX1276_DIO1_PORT                   GPIOA
#define GPIO_SX1276_DIO1_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define GPIO_SX1276_DIO1_IRQn                   EXTI1_IRQn

#define GPIO_SX1276_DIO2_PIN                    GPIO_PIN_2
#define GPIO_SX1276_DIO2_PORT                   GPIOA
#define GPIO_SX1276_DIO2_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define GPIO_SX1276_DIO2_IRQn                   EXTI2_IRQn

#define GPIO_SX1276_DIO3_PIN                    GPIO_PIN_3
#define GPIO_SX1276_DIO3_PORT                   GPIOA
#define GPIO_SX1276_DIO3_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define GPIO_SX1276_DIO3_IRQn                   EXTI3_IRQn

#define GPIO_SX1276_DIO4_PIN                    GPIO_PIN_4
#define GPIO_SX1276_DIO4_PORT                   GPIOA
#define GPIO_SX1276_DIO4_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define GPIO_SX1276_DIO4_IRQn                   EXTI4_IRQn

#define GPIO_SX1276_DIO5_PIN                    GPIO_PIN_5
#define GPIO_SX1276_DIO5_PORT                   GPIOA
#define GPIO_SX1276_DIO5_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define GPIO_SX1276_DIO5_IRQn                   EXTI5_IRQn
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
void GPIO_Wakeup(void);
void GPIO_SX1276_Tx(void);
void GPIO_SX1276_Rx(void);
void GPIO_SX1276_Reset(void);
void GPIO_SX1276_ResetOff(void);
void GPIO_SX1276_PowerOn(void);
void GPIO_SX1276_PowerOff(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
