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
#include "stm32l4xx_hal.h"

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
#define VC2_Pin GPIO_PIN_13
#define VC2_GPIO_Port GPIOC
#define DIO0_Pin GPIO_PIN_0
#define DIO0_GPIO_Port GPIOA
#define DIO1_Pin GPIO_PIN_1
#define DIO1_GPIO_Port GPIOA
#define DIO2_Pin GPIO_PIN_2
#define DIO2_GPIO_Port GPIOA
#define DIO3_Pin GPIO_PIN_3
#define DIO3_GPIO_Port GPIOA
#define DIO4_Pin GPIO_PIN_4
#define DIO4_GPIO_Port GPIOA
#define CTRL_SCS_Pin GPIO_PIN_5
#define CTRL_SCS_GPIO_Port GPIOA
#define CTRL_SYNC_Pin GPIO_PIN_6
#define CTRL_SYNC_GPIO_Port GPIOA
#define USART3_DE_Pin GPIO_PIN_7
#define USART3_DE_GPIO_Port GPIOA
#define USART3__RE_Pin GPIO_PIN_0
#define USART3__RE_GPIO_Port GPIOB
#define NRST_1278_Pin GPIO_PIN_2
#define NRST_1278_GPIO_Port GPIOB
#define DIO5_Pin GPIO_PIN_8
#define DIO5_GPIO_Port GPIOA
#define CS_FLASH_Pin GPIO_PIN_15
#define CS_FLASH_GPIO_Port GPIOA
#define SCK_FLASH_Pin GPIO_PIN_3
#define SCK_FLASH_GPIO_Port GPIOB
#define MISO_FLASH_Pin GPIO_PIN_4
#define MISO_FLASH_GPIO_Port GPIOB
#define MOSI_FLASH_Pin GPIO_PIN_5
#define MOSI_FLASH_GPIO_Port GPIOB
#define CTRL_INT_Pin GPIO_PIN_6
#define CTRL_INT_GPIO_Port GPIOB
#define CTRL_EN_Pin GPIO_PIN_7
#define CTRL_EN_GPIO_Port GPIOB
#define VC1_Pin GPIO_PIN_8
#define VC1_GPIO_Port GPIOB
#define CMD_PWR_LORA_Pin GPIO_PIN_9
#define CMD_PWR_LORA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
