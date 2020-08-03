/**
  ******************************************************************************
  * File Name          : TIM.h
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

extern TIM_HandleTypeDef htim7;

/* USER CODE BEGIN Private defines */
static uint32_t TIM7_Clock = 0;
static uint32_t TIM7_Irq_Num = 0;
static uint32_t TIM7_Point1 = 0;
static uint32_t TIM7_Point2 = 0;

#define TIM7_COUNT_CLOCK      (200000)
#define TIM7_IRQ_CLOCK        (20)
#define TIM7_PERIOD           (TIM7_COUNT_CLOCK / TIM7_IRQ_CLOCK);

void DelayMsPoll(int x);
/* USER CODE END Private defines */

void MX_TIM7_Init(void);

/* USER CODE BEGIN Prototypes */
uint32_t TIM7_GetTimeMs(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
