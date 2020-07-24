/**
  ******************************************************************************
  * File Name          : TIM.c
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

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* TIM6 init function */
void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
/* TIM7 init function */
void MX_TIM7_Init(void)
{

	TIM7_Clock = SystemCoreClock *2 / 8;
	
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = (uint32_t)(TIM7_Clock / TIM7_COUNT_CLOCK) - 1;
  htim7.Init.Period = (uint32_t)TIM7_PERIOD;
	htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
	
	TIM7_Irq_Num = 0;
	
	__HAL_TIM_CLEAR_IT(&htim7,TIM_IT_UPDATE);
	__HAL_TIM_SET_COUNTER(&htim7,0);
	
  if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* TIM6 clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();
  /* USER CODE BEGIN TIM6_MspInit 1 */

  /* USER CODE END TIM6_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspInit 0 */

  /* USER CODE END TIM7_MspInit 0 */
    /* TIM7 clock enable */
    __HAL_RCC_TIM7_CLK_ENABLE();

  /* USER CODE BEGIN TIM7_MspInit 1 */
    /* TIM7 interrupt Init */
    HAL_NVIC_SetPriority(TIM7_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* USER CODE END TIM7_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();
  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspDeInit 0 */

  /* USER CODE END TIM7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM7_CLK_DISABLE();

    /* TIM7 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspDeInit 1 */

  /* USER CODE END TIM7_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
uint32_t TIM7_GetTimeMs(void)
{
	uint32_t TimeValue = 0;
  
	TimeValue = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim7));
	//TimeValue = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim1));
	
	TimeValue = TimeValue & 0xffff;
	TimeValue = TimeValue + TIM7_Irq_Num * TIM7_PERIOD;
	TimeValue *= 1000u;	//Convert in ms
	TimeValue /= TIM7_COUNT_CLOCK;
	if(TimeValue == 0)
		TimeValue=1;	
	return  TimeValue;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
