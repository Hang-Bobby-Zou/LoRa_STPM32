/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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
#include "HAL_gpio.h"
/* USER CODE BEGIN 0 */
#include "sx1276.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
GpioIrqHandler *GpioIrq[16] = {0};
/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VC2_GPIO_Port, VC2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CTRL_SCS_Pin|CTRL_SYNC_Pin|USART3_DE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USART3__RE_GPIO_Port, USART3__RE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_NSS_Pin|CTRL_EN_Pin|VC1_Pin|CMD_PWR_LORA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_FLASH_GPIO_Port, CS_FLASH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = VC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VC2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin PAPin 
                           PAPin PAPin */
  GPIO_InitStruct.Pin = DIO0_Pin|DIO1_Pin|DIO2_Pin|DIO3_Pin 
                          |DIO4_Pin|DIO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = CTRL_SCS_Pin|CTRL_SYNC_Pin|USART3_DE_Pin|CS_FLASH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = USART3__RE_Pin|CTRL_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = NRST_1278_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRST_1278_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = SPI2_NSS_Pin|VC1_Pin|CMD_PWR_LORA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = CTRL_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CTRL_INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

void GPIO_Wakeup(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_SX1276_RX_CLK_ENABLE();
	GPIO_SX1276_TX_CLK_ENABLE();
  //GPIO_SX1276_RESET_CLK_ENABLE();
  //GPIO_SX1276_PWR_CLK_ENABLE();
	//GPIO_PWR_SENSOR_CLK_ENABLE();

	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	
	GPIO_InitStruct.Pin       = GPIO_SX1276_RESET_PIN;
	HAL_GPIO_WritePin( GPIO_SX1276_RESET_PORT, GPIO_SX1276_RESET_PIN, ( GPIO_PinState )1 );	
	HAL_GPIO_Init(GPIO_SX1276_RESET_PORT, &GPIO_InitStruct);	
	
	GPIO_InitStruct.Pin       = GPIO_SX1276_PWR_PIN;
	HAL_GPIO_WritePin( GPIO_SX1276_PWR_PORT, GPIO_SX1276_PWR_PIN, ( GPIO_PinState )1 );	
	HAL_GPIO_Init(GPIO_SX1276_PWR_PORT, &GPIO_InitStruct);	
	
	GPIO_InitStruct.Pin       = GPIO_SX1276_RX_PIN;
	HAL_GPIO_WritePin( GPIO_SX1276_RX_PORT, GPIO_SX1276_RX_PIN, ( GPIO_PinState )1 );	
	HAL_GPIO_Init(GPIO_SX1276_RX_PORT, &GPIO_InitStruct);		
	
  GPIO_InitStruct.Pin       = GPIO_SX1276_TX_PIN;
	HAL_GPIO_WritePin( GPIO_SX1276_TX_PORT, GPIO_SX1276_TX_PIN, ( GPIO_PinState )0);	
	HAL_GPIO_Init(GPIO_SX1276_TX_PORT, &GPIO_InitStruct);		
}



void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{
		GPIO_InitTypeDef   GPIO_InitStructure;
    GPIO_SX1276_DIO0_CLK_ENABLE();
    //GPIO_SX1276_DIO1_CLK_ENABLE();
    //GPIO_SX1276_DIO2_CLK_ENABLE();
    //GPIO_SX1276_DIO3_CLK_ENABLE();
    //GPIO_SX1276_DIO4_CLK_ENABLE();
	
		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStructure.Pull = GPIO_PULLUP;
	
		GPIO_InitStructure.Pin =  GPIO_SX1276_DIO0_PIN;
		HAL_GPIO_Init( GPIO_SX1276_DIO0_PORT, &GPIO_InitStructure );

		GPIO_InitStructure.Pin =  GPIO_SX1276_DIO1_PIN;
		HAL_GPIO_Init( GPIO_SX1276_DIO1_PORT, &GPIO_InitStructure );
	
		GPIO_InitStructure.Pin =  GPIO_SX1276_DIO2_PIN;
		HAL_GPIO_Init( GPIO_SX1276_DIO2_PORT, &GPIO_InitStructure );
	
		GPIO_InitStructure.Pin =  GPIO_SX1276_DIO3_PIN;
		HAL_GPIO_Init( GPIO_SX1276_DIO3_PORT, &GPIO_InitStructure );	

		GpioIrq[0] = irqHandlers[0] ;
		HAL_NVIC_SetPriority( EXTI0_IRQn , 1, 0 );
		HAL_NVIC_EnableIRQ( EXTI0_IRQn );	
		
		GpioIrq[1] = irqHandlers[1] ;
		HAL_NVIC_SetPriority( EXTI1_IRQn , 1, 0 );
		HAL_NVIC_EnableIRQ( EXTI1_IRQn );	

		GpioIrq[2] = irqHandlers[2] ;
		HAL_NVIC_SetPriority( EXTI2_IRQn , 1, 0 );
		HAL_NVIC_EnableIRQ( EXTI2_IRQn );	
		
		GpioIrq[3] = irqHandlers[3] ;
		HAL_NVIC_SetPriority( EXTI3_IRQn , 1, 0 );
		HAL_NVIC_EnableIRQ( EXTI3_IRQn );			
}

void GPIO_SX1276_Tx(void)
{
	HAL_GPIO_WritePin( GPIO_SX1276_RX_PORT, GPIO_SX1276_RX_PIN, ( GPIO_PinState )0 );
	HAL_GPIO_WritePin( GPIO_SX1276_TX_PORT, GPIO_SX1276_TX_PIN, ( GPIO_PinState )1 );
}

void GPIO_SX1276_Rx(void)
{
	HAL_GPIO_WritePin( GPIO_SX1276_RX_PORT, GPIO_SX1276_RX_PIN, ( GPIO_PinState )1 );
	HAL_GPIO_WritePin( GPIO_SX1276_TX_PORT, GPIO_SX1276_TX_PIN, ( GPIO_PinState )0 );
}
void GPIO_SX1276_Reset(void)
{
	HAL_GPIO_WritePin( GPIO_SX1276_RESET_PORT, GPIO_SX1276_RESET_PIN, ( GPIO_PinState )0 );
}

void GPIO_SX1276_ResetOff(void)
{
	HAL_GPIO_WritePin( GPIO_SX1276_RESET_PORT, GPIO_SX1276_RESET_PIN, ( GPIO_PinState )1 );
}

void GPIO_SX1276_PowerOn(void)
{
	HAL_GPIO_WritePin( GPIO_SX1276_PWR_PORT, GPIO_SX1276_PWR_PIN, ( GPIO_PinState )1 );
}

void GPIO_SX1276_PowerOff(void)
{
	HAL_GPIO_WritePin( GPIO_SX1276_PWR_PORT, GPIO_SX1276_PWR_PIN, ( GPIO_PinState )0 );
}

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
