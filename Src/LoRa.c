/**
 * \addtogroup LoRa
 * @{
 * \file LoRa.c
 * \author Hang Bobby Zou
 * \brief LoRa Code library
 */
 
/*============================================================================*/
/*                   STANDARD INCLUDE FILES                                   */
/*============================================================================*/
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>		// To include uintXX_t type
#include <stdbool.h>	// To include the bool type
#include <stdio.h>

/*============================================================================*/
/*                   INCLUDE FILES                                            */
/*============================================================================*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "main.h"
#include "usart.h"
#include "spi.h"
#include "LoRa.h"

uint8_t SPI2_RxBuffer[2] = {0};
uint8_t SPI2_TxBuffer[2] = {0};

bool LoRa_Init(void){
	
	/*
		The first byte is the address byte. It is comprises:
		` A wnr bit, which is 1 for write access and 0 for read access
		` Then 7 bits of address, MSB first
	*/
	// Enable Power LoRa
	HAL_GPIO_WritePin(CMD_PWR_LORA_GPIO_Port, CMD_PWR_LORA_Pin, GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(NRST_1278_GPIO_Port, NRST_1278_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(NRST_1278_GPIO_Port, NRST_1278_Pin, GPIO_PIN_SET);
	
	// Assmue read LoRa ID: 0(MSB)+0x42
	do{	
		USART3_PINSET_TX();
		
		myprintf("Checking LoRa Device ID\r\n\r\n");
		
		SPI2_TxBuffer[0] = 0x42;
		
		// HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
		
		// HAL_Delay(1);
		
		// if(HAL_SPI_Transmit(&hspi2, SPI2_TxBuffer, 1, 5) != HAL_OK){
		// 	myprintf("SPI2 Transmit error\r\n\r\n");
		// 	Error_Handler();
		// }
		
		// if(HAL_SPI_Receive(&hspi2, SPI2_RxBuffer,1, 5) != HAL_OK){
		// 	myprintf("SPI2 Receive error\r\n\r\n");
		// 	Error_Handler();
		// }
		
		SPI2_RxBuffer[0] = LoRa_ReadReg(SPI2_TxBuffer[0]);
		
		HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
		
		myprintf("SPI2 ID: %x %x \r\n\r\n", SPI2_RxBuffer[0], SPI2_RxBuffer[1]);

	} while (SPI2_RxBuffer[0] != 0x12);
	
	USART3_PINSET_RX();
	
	return true;
}

uint8_t LoRa_ReadReg(uint8_t Address){
	uint8_t ReturnVal;
	
	uint8_t Addr[1] = {Address};
	Addr[0] = Addr[0] & 0x7F;
	
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
		
	HAL_Delay(1);
		
	if(HAL_SPI_Transmit(&hspi2, &Addr[0], 1, 5) != HAL_OK){
		Error_Handler();
	}
		
	if(HAL_SPI_Receive(&hspi2, &ReturnVal,1, 5) != HAL_OK){
		Error_Handler();
	}
		
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	
	return ReturnVal;
	
}

void LoRa_WriteReg(uint8_t Address, uint8_t* WriteData){

	uint8_t Addr[1] = {Address};
	Addr[0] = Addr[0] | 0x80;
	
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
		
	HAL_Delay(1);
		
	if(HAL_SPI_Transmit(&hspi2, &Addr[0], 1, 5) != HAL_OK){
		Error_Handler();
	}
	
	if(HAL_SPI_Transmit(&hspi2, WriteData, 1, 5) != HAL_OK){
		Error_Handler();
	}

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	
}


