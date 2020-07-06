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

uint8_t SPI2_RxBuffer[1] = {0};
uint8_t SPI2_TxBuffer[2] = {0};

bool LoRa_Init(void){
	
	/*
		The first byte is the address byte. It is comprises:
		` A wnr bit, which is 1 for write access and 0 for read access
		` Then 7 bits of address, MSB first
	*/
	
	// Assmue read LoRa ID: 0+0x42
	SPI2_TxBuffer[0] = 0x42;
	
	HAL_SPI_Receive_IT(&hspi2, SPI2_RxBuffer, 2);
	HAL_SPI_Transmit(&hspi2, SPI2_TxBuffer, 2, 0xFFFF);

	
	if (SPI2_RxFlag == 1){
		SPI2_RxFlag = 0;
		
		USART3_PINSET_TX();
		myprintf("%x", SPI2_RxBuffer);
		USART3_PINSET_RX();
	}
	
	
	
	return true;
}


