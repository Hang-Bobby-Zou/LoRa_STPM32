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

bool LoRa_Init(void){
	
	/*
		The first byte is the address byte. It is comprises:
		` A wnr bit, which is 1 for write access and 0 for read access
		` Then 7 bits of address, MSB first
	*/
	// Enable Power LoRa
	HAL_GPIO_WritePin(CMD_PWR_LORA_GPIO_Port, CMD_PWR_LORA_Pin, GPIO_PIN_SET);
	
	
	
	return true;
}


