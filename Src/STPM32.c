/**
 * \addtogroup STPM32
 * @{
 * \file STPM32.c
 * \author Hang Bobby Zou
 * \brief STPM32 Code library
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
#include "STPM32.h"


//SYN timing defines, all Min.
#define t_ltch			20 	//ns, Time between de-selection and latch
#define t_lpw				4 	//us, Latch pulse width
#define t_w					4		//us, Time between two consecutive latch pulses
#define t_rpw				4		//us, Reset pulse width
#define t_rel				40	//ns, Time between pulse and selection
#define t_startup 	35	//ms, Time between power on and reset

//Register Address Map
#define DSP_CR3			0x04



bool SendMessage(uint8_t* Message, uint32_t Address);

bool STPM32_Init(void) {
		// Initializing STPM32
		/* Sequence: 	0. Before initializing
									1. Enable Pin set low
									2. Enable Pin set high
									3. Wait for t_startup time or PowrOK signal 
									4. Three SYN low signal of time t_rpw
									7. Do antoher software reset by writing the reset bit (S/W reset in DSP_CR3)
		
									8. Configure all parameters (not a lot)
									9. Communicate:
											8.1 Tx: Read Address, Write Address, LS Data[7:0], MS Data[15:8], CRC Byte 
											8.2 Rx: Data[7:0], Data[15:8], Data[23:16], Data[31:24], CRC Byte
											8.2 Dummy read address 0xFF increments by one the internal read pointer
											8.4 Dummy write address 0xFF specifies that no writing is requested
									10. Other: 
											BREAK frame: if received, a break flag is set and the whole packet reception aborts
											IDLE frame: the receiver can recognize an IDLE frame
											LATCH: 	1. One SYN pulse with SCS set to high. 
															2. Write to channel latch bits before read (S/W Latchx in DSP_CR3).
															3. Writing auto-latch bit (S/W Auto Latch in DSP_CR3)
		*/
		//First set SYNC and SCS to high before EN reset
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(CTRL_SCS_GPIO_Port,CTRL_SCS_Pin,GPIO_PIN_SET);
		
		//EN reset
		HAL_GPIO_WritePin(CTRL_EN_GPIO_Port, CTRL_EN_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(CTRL_EN_GPIO_Port, CTRL_EN_Pin, GPIO_PIN_SET);
		
		//Startup reset
		HAL_Delay(t_startup);
		
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_RESET);
		HAL_Delay(1); // = t_rpw
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_SET);
		HAL_Delay(1); // = t_rpw
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_RESET);
		HAL_Delay(1); // = t_rpw
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_SET);
		HAL_Delay(1); // = t_rpw
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_RESET);
		HAL_Delay(1); // = t_rpw
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_SET);
		HAL_Delay(1); // = t_rpw
		
		//Software reset
		uint8_t SW_Reset_Message [2] = {0};
			
		SW_Reset_Message[0] = 0x00;
		SW_Reset_Message[1] = 0x10;
		//= 001004E0		Writing the reset bit to perform a software reset
		if (SendMessage(SW_Reset_Message,DSP_CR3) != true){
			Error_Handler();
		}
		
		
		return true;
}

bool SendMessage(uint8_t* Message, uint32_t Address){
	/* 
		| ReadAddress | WriteAddress | LS Data [7:0] | MS Data [15:8] | CRC Byte |
		|		 0xFF 		|		 Address	 | 	 Message[0]  |   Message[1]   |    --    |
	*/
	uint8_t SendMessage[1];
		
	SendMessage[0] = 0xFF;
	if (HAL_UART_Transmit_IT(&huart1, (uint8_t*)SendMessage, sizeof(SendMessage))!= HAL_OK)
	{
		Error_Handler();
	}
	
	SendMessage[0] = Address;
	if (HAL_UART_Transmit_IT(&huart1, (uint8_t*)SendMessage, sizeof(SendMessage))!= HAL_OK)
	{
		Error_Handler();
	}
	
	SendMessage[0] = Message[0];
	if (HAL_UART_Transmit_IT(&huart1, (uint8_t*)SendMessage, sizeof(SendMessage))!= HAL_OK)
	{
		Error_Handler();
	}
	
	SendMessage[0] = Message[1];
	if (HAL_UART_Transmit_IT(&huart1, (uint8_t*)SendMessage, sizeof(SendMessage))!= HAL_OK)
	{
		Error_Handler();
	}
	
	return true;
	
}

