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
#include "STPM32_AddressMap.h"

//CRC calc defines
#define u8 unsigned char
#define CRC_8 (0x07)
#define STPM3x_FRAME_LEN (5)
static char CRC_u8Checksum;



//SYN timing defines, all Min.
//#define t_ltch			20 	//ns, Time between de-selection and latch
//#define t_lpw				4 	//us, Latch pulse width
//#define t_w					4		//us, Time between two consecutive latch pulses
//#define t_rel				40	//ns, Time between pulse and selection

// Power-on procedure TYPICAL timing
#define t_if				10	//ms, Time for interface choice locking
#define t_startup 	35	//ms, Time between power on and reset
#define t_rpw				1		//ms, Reset pulse width
#define t_scs				1		//ms, Delay from SYN to SCS

//Function decleration
static u8 CalcCRC8(u8 *pBuf);
static void Crc8Calc (u8 u8Data);
void FRAME_for_UART_mode(u8 *pBuf);
static u8 byteReverse(u8 n);

bool STPM32_Init(void) {
		// Initializing STPM32
		/* Sequence: 	0. Before initializing
									1. Enable Pin set low
									2. Enable Pin set high
									3. Wait for t_startup time or PowrOK signal 
									4. Three SYN low signal of time t_rpw
		
									5. Configure all parameters (not a lot)
									6. Communicate:
											8.1 Tx: Read Address, Write Address, LS Data[7:0], MS Data[15:8], CRC Byte 
											8.2 Rx: Data[7:0], Data[15:8], Data[23:16], Data[31:24], CRC Byte
											8.2 Dummy read address 0xFF increments by one the internal read pointer
											8.4 Dummy write address 0xFF specifies that no writing is requested
									7. Other: 
											BREAK frame: if received, a break flag is set and the whole packet reception aborts
											IDLE frame: the receiver can recognize an IDLE frame
											LATCH: 	1. One SYN pulse with SCS set to high. 
															2. Write to channel latch bits before read (S/W Latchx in DSP_CR3).
															3. Writing auto-latch bit (S/W Auto Latch in DSP_CR3)
		*/
		//First set everything low.
		HAL_GPIO_WritePin(CTRL_EN_GPIO_Port, CTRL_EN_Pin, GPIO_PIN_SET);				// EN	High
		
		HAL_Delay(3000);
		
		HAL_GPIO_WritePin(CTRL_EN_GPIO_Port, CTRL_EN_Pin, GPIO_PIN_RESET);			// EN	Low
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_RESET);	// SYNC Low
		HAL_GPIO_WritePin(CTRL_SCS_GPIO_Port, CTRL_SCS_Pin, GPIO_PIN_RESET);		// SCS Low
		
		HAL_Delay(100);
		//Then set SYNC and SCS to high before EN starts
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_SET);		// SYNC High
		HAL_GPIO_WritePin(CTRL_SCS_GPIO_Port,CTRL_SCS_Pin,GPIO_PIN_SET);				// SCS High
		
		HAL_Delay(100);
		//EN reset
		HAL_GPIO_WritePin(CTRL_EN_GPIO_Port, CTRL_EN_Pin, GPIO_PIN_SET);				// EN High
		
		
		/*SCS signal is used to reset communication peripheral and SYN signal is used to reset the DSP. 
			These reset pulses must be generated once, just after the power-on sequence. */
			
		//Startup reset
		HAL_Delay(t_startup);
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_RESET);
		HAL_Delay(t_rpw/2);
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_SET);
		HAL_Delay(t_rpw/2);
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_RESET);
		HAL_Delay(t_rpw/2);
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_SET);
		HAL_Delay(t_rpw/2);
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_RESET);
		HAL_Delay(t_rpw/2);
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_SET);
		
		HAL_Delay(t_scs);
		HAL_GPIO_WritePin(CTRL_SCS_GPIO_Port, CTRL_SCS_Pin, GPIO_PIN_RESET);
		HAL_Delay(t_rpw/2);
		HAL_GPIO_WritePin(CTRL_SCS_GPIO_Port, CTRL_SCS_Pin, GPIO_PIN_SET);
		// All hardware reset is done
		
		
		//Configure DSP_CR3, read address 0x04 (Row2), write 0xABCD to 0x05 
		uint8_t SentMsg [2] = {0};
		
		SentMsg[0] = 0x03;
		SentMsg[1] = 0x80;
		
		//for (int i = 0; i < 2; i++){
			SendMsgOnly (dsp_cr3 + 0x01, SentMsg);
		//}

		USART3_PINSET_TX();
		myprintf("INIT done\r\n");
		USART3_PINSET_RX();
		return true;
}

bool SendMsgOnly (uint32_t SendAddress, uint8_t* SendMessage){
	/* 
		| ReadAddress | WriteAddress | LS Data [7:0] | MS Data [15:8] | CRC Byte |
		|		 0xFF 		|		 Address	 | 	 Message[0]  |   Message[1]   |    --    |
	*/
	uint8_t ReadMessage[5] = {0};
	uint8_t Buffer[5] = {0};
	uint8_t CRCBuffer[5] = {0};
	
	Buffer[0] = 0x00; //This will automatically increment read pointer by 1
	Buffer[1] = SendAddress;
	Buffer[2] = SendMessage[1];
	Buffer[3] = SendMessage[0];
	
	CRCBuffer[0] = byteReverse(Buffer[0]);
	CRCBuffer[1] = byteReverse(Buffer[1]);
	CRCBuffer[2] = byteReverse(Buffer[2]);
	CRCBuffer[3] = byteReverse(Buffer[3]);
	Buffer[4] = byteReverse(CalcCRC8(CRCBuffer));
	
	if (USART1_RxFlag == 0){
		HAL_UART_Receive_IT(&huart1, (uint8_t*) ReadMessage, 5);
	}
	
	HAL_UART_Transmit(&huart1, (uint8_t*) Buffer, 5, 0xFFFF);

	HAL_Delay(1);
	
	return true;
}

bool ReadMsgOnly (uint32_t ReadAddress, uint8_t* ReadMessage){
	/* 
		| ReadAddress | WriteAddress | LS Data [7:0] | MS Data [15:8] | CRC Byte |
		|		 0xFF 		|		 Address	 | 	 Message[0]  |   Message[1]   |    --    |
	*/
	uint8_t Buffer[5] = {0};
	uint8_t CRCBuffer[5] = {0};
	
	Buffer[0] = ReadAddress;
	Buffer[1] = 0xFF;
	Buffer[2] = 0xFF;
	Buffer[3] = 0xFF;
	
	CRCBuffer[0] = byteReverse(Buffer[0]);
	CRCBuffer[1] = byteReverse(Buffer[1]);
	CRCBuffer[2] = byteReverse(Buffer[2]);
	CRCBuffer[3] = byteReverse(Buffer[3]);
	Buffer[4] = byteReverse(CalcCRC8(CRCBuffer));
	
	
	if (USART1_RxFlag == 0){
		HAL_UART_Receive_IT(&huart1, (uint8_t*) ReadMessage, 5);
	}
	
	HAL_UART_Transmit(&huart1, (uint8_t*) Buffer, 5, 0xFFFF);
	
	HAL_Delay(1);	//Delay for 1 ms between each transmit to avoid any loss of data.
	
	return true;
}
















// bool SendMessage(uint32_t ReadAddress, uint8_t* ReadMessage ,uint32_t SendAddress, uint8_t* SendMessage) {
// 	/* 
// 		| ReadAddress | WriteAddress | LS Data [7:0] | MS Data [15:8] | CRC Byte |
// 		|		 0xFF 		|		 Address	 | 	 Message[0]  |   Message[1]   |    --    |
// 	*/
// 	uint8_t Buffer[5] = {0};
// 	uint8_t CRCBuffer[5] = {0};
	
// 	Buffer[0] = ReadAddress;
// 	Buffer[1] = SendAddress;
// 	Buffer[2] = SendMessage[0];
// 	Buffer[3] = SendMessage[1];
		
// 	CRCBuffer[0] = byteReverse(Buffer[0]);
// 	CRCBuffer[1] = byteReverse(Buffer[1]);
// 	CRCBuffer[2] = byteReverse(Buffer[2]);
// 	CRCBuffer[3] = byteReverse(Buffer[3]);
// 	Buffer[4] = byteReverse(CalcCRC8(CRCBuffer));
	
// 	HAL_UART_Receive_IT(&huart1, (uint8_t*) ReadMessage, 5);
	
// 	HAL_UART_Transmit(&huart1, (uint8_t*)Buffer, 5,0xFFFF);
	
// 	//HAL_UART_Receive_IT(&huart1, (uint8_t*) ReadMessage, 5);
	
	
// 	USART3_PINSET_TX();
// 	//myprintf("ReadMessage: %x | %x | %x | %x | %x  \r\n",ReadMessage[0], ReadMessage[1], ReadMessage[2], ReadMessage[3], ReadMessage[4]);
// 	myprintf("Init register configured");
// 	myprintf("\r\n");
// 	USART3_PINSET_RX();
	 
	
// 	return true;
// }



















































/*===================================================================== */
/*					CRC Calc 																										*/
/*===================================================================== */

static u8 CalcCRC8(u8 *pBuf)
{
	u8 i;
	CRC_u8Checksum = 0x00;
	for (i=0; i<STPM3x_FRAME_LEN-1; i++)
	{
	Crc8Calc(pBuf[i]);
	}
	return CRC_u8Checksum;
}

static void Crc8Calc (u8 u8Data)
{
	u8 loc_u8Idx;
	u8 loc_u8Temp;
	loc_u8Idx=0;
	while(loc_u8Idx<8)
	{
		loc_u8Temp = u8Data^CRC_u8Checksum;
		CRC_u8Checksum<<=1;
		if(loc_u8Temp&0x80)
		{
		CRC_u8Checksum^=CRC_8;
		}
		u8Data<<=1;
		loc_u8Idx++;
	}
}

void FRAME_for_UART_mode(u8 *pBuf)
{
u8 temp[4],x,CRC_on_reversed_buf;
for (x=0;x<(STPM3x_FRAME_LEN-1);x++)
{
temp[x] = byteReverse(pBuf[x]);
}
CRC_on_reversed_buf = CalcCRC8(temp);
pBuf[4] = byteReverse(CRC_on_reversed_buf);
}

static u8 byteReverse(u8 n)
{
n = ((n >> 1) & 0x55) | ((n << 1) & 0xaa);
n = ((n >> 2) & 0x33) | ((n << 2) & 0xcc);
n = ((n >> 4) & 0x0F) | ((n << 4) & 0xF0);
return n;
}
