/**
 * \addtogroup Redundant Code
 * @{
 * \file RedundantCode.c
 * \author Hang Bobby Zou
 * \brief Redundant Code Library
 */

/*============================================================================*/
/*										Explanations of RedundantCode.c													*/			
/*----------------------------------------------------------------------------*/
/*	This C file contains redundant Code that was used in the past							*/
/*	It is intended to save a copy as these are mostly test files							*/
/*	It also contains function prototypes that are easily forgotten						*/
/*============================================================================*/

//#include "main.h"
#include <stdlib.h>

//__HAL_UART_FLUSH_DRREGISTER(&huart3);

//HAL_NVIC_DisableIRQ(USART3_IRQn);

//if (huart1.Instance == USART1){

//vTaskSuspend(USART1Handle);
//vTaskResume(USART1Handle);

//vTaskPrioritySet( USART3Handle, ( USART1_Priority - 2 ) );


//This belongs to USART1
// // Save read data to ext_flash
// 			if (flash_count > 4096){
// 				if (flash_sector_pointer == 512)
// 					flash_sector_pointer = 0;
				
// 				ext_flash_write(flash_sector_pointer, (char*) flash_buffer, 4096);
// 				ext_flash_last_write_or_erase_done();
// 				flash_sector_pointer ++;
// 				flash_count = 0;
// 			}
			
// 			flash_buffer[0] 						= 0xFF;
			
// 			flash_buffer[1] 						= i[0];
// 			flash_buffer[2] 						= 0xFF;
// 			flash_buffer[flash_count+3] = RxBuffer[0];
// 			flash_buffer[flash_count+4] = RxBuffer[1];
// 			flash_buffer[flash_count+5] = RxBuffer[2];
// 			flash_buffer[flash_count+6] = RxBuffer[3];
// 			flash_buffer[flash_count+7] = RxBuffer[4];
			
// 			flash_count += 0x08;




// USART1 Pervious test code
// //This is for testing, let the program repeatedly read from STPM32
		// if (i[0] > 0x28){
		// 	i[0] = 0x00;
		// }
		
		// if (USART1_RxFlag == 1){
		//  	//***This exceutes when a Receive is complete***
		// 	// Get the info before been overwritten
			
			
			
		// 	//***Note: Somehow, CRC byte always comes one cycle late, but normally we ignore it.
		// 	USART3_PINSET_TX();
		//  	myprintf("Received! Read Address: %x | ReadBuffer: %x | %x | %x | %x | %x  \r\n",i[0], RxBuffer[0], RxBuffer[1], RxBuffer[2], RxBuffer[3], RxBuffer[4]);
		//  	USART3_PINSET_RX();
			
		// 	// Increment the read register by 2 and clear the flag to wait for the next operation.
		// 	i[0] += 0x02;
		// 	USART1_RxFlag = 0;
		// }
		
		// Calls read message only and get the buffer as soon as it returns
		// ReadMsgOnly(i[0],ReadBuffer);
		// Test End here



// if (i[0] == dsp_reg14){
// 				myprintf("Copying CH1_RMS\r\n");
// 				uint8_cpy(CH1_RMS,RxBuffer,5);
// 			} else if (i[0] == ph1_reg1){
// 				myprintf("Copying PH1_Active_Energy\r\n");
// 				uint8_cpy(PH1_Active_Energy, RxBuffer, 5);
// 			} else if (i[0] == ph1_reg2){
// 				myprintf("Copying PH1_Fundamental_Energy\r\n");
// 				uint8_cpy(PH1_Fundamental_Energy, RxBuffer, 5);
// 			} else if (i[0] == ph1_reg3){
// 				myprintf("Copying PH1_Reactive_Energy\r\n");
// 				uint8_cpy(PH1_Reactive_Energy, RxBuffer, 5);
// 			} else if (i[0] == ph1_reg4){
// 				myprintf("Copying PH1_Apparent_Energy\r\n");
// 				uint8_cpy(PH1_Apparent_Energy, RxBuffer,5);
// 			} else if (i[0] == ph1_reg5){
// 				myprintf("Copying PH1_Active_Power\r\n");
// 				uint8_cpy(PH1_Active_Power, RxBuffer, 5);
// 			} else if (i[0] == ph1_reg6){
// 				myprintf("Copying PH1_Fundamental_Power\r\n");
// 				uint8_cpy(PH1_Fundamental_Power, RxBuffer, 5);
// 			} else if (i[0] == ph1_reg7){
// 				myprintf("Copying PH1_Reactive_Power\r\n");
// 				uint8_cpy(PH1_Reactive_Power, RxBuffer, 5);
// 			} else if (i[0] == ph1_reg8){
// 				myprintf("Copying PH1_Apparent_RMS_Power\r\n");
// 				uint8_cpy(PH1_Apparent_RMS_Power, RxBuffer, 5);
// 			} else if (i[0] == tot_reg1){
// 				myprintf("Copying Total_Active_Energy\r\n");
// 				uint8_cpy(Total_Active_Energy, RxBuffer, 5);
// 			} else if (i[0] == tot_reg2){
// 				myprintf("Copying Total_Fundamental_Energy\r\n");
// 				uint8_cpy(Total_Fundamental_Energy, RxBuffer, 5);
// 			} else if (i[0] == tot_reg3){
// 				myprintf("Copying Total_Reactive_Energy\r\n");
// 				uint8_cpy(Total_Reactive_Energy, RxBuffer, 5);
// 			} else if (i[0] == tot_reg4){
// 				myprintf("Copying Total_Apparent_Energy\r\n");
// 				uint8_cpy(Total_Apparent_Energy, RxBuffer, 5);
// 			}


// bool Read_Reg(uint32_t ReadAddress, uint8_t* ReturnBuffer){
	
// 	uint8_t ReadBuffer[5] = {0};	
	
// 	while(1){
// 		if (USART1_RxFlag == 1){
// 			ReturnBuffer[0] = ReadBuffer[0];
// 			ReturnBuffer[1] = ReadBuffer[1];
// 			ReturnBuffer[2] = ReadBuffer[2];
// 			ReturnBuffer[3] = ReadBuffer[3];
// 			ReturnBuffer[4] = ReadBuffer[4];
			
// 			USART3_PINSET_TX();
// 			myprintf("Address: %x \r\nData: %x | %x | %x | %x | %x  \r\n",ReadAddress , ReturnBuffer[0], ReturnBuffer[1], ReturnBuffer[2], ReturnBuffer[3], ReturnBuffer[4]);
// 			USART3_PINSET_RX();
			
// 			USART1_RxFlag = 0;
// 			return true;
// 		}

// 		ReadMsgOnly(ReadAddress,ReadBuffer);
		
		
// 	}
	
// }





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



// USART3
// USART3_PINSET_TX();
			// 	if (aRxBuffer[0] == dsp_reg14){
			// 		myprintf("Reading: CH1_RMS\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", CH1_RMS[0], CH1_RMS[1], CH1_RMS[2], CH1_RMS[3], CH1_RMS[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg1){
			// 		myprintf("Reading: PH1_Active_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Active_Energy[0],PH1_Active_Energy[1],PH1_Active_Energy[2],PH1_Active_Energy[3],PH1_Active_Energy[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg2){
			// 		myprintf("Reading: PH1_Fundamental_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Fundamental_Energy[0], PH1_Fundamental_Energy[1], PH1_Fundamental_Energy[2], PH1_Fundamental_Energy[3], PH1_Fundamental_Energy[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg3){
			// 		myprintf("Reading: PH1_Reactive_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Reactive_Energy[0], PH1_Reactive_Energy[1], PH1_Reactive_Energy[2], PH1_Reactive_Energy[3], PH1_Reactive_Energy[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg4){
			// 		myprintf("Reading: PH1_Apparent_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Apparent_Energy[0], PH1_Apparent_Energy[1], PH1_Apparent_Energy[2], PH1_Apparent_Energy[3], PH1_Apparent_Energy[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg5){
			// 		myprintf("Reading: PH1_Active_Power\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Active_Power[0], PH1_Active_Power[1], PH1_Active_Power[2], PH1_Active_Power[3], PH1_Active_Power[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg6){
			// 		myprintf("Reading: PH1_Fundamental_Power\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Fundamental_Power[0], PH1_Fundamental_Power[1], PH1_Fundamental_Power[2], PH1_Fundamental_Power[3], PH1_Fundamental_Power[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg7){
			// 		myprintf("Reading: PH1_Reactive_Power\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Reactive_Power[0], PH1_Reactive_Power[1], PH1_Reactive_Power[2], PH1_Reactive_Power[3], PH1_Reactive_Power[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg8){
			// 		myprintf("Reading: PH1_Apparent_RMS_Power\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Apparent_RMS_Power[0], PH1_Apparent_RMS_Power[1], PH1_Apparent_RMS_Power[2], PH1_Apparent_RMS_Power[3], PH1_Apparent_RMS_Power[4]);
			// 	} else if (aRxBuffer[0] == tot_reg1){
			// 		myprintf("Reading: Total_Active_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", Total_Active_Energy[0], Total_Active_Energy[1], Total_Active_Energy[2], Total_Active_Energy[3], Total_Active_Energy[4]);
			// 	} else if (aRxBuffer[0] == tot_reg2){
			// 		myprintf("Reading: Total_Fundamental_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", Total_Fundamental_Energy[0], Total_Fundamental_Energy[1], Total_Fundamental_Energy[2], Total_Fundamental_Energy[3], Total_Fundamental_Energy[4]);
			// 	} else if (aRxBuffer[0] == tot_reg3){
			// 		myprintf("Reading: Total_Reactive_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", Total_Reactive_Energy[0], Total_Reactive_Energy[1], Total_Reactive_Energy[2], Total_Reactive_Energy[3], Total_Reactive_Energy[4]);
			// 	} else if (aRxBuffer[0] == tot_reg4){
			// 		myprintf("Reading: Total_Apparent_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", Total_Apparent_Energy[0], Total_Apparent_Energy[1], Total_Apparent_Energy[2], Total_Apparent_Energy[3], Total_Apparent_Energy[4]);
			// 	} else {
			// 		myprintf(" Not a valid address \r\n");
			// 	}
			// USART3_PINSET_RX();



//USART3_PINSET_TX();
		 	//myprintf("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], RxBuffer[0], RxBuffer[1], RxBuffer[2], RxBuffer[3], RxBuffer[4]);
		 	//USART3_PINSET_RX();





/*============================================================================*/
/*                   END OF FILE                                              */
/*============================================================================*/
