/*
	Redundant Code that was used in the past


*/

#include "main.h"

//__HAL_UART_FLUSH_DRREGISTER(&huart3);

//if (huart1.Instance == USART1){

//vTaskSuspend(USART1Handle);


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
