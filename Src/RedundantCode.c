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


