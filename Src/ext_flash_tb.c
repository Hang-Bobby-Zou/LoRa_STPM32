/*============================================================================*/
/*                   STANDARD INCLUDE FILES                                   */
/*============================================================================*/
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>		// To include uintXX_t type
#include <stdbool.h>	// To include the bool type

/*============================================================================*/
/*                   INCLUDE FILES                                            */
/*============================================================================*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "main.h"
#include "ext_flash.h"
#include "usart.h"
#include "spi.h"
#include "ext_flash_tb.h"

#define TestDataSize 128



/**
  * @brief  Test bench for external flash
  * @retval bool
  */
bool ext_flash_tb(void){
		
		uint32_t IncorrectFlag = 0;
		
		char WriteData[] = "Test Flash";
		char ReadData[TestDataSize];
	
		//==========================================================================
		//		Detection Test
		//==========================================================================
		// If flash is not detected, then goes to Error_Handler
		if (ext_flash_is_detected() != 1){
				IncorrectFlag = 1;
		}
		
		//==========================================================================
		//		ReadWrite Test
		//==========================================================================
		ext_flash_write(0x00,WriteData,sizeof(WriteData));
		ext_flash_last_write_or_erase_done();
		
		ext_flash_read(0x00, ReadData,128);
		
		if (memcmp(WriteData, ReadData, sizeof(WriteData)) == 1){
				IncorrectFlag = 1;
		}
		
		//==========================================================================
		//		Erase sector Test
		//==========================================================================
		ext_flash_write(0x00,WriteData,sizeof(WriteData));
		ext_flash_last_write_or_erase_done();
		
		ext_flash_read(0x00, ReadData,128);
		
		ext_flash_erase_sector(0);
		ext_flash_last_write_or_erase_done();
		
		ext_flash_read(0x00, ReadData,128);
		if (memcmp("          ", ReadData, sizeof(WriteData)) == 1){
				IncorrectFlag = 1;
		}
		
		//==========================================================================
		//		Erase block Test
		//==========================================================================
		ext_flash_write(0x00,WriteData,sizeof(WriteData));
		ext_flash_last_write_or_erase_done();
		
		ext_flash_read(0x00, ReadData,128);
		
		ext_flash_erase_block(0);
		ext_flash_last_write_or_erase_done();
		
		ext_flash_read(0x00, ReadData,128);
		if (memcmp("          ", ReadData, sizeof(WriteData)) == 1){
				IncorrectFlag = 1;
		}
		

		// If all test passes, IncorrectFlag is set to be zero, outputing true.
		if (IncorrectFlag == 1){
				return false; 
		} else {
				return true;
		}
		
		
		
}
