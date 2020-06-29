/**
 * \addtogroup ext_flash_tb
 * @{
 * \file ext_flash_tb.c
 * \author Hang Bobby Zou
 * \brief External SPI flash testbench
 */
 
/******************************************************************************/
/*					External Flash Memory Organization											
					|-------------------------------------|
					| Block | Sector |    Address Range   |
					|-------------------------------------|
					|	  		|  511   |  1FF000h | 1FFFFFh |
					|	 31   |   :		 |     :    |    :    |
					|	  		|  496   |  1F0000h | 1F0FFFh |
					|-------------------------------------|
					|	  		|  495   |  1EF000h | 1EFFFFh |
					|	 30   |   :		 |     :    |    :    |
					|	  		|  480   |  1E0000h | 1E0FFFh |
					|-------------------------------------|
					|   :   |   :    |     :    |    :    |
					|   :   |   :    |     :    |    :    |
					|-------------------------------------|
					|	  		|   15   |  00F000h | 00FFFFh |
					|	      |   :		 |     :    |    :    |
					|	  		|   3    |  003000h | 003FFFh |
					|	  0		|   2    |  002000h | 002FFFh |
					|	  		|   1    |  001000h | 001FFFh |
					|	  		|   0    |  000000h | 000FFFh |
					|-------------------------------------|
					// Block Size  	: 65536 bytes
					// Sector Size 	:	4096 bytes
					
					
*/
/******************************************************************************/
 
 
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
#include "ext_flash.h"
#include "usart.h"
#include "spi.h"
#include "ext_flash_tb.h"

#define TestDataSize 	128
#define SectorSize		4096
#define BlockSize			65536

bool Randomize(char str[],uint32_t num);

/**
  * @brief  Test bench for external flash
  * @retval bool
  */
bool ext_flash_tb(void){
		
		char WriteData			[128];
		char ReadData				[128];
		char RandomString 	[128];
/*======================================================================*/
/*		Detection Test																										*/
/*======================================================================*/
		// If flash is not detected, then goes to Error_Handler
		if (ext_flash_is_detected() != 1){
				return false;
		}
		
		
/*======================================================================*/
/*		ReadWrite Test																										*/
/*======================================================================*/
		
		Randomize(RandomString,128);
		strcpy(WriteData,RandomString);
		
		ext_flash_write(0x00,WriteData,sizeof(WriteData));
		ext_flash_last_write_or_erase_done();

		ext_flash_read(0x00, ReadData,128);
		
		if (memcmp(WriteData, ReadData, sizeof(WriteData)) == 1){
				return false;
		}
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		/*======================================================================*/
		/*		Erase sector Test																									*/
		/*======================================================================*/
		ext_flash_write(0x00,WriteData,sizeof(WriteData));
		ext_flash_last_write_or_erase_done();
		
		ext_flash_read(0x00, ReadData,128);
		
		ext_flash_erase_sector(0);
		ext_flash_last_write_or_erase_done();
		
		ext_flash_read(0x00, ReadData,128);
		if (memcmp("          ", ReadData, sizeof(WriteData)) == 1){
				return false;
		}
		
		/*======================================================================*/
		/*		Erase block Test																									*/
		/*======================================================================*/
		ext_flash_write(0x00,WriteData,sizeof(WriteData));
		ext_flash_last_write_or_erase_done();
		
		ext_flash_read(0x00, ReadData,128);
		
		ext_flash_erase_block(0);
		ext_flash_last_write_or_erase_done();
		
		ext_flash_read(0x00, ReadData,128);
		if (memcmp("          ", ReadData, sizeof(WriteData)) == 1){
				return false;
		}
		
		
		//If all test passes
		return true;
		
}

bool Randomize(char str[],uint32_t num){
		if (strlen(str) > num){
				return false;
		}
		
		
		for (int i = 0; i < num; i++){
				str[i] = (rand()% 122 + 48);
		}
		
		return true;
}
