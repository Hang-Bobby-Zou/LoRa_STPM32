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

#define ReadWriteSize 1024	//In bytes max
#define SectorNum			512		//In numbers
#define BlockNum			31		//In numbers
#define TestSize			3			//Defined in Sectors, change this to change the number of sectors to be tested, max 512

//Function prototypes
bool Randomize(char str[],uint32_t num,uint32_t param);
void WriteSector_tb(uint32_t address);
bool ReadWrite_tb(uint32_t address);
bool EraseSector_tb(uint32_t address);
bool EraseBlock_tb(uint32_t address);

/**
  * @brief  Test bench for external flash
	* @intval	NONE
  * @retval bool
  */
bool ext_flash_tb(void){
		
		uint32_t AddressMap[SectorNum] = {0};
		for (int i = 0; i < SectorNum; i++){
				AddressMap[i] = i * 4096; 	//i * 0x001000
		}
		
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
		for (int i = 0; i < TestSize; i++){
			if (ReadWrite_tb(AddressMap[i]) != true)
				return false;
			
		HAL_Delay(1);
		}
		

/*======================================================================*/
/*		Erase sector Test																									*/
/*======================================================================*/
		for (int i = 0; i < TestSize; i++){
			if (EraseSector_tb(AddressMap[i]) != true)
				return false;
			
		HAL_Delay(1);
		}

/*======================================================================*/
/*		Erase block Test																									*/
/*======================================================================*/
		for (int i = 0; i< TestSize; i++){
				WriteSector_tb(AddressMap[i]);
		}
		
		for (int i = 0; i< TestSize; i += 16){
				if (EraseBlock_tb(AddressMap[i]) != true)
					return false;
				
		HAL_Delay(1);
		}
		

/*======================================================================*/
/*		Finish Judge																											*/
/*======================================================================*/
		
		//If all test passes, return true
		return true;
		
}


//
//	Testbench functions, ending with _tb
//

/**
  * @brief  Write to a specific sector
	* @intval	Sector Address
  * @retval NONE
  */
void WriteSector_tb(uint32_t address){
		char RandomString 	[ReadWriteSize];
	
		Randomize(RandomString,ReadWriteSize,0);
				
		ext_flash_erase_sector(address);
		ext_flash_last_write_or_erase_done();			// Wait for erase to be done
		//ext_flash_read(address, ReadData,ReadWriteSize);		// Read from address
				
		ext_flash_write(address,RandomString,ReadWriteSize);
		ext_flash_last_write_or_erase_done();			// Wait for write to be done
}

/**
  * @brief  Erase the whole block of data starting sector address
	*					and validate itself
	* @intval	Sector Address
  * @retval If successful, return true, else return false
  */
bool EraseBlock_tb(uint32_t address){
		char ReadData 			[ReadWriteSize];
		char EmptyData			[ReadWriteSize] = {0};
		
		for (int i = 0; i < ReadWriteSize; i++){
				EmptyData[i] = 0xFF;
		}
		
		ext_flash_erase_block(address);
		ext_flash_last_write_or_erase_done();
		
		for (int i = 0; i < 0x010000; i += 4096){
				ext_flash_read(address + i, ReadData,ReadWriteSize);
			
				if (memcmp(EmptyData, ReadData, ReadWriteSize) != 0){
					return false;
				}
		}
		return true;
}


/**
  * @brief  Erase the whole block of sector starting sector address
	*					and validate itself
	* @intval	Sector Address
  * @retval If successful, return true, else return false
  */
bool EraseSector_tb(uint32_t address){
		char ReadData 			[ReadWriteSize];
		char EmptyData			[ReadWriteSize] = {0};
		
		for (int i = 0; i < ReadWriteSize; i++){
				EmptyData[i] = 0xFF;
		}
	
		ext_flash_erase_sector(address);
		ext_flash_last_write_or_erase_done();
		
		ext_flash_read(address, ReadData,ReadWriteSize);
		
		if (memcmp(EmptyData, ReadData, ReadWriteSize) != 0){
				return false;
		}
		
		return true;
}

/**
  * @brief  Do a sector erase, write and read sequence
	*					and validate itself
	* @intval	Sector Address
  * @retval If successful, return true, else return false
  */
bool ReadWrite_tb(uint32_t address){
		char ReadData				[ReadWriteSize];
		char RandomString 	[ReadWriteSize];
	
		Randomize(RandomString,ReadWriteSize,0);
				
		ext_flash_erase_sector(address);
		ext_flash_last_write_or_erase_done();			// Wait for erase to be done
		//ext_flash_read(address, ReadData,ReadWriteSize);		// Read from address
				
		ext_flash_write(address,RandomString,ReadWriteSize);
		ext_flash_last_write_or_erase_done();			// Wait for write to be done
				
		ext_flash_read(address, ReadData,ReadWriteSize);		// Read from address
				
		if (memcmp(RandomString, ReadData,ReadWriteSize) != 0){
				return false;
		}
		return true;
}

/**
  * @brief  Randomize a string
	* @intval	String pointer, number of elements and randomize variable param
  * @retval If successful, return true, else return false
  */
bool Randomize(char str[],uint32_t num, uint32_t param){
		srand((unsigned)param);
		
		for (int i = 0; i < num; i++){
				str[i] = (rand()% 122 + 48);
		}
		
		return true;
}
