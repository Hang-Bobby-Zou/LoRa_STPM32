/**
 * \addtogroup ext_flash
 * @{
 * \file ext_flash.c
 * \copyright 
 * \author Matthew wang
 * \adapted by Bobby in purpose of using flash with STM32L433 board
 * \brief External SPI flash driver source code.
 */

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
#include "HAL_spi.h"
#include "ext_flash_tb.h"
#include "tim.h"

/*============================================================================*/
/*                   MACRO DEFINITION                                         */
/*============================================================================*/
#define ARRAY_SIZE(tab)	(sizeof(tab)/sizeof(tab[0]))
#define ENABLE_FLASH 		1
#define DISABLE_FLASH 	0

/******************************************************************************/
/** \brief List of commands supported by the external Flash MX25L1606E
 *
 * MX_CMD_* provides all the existing commands supported by the external
 * Flash memory.
 *
 *  \sa    flash_ext_wr_rd that uses one such command as its first input parameter
 ******************************************************************************/
#define MX_CMD_WREN   "\x06" /*!< Write enable */
#define MX_CMD_WRDI   "\x04" /*!< Write disable */
#define MX_CMD_WRSR   "\x01" /*!< Write status register */
#define MX_CMD_RDID   "\x9F" /*!< Read identification */
#define MX_CMD_RDSR   "\x05" /*!< Read status register */
#define MX_CMD_READ   "\x03" /*!< Read data */
#define MX_CMD_F_READ "\x0B" /*!< Fast read data */
#define MX_CMD_RDSFDP "\x5A" /*!< Read SFDP */
#define MX_CMD_RES    "\xAB" /*!< Read electronic ID */
#define MX_CMD_DREAD  "\x3B" /*!< Double output mode command */
#define MX_CMD_SE     "\x20" /*!< Sector erase */
#define MX_CMD_BE     "\xD8" /*!< Block erase *///2019-09-27 matthew: "\x52" - 32k block erase, "\xD8" - 64k block erase
#define MX_CMD_CE     "\x60" /*!< Chip erase */
#define MX_CMD_PP     "\x02" /*!< Page program */
#define MX_CMD_RDSCUR "\x2B" /*!< Read security register */
#define MX_CMD_WRSCUR "\x2F" /*!< Write security register */
#define MX_CMD_ENSO   "\xB1" /*!< Enter secured OTP */
#define MX_CMD_EXSO   "\xC1" /*!< Exit secured OTP */
#define MX_CMD_DP     "\xB9" /*!< Deep power down */
#define MX_CMD_RDP    "\xAB" /*!< Release from deep power down mode */

/******************************************************************************/
/** \brief External Flash MX25L1606E page size, 256 bytes
 ******************************************************************************/
#define MX_PAGE_SIZE	256U

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

/******************************************************************************/
/** \brief Prepare the external Flash for a write or erase operation
 ******************************************************************************/
#define PrepareWrite()								\
do {												\
	ext_flash_last_write_or_erase_done();			\
	flash_ext_wr_rd(MX_CMD_WREN, 1, NULL, 0);		\
} while (0)

/******************************************************************************/
/** \brief Define value for flash recognition
 ******************************************************************************/
#define	JEDEC_ID_LEN	3U
#define MANUFACTURER_MICRON     0x20
#define MANUFACTURER_MXIC     	0xC2

// Configure info name "LOGELEVEL"

/*============================================================================*/
/*                   TYPE DEFINITION                                          */
/*============================================================================*/
/******************************************************************************/
/** \brief A NOR flash definition by JEDEC ID
		Used to id flash part number
 ******************************************************************************/
typedef struct {
	uint8_t		u8_jedec_id[JEDEC_ID_LEN];
	uint32_t	u32_nb_sectors_64k;
	char 		*name;
} s_Def_flash_nor;

/*============================================================================*/
/*                   PUBLIC VARIABLES DEFINITION                              */
/*============================================================================*/

/*============================================================================*/
/*                   STATIC FUNCTIONS DECLARATIONS                            */
/*============================================================================*/
/******************************************************************************/
/** \brief Internal read/write common code of any external Flash access
 *
 *  This function centralizes every SPI access to the external Flash by
 *  providing a write buffer + its size followed by a read buffer + its size.
 *  The write part is mandatory starting by the command ID, and followed by
 *  a variable length array (size depending on command and user need).
 *  The read part is optional, depending on the kind of command. Its size also
 *  depends on the kind of command and the user request.
 *
 *  flash_ext_wr_rd has also been split in two parts for write-only cycles :
 *  - 1: flash_ext_begin_wr performs the initial write
 *  - 2: flash_ext_end_wr performs the ending write
 *  The reason of this split is to avoid allocating and copying the source
 *  buffer to reduce the RAM usage.
 *
 * @param[in] wr_buf  Write buffer pointer (read-only)
 * @param[in] wr_size Size of the write buffer
 * @param[out] rd_buf  Read buffer pointer to store the result (NULL if N/A)
 * @param[out] rd_size Size of the read buffer
 *
 *  \sa    MX_CMD_* as low-level command
******************************************************************************/
static void flash_ext_wr_rd(const char *wr_buf, uint16_t wr_size, char *rd_buf, uint16_t rd_size);
static void flash_ext_begin_wr(const char *wr_buf, int32_t wr_size);
static void flash_ext_end_wr(const char *wr_buf, int32_t wr_size);
static void vd_ext_flash_reset(void);
static int32_t ext_flash_read_id(void);

// Debug commands
//static void vd_lib_flash_status(char *s8p_Commande, int32_t s32_Param);
//static void vd_lib_flash_dump(char *s8p_Commande, int32_t s32_Param);
//static void vd_lib_flash_erase_sector(char *s8p_Commande, int32_t s32_Param);

/*============================================================================*/
/*                   STATIC VARIABLES DECLARATIONS                            */
/*============================================================================*/
static uint32_t	u32_ext_flash_size = 0U;
static bool ext_flash_is_enable = false;

static SemaphoreHandle_t xMutex;

/*============================================================================*/
/*                   PUBLIC FUNCTIONS DEFINITIONS                             */
/*============================================================================*/

//==============================================================================
//	External Flash Initialization
//==============================================================================
void ext_flash_init(void)
{
	xMutex = xSemaphoreCreateRecursiveMutex();
	assert_param(xMutex);
}

//==============================================================================
//	External Flash Deinitializaiton
//==============================================================================
void ext_flash_deinit(void)
{
	ext_flash_deep_powerdown();

	ext_flash_power_off();

	vSemaphoreDelete(xMutex);
	//TODO: should be adapted
    // GPIO are handled by drv_gpio
}

//==============================================================================
//	Get External Flash size
//==============================================================================
uint32_t ext_flash_Get_Size(void)
{
	return(u32_ext_flash_size);
}

//==============================================================================
//	Reset External Flash
//==============================================================================
static void vd_ext_flash_reset(void)
{
	// wait 10µs from last deselect
	//vTaskDelay(pdMS_TO_TICKS( 10 ));
	//HAL_Delay(1);
	DelayMsPoll(1);
	
	// pulse of 10µs
	HAL_GPIO_WritePin(GPIOA, CS_FLASH_Pin, GPIO_PIN_SET);
	//vTaskDelay(pdMS_TO_TICKS( 10 ));
	//HAL_Delay(1);
	DelayMsPoll(1);
	
	HAL_GPIO_WritePin(GPIOA, CS_FLASH_Pin, GPIO_PIN_RESET);
	
	// reset recovery, wait 30µs
	//vTaskDelay(pdMS_TO_TICKS( 30 ));
	//HAL_Delay(3);
	DelayMsPoll(3);
}


//==============================================================================
//	Read ID and Size of External Flash
//==============================================================================
static int32_t ext_flash_read_id(void)
{
	/******************************************************************************/
	/** \brief List of recognized Flash NOR
	 ******************************************************************************/
	const s_Def_flash_nor ts_nor_id[]= {
		{ {MANUFACTURER_MICRON, 0x80,	0x14},	16,	"m25pe80"},
		{ {MANUFACTURER_MICRON, 0x80,	0x15},	32,	"m25pe16"},
		{ {MANUFACTURER_MXIC, 	0x20,	0x15},	32,	"MX25L1606E"},
		{ {MANUFACTURER_MXIC, 	0x20,	0x17},	128,"MX25L6433F"},
	};

	// Read buffer
	char rd_buf[JEDEC_ID_LEN];

	// Read JEDEC Identification, Comman : MX_CMD_RDID
	flash_ext_wr_rd(MX_CMD_RDID, 1, rd_buf, sizeof(rd_buf));
	
	int16_t i16_found= -1;
	for(uint16_t i=0U; i<ARRAY_SIZE(ts_nor_id); i++)
	{
		if (!memcmp(rd_buf, ts_nor_id[i].u8_jedec_id, JEDEC_ID_LEN))
		{
			i16_found= (int16_t)i;
		}
	}

	if (i16_found >= 0)
	{
		u32_ext_flash_size= ts_nor_id[i16_found].u32_nb_sectors_64k << 16;
		INFO("Flash detected \"%s\", size is %lu bytes", ts_nor_id[i16_found].name, u32_ext_flash_size);
		//myprintf("Flash detected \"%s\", size is %lu bytes\n", ts_nor_id[i16_found].name, u32_ext_flash_size);
	}
	else
	{
		u32_ext_flash_size = 0;
		INFO("Flash not recognized or not detected (id %02x %02x %02x)", rd_buf[0], rd_buf[1], rd_buf[2]);
		//myprintf("Flash not recognized or not detected (id %02x %02x %02x)\n", rd_buf[0], rd_buf[1], rd_buf[2]);
	}

	return(u32_ext_flash_size ? 1 : 0);
}

//==============================================================================
//	Detect External Flash
//==============================================================================
#define	EXT_FLASH_NB_RESET	2	//<! max number of reset before giving up
int32_t ext_flash_is_detected(void)
{
	if (ext_flash_is_enable == false)		//If flash is not enabled, enable it
		ext_flash_power_on();

	int32_t s32_cpt = 0;
	int32_t s32_ret = 0;
	do
	{
		// reset the flash
		vd_ext_flash_reset();

		// then try to read the ID
		s32_ret = ext_flash_read_id();

		s32_cpt++;
	} while ((s32_ret == 0) && (s32_cpt < EXT_FLASH_NB_RESET));		//Try 2 times before quit

	return(s32_ret);
}

//==============================================================================
//	Erase Sector of External Flash
//==============================================================================
void ext_flash_erase_sector(uint32_t FLASH_Sector)
{
	if (xSemaphoreTakeRecursive(xMutex, 1000) == pdTRUE)
	{
		if (ext_flash_is_enable == false)
			ext_flash_power_on();

		PrepareWrite();
		uint8_t cmd[4] = MX_CMD_SE;
		cmd[1] = (FLASH_Sector >> 16) & 0xFFU;
		cmd[2] = (FLASH_Sector >>  8) & 0xFFU;
		cmd[3] = (FLASH_Sector >>  0) & 0xFFU;
		flash_ext_wr_rd((char *)cmd, 4, NULL, 0);
		xSemaphoreGiveRecursive(xMutex);
	}
}

//==============================================================================
//	Erase Blcok of External Flash
//==============================================================================
void ext_flash_erase_block(uint32_t FLASH_Block)
{
	if (xSemaphoreTakeRecursive(xMutex, 1000) == pdTRUE)
	{
		if (ext_flash_is_enable == false)
			ext_flash_power_on();

		PrepareWrite();
		uint8_t cmd[4] = MX_CMD_BE;
		cmd[1] = (FLASH_Block >> 16) & 0xFFU;
		cmd[2] = (FLASH_Block >>  8) & 0xFFU;
		cmd[3] = (FLASH_Block >>  0) & 0xFFU;
		flash_ext_wr_rd((char *)cmd, 4, NULL, 0);
		xSemaphoreGiveRecursive(xMutex);
	}
}

//==============================================================================
//	Write to specific address
//==============================================================================
void ext_flash_write(uint32_t address, const char *data, uint32_t nb_byte)
{
	if (xSemaphoreTakeRecursive(xMutex, 1000) == pdTRUE)
	{
		if (ext_flash_is_enable == false)
			ext_flash_power_on();

		const char *ptr = data;
		while (nb_byte > 0U) {
			// Ready to program the current page...
			uint8_t cmd[4] = MX_CMD_PP;
			PrepareWrite();
			cmd[1] = (address >> 16) & 0xFFU;
			cmd[2] = (address >>  8) & 0xFFU;
			cmd[3] = (address >>  0) & 0xFFU;
			flash_ext_begin_wr((char *)cmd, 4);
			uint32_t wr_len = nb_byte;
			if (wr_len > (MX_PAGE_SIZE - address % MX_PAGE_SIZE))
				wr_len = (MX_PAGE_SIZE - address % MX_PAGE_SIZE);
			flash_ext_end_wr(ptr, (int32_t)wr_len);
			// Prepare the next iteration
			nb_byte -= wr_len;
			ptr += wr_len;
			address += wr_len;
		}
		xSemaphoreGiveRecursive(xMutex);
	}
}

//==============================================================================
//	Wait for write or erase done
//==============================================================================
void ext_flash_last_write_or_erase_done(void)
{
	if (ext_flash_is_enable == false)
		ext_flash_power_on();

	uint8_t rd_buf[1];
	do {
		flash_ext_wr_rd(MX_CMD_RDSR, 1, (char *)rd_buf, 1);
	} while ((rd_buf[0] & 0x1U) != 0x00U);    // WIP=0
}

//==============================================================================
//	Read from specific address
//==============================================================================
void ext_flash_read(uint32_t address, char *data, uint32_t nb_byte)
{
	if (xSemaphoreTakeRecursive(xMutex, 1000) == pdTRUE)
	{
		if (ext_flash_is_enable == false)
		ext_flash_power_on();

		char *ptr = data;
		while (nb_byte > 0U) {
			uint8_t cmd[4] = MX_CMD_READ;
			cmd[1] = (address >> 16) & 0xFFU;
			cmd[2] = (address >>  8) & 0xFFU;
			cmd[3] = (address >>  0) & 0xFFU;
			uint32_t tr_len = nb_byte;
			flash_ext_wr_rd((char *)cmd, 4, ptr, (int32_t)tr_len);	// TODO check return value
			// Prepare the next iteration
			nb_byte -= tr_len;
			ptr += tr_len;
			address += tr_len;
		}
		xSemaphoreGiveRecursive(xMutex);
	}
}

//==============================================================================
//	Turn External Flash into deep powerdown mode, saves 20mA
//==============================================================================
void ext_flash_deep_powerdown(void)
{
	flash_ext_wr_rd(MX_CMD_DP, 1, NULL, 0);
}

//==============================================================================
//	Wake External Flash from deep powerdown mode
//==============================================================================
void ext_flash_release_from_deep_powerdown(void)
{
	flash_ext_wr_rd(MX_CMD_RDP, 1, NULL, 0);
}

//==============================================================================
//	Switch on power of External Flash (reduntant)
//==============================================================================
void ext_flash_power_on(void)
{
  INFO("Switching Flash ON");

	HAL_GPIO_WritePin(GPIOA, CS_FLASH_Pin, GPIO_PIN_RESET);		// Pull down to enable
	
	ext_flash_is_enable = true;
}

//==============================================================================
//	Switch off power of External Flash (reduntant)
//==============================================================================
void ext_flash_power_off(void)
{
	//INFO("Switching Flash OFF");
	myprintf("Switching Flash OFF");
	
	//Be sure that flash has finished to write data
	ext_flash_last_write_or_erase_done();


	//Switching off VDD_FLASH
	HAL_GPIO_WritePin(GPIOA, CS_FLASH_Pin, GPIO_PIN_SET);		// Pull us to disable

	ext_flash_is_enable = false;
}


/*============================================================================*/
/*                   STATIC FUNCTIONS DEFINTIONS                              */
/*============================================================================*/

//==============================================================================
//						External Flash read and write all in one
//==============================================================================
static void flash_ext_wr_rd(const char *wr_buf, uint16_t wr_size, char *rd_buf, uint16_t rd_size)
{
	// Enable Flash, Hardware NSS output signal	
	HAL_GPIO_WritePin(GPIOA, CS_FLASH_Pin, GPIO_PIN_RESET);
	// Wait for 10 ms
	//vTaskDelay(pdMS_TO_TICKS( 10 ));
	//HAL_Delay(1);
	DelayMsPoll(1);
	
	// Transmit data
	if(HAL_SPI_Transmit(&hspi1, (uint8_t *)wr_buf, wr_size, 5) != HAL_OK){
		//ERROR("HAL_SPI_Transmit ERROR in flash_ext_wr_rd()!");
		//myprintf("HAL_SPI_Transmit ERROR in flash_ext_wr_rd()!\n");
	}

	// Read data
	if(HAL_SPI_Receive(&hspi1, (uint8_t *)rd_buf, rd_size, 5) != HAL_OK){
		//DEBUG("HAL_SPI_Receive NOTHING in flash_ext_wr_rd()!");
		//myprintf("\nHAL_SPI_Receive NOTHING in flash_ext_wr_rd()!\n");
	}

	// Disable Flash, Hardware NSS output signal
	HAL_GPIO_WritePin(GPIOA, CS_FLASH_Pin, GPIO_PIN_SET);
}

//==============================================================================
//						External Flash read begin
//==============================================================================
static void flash_ext_begin_wr(const char *wr_buf, int32_t wr_size)
{
	// Enable Flash, Hardware NSS output signal
	//HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CS_FLASH_GPIO_Port, CS_FLASH_Pin, GPIO_PIN_RESET);

	
	// Transmit data
	if(HAL_SPI_Transmit(&hspi1, (uint8_t *)wr_buf, wr_size, 5) != HAL_OK){
		//ERROR("HAL_SPI_Transmit ERROR in flash_ext_begin_wr()!");
		myprintf("HAL_SPI_Transmit ERROR in flash_ext_begin_wr()!");
	}
		
}

//==============================================================================
//						External Flash read write
//==============================================================================
static void flash_ext_end_wr(const char *wr_buf, int32_t wr_size)
{
	// Transmit data
	if(HAL_SPI_Transmit(&hspi1, (uint8_t *)wr_buf, wr_size, 5) != HAL_OK){
		//ERROR("HAL_SPI_Transmit ERROR in flash_ext_end_wr()!");
		myprintf("HAL_SPI_Transmit ERROR in flash_ext_end_wr()!");
	}
		

	// Disable Flash, Hardware NSS output signal
	//HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_FLASH_GPIO_Port, CS_FLASH_Pin, GPIO_PIN_SET);
}

/** @} */
/*============================================================================*/
/*                   END OF FILE                                              */
/*============================================================================*/
