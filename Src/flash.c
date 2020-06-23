/*
 * \addtogroup flash
 * \file flash.c
 * \brief External flash driver via SPI protocal
*/

/* Standard Includes */
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>		// To include uintXX_t type
#include <stdbool.h>	// To include the bool type

/* Include Project Headers */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "main.h"
#include "flash.h"
#include "ext_flash.h"
#include "usart.h"
#include "spi.h"

/* Commands supported by external Flash MX25L1606E */
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

/* Flash page size of MX25L1606E */
#define MX_PAGE_SIZE    256U

/* Flash Recognition */
#define JEDEC_ID_LEN    3U
#define MANUFACTURER_MICRON     0x20
#define MANUFACTURER_MXIC     	0xC2
#define FLASH_USART	USART3	//TO ADAPT WITH YOUR BOARD

/* Testing LOG */
#define LOGLEVEL LOGINFO

/* NOR Flash definition by JEDEC ID */
typedef struct {
	uint8_t		u8_jedec_id[JEDEC_ID_LEN];
	uint32_t	u32_nb_sectors_64k;
	char 		*name;
} s_Def_flash_nor;

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

/*============================================================================*/
/*                   STATIC VARIABLES DECLARATIONS                            */
/*============================================================================*/
static uint32_t	u32_ext_flash_size = 0U;
static bool ext_flash_is_enable = false;

static SemaphoreHandle_t xMutex;


/* Public functions */
//void ext_flash_init(void){
//	xMutex = xSemaphoreCreateRecursiveMutex();
//	assert_param(xMutex);
//}




