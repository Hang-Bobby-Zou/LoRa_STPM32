// Standard Includes
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>		// To include uintXX_t type
#include <stdbool.h>	// To include the bool type

// Include project headers
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "main.h"
#include "flash.h"
#include "ext_flash.h"
#include "usart.h"
#include "spi.h"

// Commands supported by external Flash MX25L1606E
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

//Static variables
//static SemaphoreHandle_t xMutex;


//Public functions
//void ext_flash_init(void){
//	xMutex = xSemaphoreCreateRecursiveMutex();
//	assert_param(xMutex);
//}




