/**
 * \addtogroup ext_flash
 * @{
 * \file ext_flash.h
 * \copyright 
 * \author Matthew wang
 * \brief External SPI flash driver source code.
 */
 
#ifndef EXT_FLASH_H
#define EXT_FLASH_H

/*============================================================================*/
/*                   PROJECT INCLUDE FILES                                    */
/*============================================================================*/
#include <stdint.h>
#include <stdbool.h>	// To include the bool type
//#include "acceler_ADXL355.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/*                   MACRO DEFINITION                                         */
/*============================================================================*/

/******************************************************************************/
/** \brief Internal Flash erase areas sizes
 ******************************************************************************/
#define EXT_FLASH_SECTOR_SIZE	0x1000U		/*!< Sector size is 4 KByte */
#define EXT_FLASH_BLOCK_SIZE	0x10000		/*!< Block size is 64 KByte */

#define FLASH_PARAMETERS_ADDR			0x0000	/* address of sector 0 */
#define FLASH_TEMPERATURES_ADDR			0x1000	/* address of sector 1 */
#define FLASH_BATTERY_LEVEL_ADDR		0x2000	/* address of sector 2 */
#define FLASH_LORA_PARAMETERS_ADDR			0x3000	/* address of sector 3 */


#define FLASH_PARAMETERS_SECTOR			0*EXT_FLASH_SECTOR_SIZE
#define FLASH_TEMPERATURES_SECTOR		1*EXT_FLASH_SECTOR_SIZE
#define FLASH_BATTERY_LEVEL_SECTOR		2*EXT_FLASH_SECTOR_SIZE
#define FLASH_LORA_PARAMETERS_SECTOR		3*EXT_FLASH_SECTOR_SIZE

/*============================================================================*/
/*                   ENUM OR TYPE DEFINITION                                  */
/*============================================================================*/

/*============================================================================*/
/*                   EXTERN DATA DEFINITION                                   */
/*============================================================================*/
//extern param_algo params_algo;
/*============================================================================*/
/*                   EXTERN FUNCTION PROTOTYPES                               */
/*============================================================================*/

/******************************************************************************/
/** \brief Initialize the external flash driver
 * This should be called once to give the SPI handle to the ext_flash driver.
 ******************************************************************************/
void ext_flash_init(void);

/******************************************************************************/
/** \brief Deinitialize the external flash driver
 ******************************************************************************/
void ext_flash_deinit(void);

/******************************************************************************/
/** \brief Returns the size in bytes of the detected flash.
 * It returns 0 if no flash was found.
 ******************************************************************************/
uint32_t ext_flash_Get_Size(void);

/******************************************************************************/
/** \brief Erase a sector of 4KByte of external Flash
 *
 * This call ensures the previous write or erase operation terminated (and
 * performs an active-wait until this condition becomes true) before erasing
 * the sector.
 *
 * @param[in] FLASH_Sector Any address within the 4KByte sector to erase
 ******************************************************************************/
void ext_flash_erase_sector(uint32_t FLASH_Sector);

/******************************************************************************/
/** \brief Erase a block of 64KByte of external Flash
 *
 * This call ensures the previous write or erase operation terminated (and
 * performs an active-wait until this condition becomes true) before erasing
 * the block.
 *
 * @param[in] FLASH_Block Any address within the 64KByte block to erase
 ******************************************************************************/
void ext_flash_erase_block(uint32_t FLASH_Block);

/******************************************************************************/
/** \brief Write data to the external flash
 *
 * This call ensures the previous write or erase operation terminated (and
 * performs an active-wait until this condition becomes true) before writing
 * new data.
 * The area which is written must have been previously erased with
 * ext_flash_erase_sector() or ext_flash_erase_block().
 *
 * @param[in] address The first address in external Flash where the write will occur
 * @param[in] data The input data buffer to write
 * @param[in] nb_byte The number of bytes to write
 ******************************************************************************/
void ext_flash_write(uint32_t address, const char *data, uint32_t nb_byte);

/******************************************************************************/
/** \brief Wait until the last write or erase operation is terminated
 * Use with caution as it performs an active wait (CPU is unavailable for other
 * tasks - except interrupts)
 ******************************************************************************/
void ext_flash_last_write_or_erase_done(void);

/******************************************************************************/
/** \brief Read data from the external flash
 *
 * @param[in] address The first address in external Flash where the read will occur
 * @param[out] data The output data buffer where the read result will be stored
 * @param[in] nb_byte The number of bytes to read
 ******************************************************************************/
void ext_flash_read(uint32_t address, char *data, uint32_t nb_byte);

/******************************************************************************/
/** \brief Check that external flash is present
 * \return
 * - 1 if found
 * - 0 if not found
 ******************************************************************************/
int32_t ext_flash_is_detected(void);

/******************************************************************************/
/** \brief Put the external Flash in DeepPowerDown mode
 *
 * Once in DeepowerDown mode, the external Flash current consumption is reduced
 * to ~2µA. The only way to wake-up the external Flash after that is to call
 * ext_flash_release_from_deep_powerdown().
 ******************************************************************************/
void ext_flash_deep_powerdown(void);

/******************************************************************************/
/** \brief Release the external Flash from DeepPowerDown mode
 *
 * This call must be made after a previous call of ext_flash_deep_powerdown()
 * in order to use ext_flash functions.
 ******************************************************************************/
void ext_flash_release_from_deep_powerdown(void);

/******************************************************************************/
/** \brief Enabling the flash by switching on its VDD
 *
 * This call should be done safely
 ******************************************************************************/
void ext_flash_power_on(void);

/******************************************************************************/
/** \brief Shutting off the flash by disabling its VDD
 *
 * This call should be done safely
 ******************************************************************************/
void ext_flash_power_off(void);

/*"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""*/
/**
  * @brief Debug command handling for lib_flash module.
  *
  * @param [IN] s8p_Commande command to be handled
  */
/*"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""*/
void vd_lib_flash_commands(char * s8p_Commande, int32_t s32_Param);

int loading_parameters(char *params, bool check_new_value, uint32_t new_value);
int saving_parameters(char *params, bool check_new_value, uint32_t new_value);
void init_usecase_parameters(bool reset);

#ifdef __cplusplus
}
#endif

#endif /* EXT_FLASH_H */
/** @} */
/*============================================================================*/
/*                   END OF FILE                                              */
/*============================================================================*/
