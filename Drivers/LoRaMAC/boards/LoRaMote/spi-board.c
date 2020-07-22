/*!
 * \file      spi-board.c
 *
 * \brief     Target board SPI driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include "stm32l4xx.h"
#include "utilities.h"
#include "board.h"
#include "gpio.h"
#include "spi-board.h"

#include "HAL_spi.h"

static SPI_HandleTypeDef SpiHandle[2];

/*============================================================================*/
/*                   UNUSED FUNCTIONS			                                    */
/*============================================================================*/
void SpiInit( Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss ){ }
void SpiDeInit( Spi_t *obj ){ }
void SpiFormat( Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave ){ }
void SpiFrequency( Spi_t *obj, uint32_t hz ){ }

/*============================================================================*/
/*                   USER FUNCTIONS																						*/
/*============================================================================*/
uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
	return ( SPI2_InOut(outData) );	
}

