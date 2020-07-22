/*!
 * \file      gpio-ioe.h
 *
 * \brief     IO expander driver implementation (based on the sx1509)
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
#include <stdlib.h>
#include <stdbool.h>
#include "gpio-ioe.h"
#include "sx1509.h"

static GpioIoeIrqHandler *GpioIoeIrq[16];

void GpioIoeInit( Gpio_t *obj, PinNames pin, PinModes mode,  PinConfigs config, PinTypes type, uint32_t value )
{
    
}

void GpioIoeWrite( Gpio_t *obj, uint32_t value )
{
	
}

void GpioIoeToggle( Gpio_t *obj )
{
    GpioIoeWrite( obj, GpioIoeRead( obj ) ^ 1 );
}

uint32_t GpioIoeRead( Gpio_t *obj )
{
    uint8_t regAdd = 0;
    uint8_t regVal = 0;

    if( ( obj->pin % 16 ) > 0x07 )
    {
        regAdd = RegDataB;
    }
    else
    {
        regAdd = RegDataA;
    }

    SX1509Read( regAdd, &regVal );

    if( ( regVal & obj->pinIndex ) == 0x00 )
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

void GpioIoeInterruptHandler( void )
{

}
