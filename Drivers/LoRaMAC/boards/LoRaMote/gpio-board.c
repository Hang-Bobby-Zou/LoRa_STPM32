/*!
 * \file      gpio-board.c
 *
 * \brief     Target board GPIO driver implementation
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
#include "board-config.h"
#include "rtc-board.h"
#include "gpio-board.h"
#if defined( BOARD_IOE_EXT )
#include "gpio-ioe.h"
#endif
#include "HAL_gpio.h"

static GpioIrqHandler *GpioIrq[16];

/*============================================================================*/
/*                   UNUSED FUNCTIONS			                                    */
/*============================================================================*/
void GpioMcuInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value ){ }
void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler ){ }
void GpioMcuRemoveInterrupt( Gpio_t *obj ){ }

/*============================================================================*/
/*                   USER FUNCTIONS																						*/
/*============================================================================*/
void GpioMcuWrite( Gpio_t *obj, uint32_t value )
{
        if( ( obj == NULL ) || ( obj->port == NULL ) )
        {
            assert_param( FAIL );
        }
        // Check if pin is not connected
        if( obj->pin == NC )
        {
            return;
        }
        HAL_GPIO_WritePin( obj->port, obj->pinIndex , ( GPIO_PinState )value );
}

void GpioMcuToggle( Gpio_t *obj )
{

        if( ( obj == NULL ) || ( obj->port == NULL ) )
        {
            assert_param( FAIL );
        }

        // Check if pin is not connected
        if( obj->pin == NC )
        {
            return;
        }
        HAL_GPIO_TogglePin( obj->port, obj->pinIndex );
}

uint32_t GpioMcuRead( Gpio_t *obj )
{
    if( obj->pin < IOE_0 )
    {
        if( obj == NULL )
        {
            assert_param( FAIL );
        }
        // Check if pin is not connected
        if( obj->pin == NC )
        {
            return 0;
        }
        return HAL_GPIO_ReadPin( obj->port, obj->pinIndex );
    }
    else
    {
			return 0;
    }
}


void EXTI0_IRQHandler( void )
{
//#if !defined( USE_NO_TIMER )
//    RtcRecoverMcuStatus( );
//#endif
    HAL_GPIO_EXTI_IRQHandler(GPIO_SX1276_DIO0_PIN);
}

void EXTI1_IRQHandler( void )
{
//#if !defined( USE_NO_TIMER )
//    RtcRecoverMcuStatus( );
//#endif
    HAL_GPIO_EXTI_IRQHandler(GPIO_SX1276_DIO1_PIN);
}

void EXTI2_IRQHandler( void )
{
//#if !defined( USE_NO_TIMER )
//    RtcRecoverMcuStatus( );
//#endif
    HAL_GPIO_EXTI_IRQHandler(GPIO_SX1276_DIO2_PIN);
}

void EXTI3_IRQHandler( void )
{
//#if !defined( USE_NO_TIMER )
//    RtcRecoverMcuStatus( );
//#endif
    HAL_GPIO_EXTI_IRQHandler(GPIO_SX1276_DIO3_PIN);
}

void EXTI4_IRQHandler( void )
{
//#if !defined( USE_NO_TIMER )
//    RtcRecoverMcuStatus( );
//#endif
    HAL_GPIO_EXTI_IRQHandler(GPIO_SX1276_DIO4_PIN);
}

void EXTI9_5_IRQHandler( void )
{
//#if !defined( USE_NO_TIMER )
//    RtcRecoverMcuStatus( );
//#endif
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_5 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_6 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_7 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_8 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_9 );
}

void EXTI15_10_IRQHandler( void )
{
//#if !defined( USE_NO_TIMER )
//    RtcRecoverMcuStatus( );
//#endif
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_10 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_11 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_12 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_13 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_14 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_15 );
	HAL_GPIO_EXTI_IRQHandler(GPIO_SWITCH_PIN);
}



void HAL_GPIO_EXTI_Callback( uint16_t gpioPin )
{

	uint8_t callbackIndex = 0;

    if( gpioPin > 0 )
    {
        while( gpioPin != 0x01 )
        {
            gpioPin = gpioPin >> 1;
            callbackIndex++;
        }
    }

    if( GpioIrq[callbackIndex] != NULL )
    {
        GpioIrq[callbackIndex]( );
    }

/*
switch(gpioPin)
	{
		case(GPIO_SX1276_DIO0_PIN):
		{
      GpioIrq[0]();
			break;
		}
		case(GPIO_SX1276_DIO1_PIN):
		{
      GpioIrq[1]();
			break;
		}
		case(GPIO_SX1276_DIO2_PIN):
		{
      GpioIrq[2]();
			break;
		}
		case(GPIO_SX1276_DIO3_PIN):
		{
      GpioIrq[3]();
			break;
		}
		case(GPIO_SX1276_DIO4_PIN):
		{
      GpioIrq[4]();
			break;
		}	
	}
*/
}