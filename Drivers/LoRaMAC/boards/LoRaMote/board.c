/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
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
#include "delay.h"
#include "gpio.h"
#include "gpio-ioe.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "timer.h"
#include "gps.h"
#include "mpl3115.h"
#include "mag3110.h"
#include "mma8451.h"
#include "sx9500.h"
#include "board-config.h"
#include "rtc-board.h"
#include "sx1276-board.h"
#if defined( USE_USB_CDC )
#include "uart-usb-board.h"
#endif
#include "board.h"

#include "main.h"

/*!
 * Unique Devices IDs register set ( STM32L1xxx )
 */
#define         ID1                                 ( 0x1FF80050 )
#define         ID2                                 ( 0x1FF80054 )
#define         ID3                                 ( 0x1FF80064 )

/*!
 * LED GPIO pins objects
 */
Gpio_t Led1;
Gpio_t Led2;
Gpio_t Led3;

/*
 * MCU objects
 */
Adc_t Adc;
I2c_t I2c;
Uart_t Uart1;
#if defined( USE_USB_CDC )
Uart_t UartUsb;
#endif

/*!
 * System Clock Re-Configuration when waking up from STOP mode
 */
static void SystemClockReConfig( void );

/*!
 * Timer used at first boot to calibrate the SystemWakeupTime
 */
//static TimerEvent_t CalibrateSystemWakeupTimeTimer;

/*!
 * Flag to indicate if the MCU is Initialized
 */
//static bool McuInitialized = false;

/*!
 * Flag to indicate if the SystemWakeupTime is Calibrated
 */
static bool SystemWakeupTimeCalibrated = false;

/*!
 * Callback indicating the end of the system wake-up time calibration
 */
static void OnCalibrateSystemWakeupTimeTimerEvent( void )
{
    SystemWakeupTimeCalibrated = true;
}

/*!
 * Nested interrupt counter.
 *
 * \remark Interrupt should only be fully disabled once the value is 0
 */
static uint8_t IrqNestLevel = 0;

void BoardDisableIrq( void )
{
    __disable_irq( );
    IrqNestLevel++;
}

void BoardEnableIrq( void )
{
    IrqNestLevel--;
    if( IrqNestLevel == 0 )
    {
        __enable_irq( );
    }
}

void BoardInitPeriph( void )
{

}

void BoardInitMcu( void )
{
	SX1276.Spi.Nss.pin = PB_12;
	//SX1276.Spi.Nss.pin = SPI2_NSS_Pin;
	
	SX1276.Spi.Nss.pinIndex = ( 0x01 << ( SX1276.Spi.Nss.pin & 0x0F ) );
	SX1276.Spi.Nss.port = SPI2_NSS_GPIO_Port;
	
	SX1276.Spi.Nss.pull = PIN_PULL_UP;
	//SX1276.Spi.Nss.pull = GPIO_NOPULL;
/*	
	SX1276.Spi.Sclk.pin = GPIO_PIN_13;
	SX1276.Spi.Sclk.pinIndex = ( 0x01 << ( SX1276.Spi.Sclk.pin & 0x0F ) );
	SX1276.Spi.Sclk.port = GPIOB;
	SX1276.Spi.Sclk.pull = GPIO_NOPULL;
	
	SX1276.Spi.Miso.pin = GPIO_PIN_14;
	SX1276.Spi.Miso.pinIndex = ( 0x01 << ( SX1276.Spi.Miso.pin & 0x0F ) );
	SX1276.Spi.Miso.port = GPIOB;
	SX1276.Spi.Miso.pull = GPIO_NOPULL;
	
	SX1276.Spi.Mosi.pin = GPIO_PIN_15;
	SX1276.Spi.Mosi.pinIndex = ( 0x01 << ( SX1276.Spi.Mosi.pin & 0x0F ) );
	SX1276.Spi.Mosi.port = GPIOB;
	SX1276.Spi.Mosi.pull = GPIO_NOPULL;
	
	SX1276.Spi.SpiId = SPI_2;
*/
	
	
}

void BoardResetMcu( void )
{
    
}

void BoardDeInitMcu( void )
{

}

uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

void BoardGetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

/*!
 * Factory power supply
 */
#define FACTORY_POWER_SUPPLY                        3300 // mV

/*!
 * VREF calibration value
 */
#define VREFINT_CAL                                 ( *( uint16_t* )0x1FF80078 )

/*!
 * ADC maximum value
 */
#define ADC_MAX_VALUE                               4095

/*!
 * Battery thresholds
 */
#define BATTERY_MAX_LEVEL                           9000 // mV
#define BATTERY_MIN_LEVEL                           4700 // mV
#define BATTERY_SHUTDOWN_LEVEL                      4800 // mV

//static uint16_t BatteryVoltage = BATTERY_MAX_LEVEL;

uint16_t BoardBatteryMeasureVolage( void )
{
    return 0;
}

uint32_t BoardGetBatteryVoltage( void )
{
    return 0;
}

uint8_t BoardGetBatteryLevel( void )
{
    return 0;
}

void SystemClockReConfig( void )
{

}

//void SysTick_Handler( void )
//{
//    HAL_IncTick( );
//    HAL_SYSTICK_IRQHandler( );
//}

uint8_t GetBoardPowerSource( void )
{
#if defined( USE_USB_CDC )
    if( UartUsbIsUsbCableConnected( ) == 0 )
    {
        return BATTERY_POWER;
    }
    else
    {
        return USB_POWER;
    }
#else
    return BATTERY_POWER;
#endif
}



#ifdef USE_FULL_ASSERT
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %lu\r\n", file, line) */

    printf( "Wrong parameters value: file %s on line %lu\r\n", ( const char* )file, line );
    /* Infinite loop */
    while( 1 )
    {
    }
}
#endif
