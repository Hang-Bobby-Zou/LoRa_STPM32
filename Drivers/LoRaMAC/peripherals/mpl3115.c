/*!
 * \file      mpl3115.h
 *
 * \brief     MPL3115 Temperature, pressure and altitude sensor driver implementation
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
#include <stdbool.h>
#include "utilities.h"
#include "delay.h"
#include "i2c.h"
#include "mpl3115.h"

extern I2c_t I2c;

/*!
 * I2C device address
 */
static uint8_t I2cDeviceAddr = 0;

/*!
 * Indicates if the MPL3115 is initialized or not
 */
static bool MPL3115Initialized = false;

/*!
 * Defines the barometric reading types
 */
typedef enum
{
    PRESSURE,
    ALTITUDE,
}BarometerReadingType_t;

/*!
 * \brief Writes a byte at specified address in the device
 *
 * \param [IN]:    addr
 * \param [IN]:    data
 * \retval status [SUCCESS, FAIL]
 */
uint8_t MPL3115Write( uint8_t addr, uint8_t data );

/*!
 * \brief Writes a buffer at specified address in the device
 *
 * \param [IN]: addr
 * \param [IN]: data
 * \param [IN]: size
 * \retval status [SUCCESS, FAIL]
 */
uint8_t MPL3115WriteBuffer( uint8_t addr, uint8_t *data, uint8_t size );

/*!
 * \brief Reads a byte at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \retval status [SUCCESS, FAIL]
 */
uint8_t MPL3115Read( uint8_t addr, uint8_t *data );

/*!
 * \brief Reads a buffer at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \param [IN]: size
 * \retval status [SUCCESS, FAIL]
 */
uint8_t MPL3115ReadBuffer( uint8_t addr, uint8_t *data, uint8_t size );

/*!
 * \brief Sets the I2C device slave address
 *
 * \param [IN]: addr
 */
void MPL3115SetDeviceAddr( uint8_t addr );

/*!
 * \brief Gets the I2C device slave address
 *
 * \retval: addr Current device slave address
 */
uint8_t MPL3115GetDeviceAddr( void );

/*!
 * \brief Sets the device in barometer Mode
 */
void MPL3115SetModeBarometer( void );

/*!
 * \brief Sets the device in altimeter Mode
 */
void MPL3115SetModeAltimeter( void );

/*!
 * \brief Sets the device in standby
 */
void MPL3115SetModeStandby( void );

/*!
 * \brief Sets the device in active Mode
 */
void MPL3115SetModeActive( void );

/*!
 * \brief Toggles the OST bit causing the sensor to immediately take another
 *        reading
 */
void MPL3115ToggleOneShot( void );

uint8_t MPL3115Init( void )
{
    uint8_t regVal = 0;

    MPL3115SetDeviceAddr( MPL3115A_I2C_ADDRESS );

    if( MPL3115Initialized == false )
    {
        MPL3115Write( CTRL_REG1, RST );
        DelayMs( 50 );
        I2cResetBus( &I2c );

        // Check MPL3115 ID
        MPL3115Read( MPL3115_ID, &regVal );
        if( regVal != 0xC4 )
        {
            return FAIL;
        }

        MPL3115Write( PT_DATA_CFG_REG, DREM | PDEFE | TDEFE );      // Enable data ready flags for pressure and temperature )
        MPL3115Write( CTRL_REG1, ALT | OS_32 | SBYB );              // Set sensor to active state with oversampling ratio 128 (512 ms between samples)
        MPL3115Initialized = true;
    }
    return SUCCESS;
}

uint8_t MPL3115Reset( void )
{
    // Reset all registers to POR values
    if( MPL3115Write( CTRL_REG1, RST ) == SUCCESS )
    {
        return SUCCESS;
    }
    return FAIL;
}

uint8_t MPL3115Write( uint8_t addr, uint8_t data )
{
    return MPL3115WriteBuffer( addr, &data, 1 );
}

uint8_t MPL3115WriteBuffer( uint8_t addr, uint8_t *data, uint8_t size )
{
    return I2cWriteBuffer( &I2c, I2cDeviceAddr << 1, addr, data, size );
}

uint8_t MPL3115Read( uint8_t addr, uint8_t *data )
{
    return MPL3115ReadBuffer( addr, data, 1 );
}

uint8_t MPL3115ReadBuffer( uint8_t addr, uint8_t *data, uint8_t size )
{
    return I2cReadBuffer( &I2c, I2cDeviceAddr << 1, addr, data, size );
}

void MPL3115SetDeviceAddr( uint8_t addr )
{
    I2cDeviceAddr = addr;
}

uint8_t MPL3115GetDeviceAddr( void )
{
    return I2cDeviceAddr;
}

static float MPL3115ReadBarometer( BarometerReadingType_t type )
{
		return 0;
}

float MPL3115ReadAltitude( void )
{
    return MPL3115ReadBarometer( ALTITUDE );
}

float MPL3115ReadPressure( void )
{
    return MPL3115ReadBarometer( PRESSURE );
}

float MPL3115ReadTemperature( void )
{
		return 0;
}

void MPL3115ToggleOneShot( void )
{
    uint8_t ctrlReg = 0;

    MPL3115SetModeStandby( );

    MPL3115Read( CTRL_REG1, &ctrlReg );           // Read current settings
    ctrlReg &= ~OST;                              // Clear OST bit
    MPL3115Write( CTRL_REG1, ctrlReg );

    MPL3115Read( CTRL_REG1, &ctrlReg );           // Read current settings to be safe
    ctrlReg |= OST;                               // Set OST bit
    MPL3115Write( CTRL_REG1, ctrlReg );

    MPL3115SetModeActive( );
}

void MPL3115SetModeBarometer( void )
{
    uint8_t ctrlReg = 0;

    MPL3115SetModeStandby( );

    MPL3115Read( CTRL_REG1, &ctrlReg );           // Read current settings
    ctrlReg &= ~ALT;                              // Set ALT bit to zero
    MPL3115Write( CTRL_REG1, ctrlReg );

    MPL3115SetModeActive( );
}

void MPL3115SetModeAltimeter( void )
{
    uint8_t ctrlReg = 0;

    MPL3115SetModeStandby( );

    MPL3115Read( CTRL_REG1, &ctrlReg );           // Read current settings
    ctrlReg |= ALT;                               // Set ALT bit to one
    MPL3115Write( CTRL_REG1, ctrlReg );

    MPL3115SetModeActive( );
}

void MPL3115SetModeStandby( void )
{
    uint8_t ctrlReg = 0;

    MPL3115Read( CTRL_REG1, &ctrlReg );
    ctrlReg &= ~SBYB;                             // Clear SBYB bit for Standby mode
    MPL3115Write( CTRL_REG1, ctrlReg );
}

void MPL3115SetModeActive( void )
{
    uint8_t ctrlReg = 0;

    MPL3115Read( CTRL_REG1, &ctrlReg );
    ctrlReg |= SBYB;                              // Set SBYB bit for Active mode
    MPL3115Write( CTRL_REG1, ctrlReg );
}
