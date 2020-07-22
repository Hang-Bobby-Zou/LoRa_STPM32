/*!
 * \file      rtc-board.c
 *
 * \brief     Target board RTC timer and low power modes management
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
#include <math.h>
#include "stm32l4xx.h"
#include "utilities.h"
#include "board.h"
#include "timer.h"
#include "gpio.h"
#include "rtc-board.h"
#include "tim.h"


static uint32_t RtcSetTimeout_TimerValue = 0;


/*!
 * RTC Time base in ms
 */
#define RTC_ALARM_TICK_DURATION                     0.48828125      // 1 tick every 488us
#define RTC_ALARM_TICK_PER_MS                       2.048           // 1/2.048 = tick duration in ms

/*!
 * Maximum number of days (with some margin) that can be handled by the RTC alarm counter before overflow.
 */
#define RTC_ALARM_MAX_NUMBER_OF_DAYS                27

/*!
 * RTC timer context
 */
typedef struct RtcCalendar_s
{
    uint16_t CalendarCentury;     //! Keep track of century value
    //RTC_DateTypeDef CalendarDate; //! Reference time in calendar format
    //RTC_TimeTypeDef CalendarTime; //! Reference date in calendar format
} RtcCalendar_t;

/*!
 * Current RTC timer context
 */
RtcCalendar_t RtcCalendarContext;

/*!
 * \brief Flag to indicate if the timestamp until the next event is long enough
 * to set the MCU into low power mode
 */
//static bool RtcTimerEventAllowsLowPower = false;

/*!
 * \brief Flag to disable the low power mode even if the timestamp until the
 * next event is long enough to allow low power mode
 */
//static bool LowPowerDisableDuringTask = false;

/*!
 * \brief RTC Handler
 */
//RTC_HandleTypeDef RtcHandle = { 0 };

/*!
 * \brief Indicates if the RTC is already Initialized or not
 */
//static bool RtcInitialized = false;

/*!
 * \brief Indicates if the RTC Wake Up Time is calibrated or not
 */
//static bool WakeUpTimeInitialized = false;

/*!
 * \brief Hold the Wake-up time duration in ms
 */
volatile uint32_t McuWakeUpTime = 0;

/*!
 * \brief Hold the cumulated error in micro-second to compensate the timing errors
 */
//static int32_t TimeoutValueError = 0;

/*!
 * \brief RTC wakeup time computation
 */
//static void RtcComputeWakeUpTime( void );

/*!
 * \brief Start the RTC Alarm (timeoutValue is in ms)
 */
//static void RtcStartWakeUpAlarm( uint32_t timeoutValue );

/*!
 * \brief Converts a TimerTime_t value into RtcCalendar_t value
 *
 * \param[IN] timeCounter Value to convert to RTC calendar
 * \retval rtcCalendar New RTC calendar value
 */
//
// REMARK: Removed function static attribute in order to suppress
//         "#177-D function was declared but never referenced" warning.
// static RtcCalendar_t RtcConvertTimerTimeToCalendarTick( TimerTime_t timeCounter )
//
RtcCalendar_t RtcConvertTimerTimeToCalendarTick( TimerTime_t timeCounter );




void RtcInit( void ){}

void RtcSetTimeout( uint32_t timeout )
{
	RtcSetTimeout_TimerValue = RtcGetTimerValue();
}

TimerTime_t RtcGetAdjustedTimeoutValue( uint32_t timeout )
{
	return timeout;
}

TimerTime_t RtcGetTimerValue( void )
{
	return TIM7_GetTimeMs();
}

TimerTime_t RtcGetElapsedAlarmTime( void )
{
	return( RtcGetTimerValue() - RtcSetTimeout_TimerValue );
}

TimerTime_t RtcComputeFutureEventTime( TimerTime_t futureEventInTime )
{
	return( TIM7_GetTimeMs( ) + futureEventInTime );
}

TimerTime_t RtcComputeElapsedTime( TimerTime_t eventInTime )
{
	return( RtcGetTimerValue() - eventInTime );
}

void BlockLowPowerDuringTask ( bool status ){ }

void RtcEnterLowPowerStopMode( void ){ }

void RtcRecoverMcuStatus( void ){ }

void RtcProcess( void )
{
    // Not used on this platform.
}
