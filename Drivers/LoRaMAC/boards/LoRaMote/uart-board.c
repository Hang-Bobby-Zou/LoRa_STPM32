/*!
 * \file      uart-board.c
 *
 * \brief     Target board UART driver implementation
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
#include "uart-board.h"
#include "usart.h"

static UART_HandleTypeDef UartHandle;
uint8_t RxData = 0;
uint8_t TxData = 0;

extern Uart_t Uart1;

/*============================================================================*/
/*                   UNUSED FUNCTIONS			                                    */
/*============================================================================*/
void UartMcuInit( Uart_t *obj, UartId_t uartId, PinNames tx, PinNames rx ){ }
void UartMcuConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl ){ }
void UartMcuDeInit( Uart_t *obj ){ }
uint8_t UartMcuPutChar( Uart_t *obj, uint8_t data ){return 0; }
uint8_t UartMcuGetChar( Uart_t *obj, uint8_t *data ){return 0; }
uint8_t UartMcuPutBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size ){return 0; }
uint8_t UartMcuGetBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size, uint16_t *nbReadBytes ){return 0; }

/*============================================================================*/
/*                   USER FUNCTIONS				                                    */
/*============================================================================*/
//Migrated form freertos.c

// void HAL_UART_TxCpltCallback( UART_HandleTypeDef *handle )
// {
// 	/*
// 	if( IsFifoEmpty( &Uart1.FifoTx ) == false )
//     {
//         TxData = FifoPop( &Uart1.FifoTx );
//         //  Write one byte to the transmit data register
//         HAL_UART_Transmit_IT( &UartHandle, &TxData, 1 );
//     }

//     if( Uart1.IrqNotify != NULL )
//     {
//         Uart1.IrqNotify( UART_NOTIFY_TX );
//     }
// 	*/
// }

// void HAL_UART_RxCpltCallback( UART_HandleTypeDef *handle )
// {
// 	/*
// 	if( IsFifoFull( &Uart1.FifoRx ) == false )
//     {
//         // Read one byte from the receive data register
//         FifoPush( &Uart1.FifoRx, RxData );
//     }

//     if( Uart1.IrqNotify != NULL )
//     {
//         Uart1.IrqNotify( UART_NOTIFY_RX );
//     }

//     HAL_UART_Receive_IT( &UartHandle, &RxData, 1 );
// 	*/
// }

//void HAL_UART_ErrorCallback( UART_HandleTypeDef *handle )
//{
//    HAL_UART_Receive_IT( &UartHandle, &RxData, 1 );
//}
