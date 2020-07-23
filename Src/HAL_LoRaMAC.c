
#include "utilities.h"
#include "board.h"	
#include "gpio.h"
#include "gps.h"
#include "mpl3115.h"
#include "LoRaMac.h"
#include "Commissioning.h"

#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"
#include "tim.h"
#include "HAL_LoRaMAC.h"
#include "usart.h"

// Define the active region to be CN470
#define REGION_CN470
#define ACTIVE_REGION	LORAMAC_REGION_CN470
#define LORAWAN_DEVICE_CLASS         CLASS_A

LoRaMacPrimitives_t LoRaMacPrimitives;
LoRaMacCallback_t LoRaMacCallbacks;
MibRequestConfirm_t mibReq;

uint32_t frame_count = 0;
extern uint8_t aRxBuffer[8];
extern uint32_t UpLinkCounter;

#define DelayMsPoll(x) { for (uint32_t j = 0; j < x; j++) {for (uint32_t i = 0; i < 8000; i++) {  }}}	

#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_EU868 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_EU868

#endif

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              0

#if defined( REGION_EU868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = 16;
static uint8_t AppDataSizeBackup = 16;

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
//static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

/*!
 * LED GPIO pins objects
 */
//extern Gpio_t Led1;
//extern Gpio_t Led2;
//extern Gpio_t Led3;

static LoRaMacStatus_t status;


/*!
 * \brief   Prepares the payload of the frame
 */
void PrepareTxFrame( uint8_t port )
{
    const LoRaMacRegion_t region = ACTIVE_REGION;

    switch( port )
    {
    case 2:
        switch( region )
        {
            case LORAMAC_REGION_CN779:
            case LORAMAC_REGION_EU868:
            case LORAMAC_REGION_IN865:
            case LORAMAC_REGION_KR920:
            case LORAMAC_REGION_AS923:
            case LORAMAC_REGION_AU915:
            case LORAMAC_REGION_US915:
            case LORAMAC_REGION_US915_HYBRID:
            default:
                // Unsupported region.
                break;
        }
        break;
    case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            // Network not joined yet. Try to join again
            MlmeReq_t mlmeReq;
            mlmeReq.Type = MLME_JOIN;
            mlmeReq.Req.Join.DevEui = DevEui;
            mlmeReq.Req.Join.AppEui = AppEui;
            mlmeReq.Req.Join.AppKey = AppKey;
            mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

            if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
            {
                DeviceState = DEVICE_STATE_SLEEP;
            }
            else
            {
                DeviceState = DEVICE_STATE_CYCLE;
            }
        }
    }
}

/*!
 * \brief Function executed on Led 1 Timeout event
 */
//static void OnLed1TimerEvent( void )
//{
//    TimerStop( &Led1Timer );
    // Switch LED 1 OFF
//    GpioWrite( &Led1, 1 );
//}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
//static void OnLed2TimerEvent( void )
//{
//    TimerStop( &Led2Timer );
		//Switch LED 2 OFF
//    GpioWrite( &Led2, 1 );
//}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    // if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    // {
    //     switch( mcpsConfirm->McpsRequest )
    //     {
    //         case MCPS_UNCONFIRMED:
    //         {
    //             // Check Datarate
    //             // Check TxPower
    //             break;
    //         }
    //         case MCPS_CONFIRMED:
    //         {
    //             // Check Datarate
    //             // Check TxPower
    //             // Check AckReceived
    //             // Check NbTrials
    //             break;
    //         }
    //         case MCPS_PROPRIETARY:
    //         {
    //             break;
    //         }
    //         default:
    //             break;
    //     }

    //     // Switch LED 1 ON
    //     //GpioWrite( &Led1, 0 );
    //     //TimerStart( &Led1Timer );
    // }
    // NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    if( mcpsIndication->FramePending == true )
    {
        // The server signals that it has pending data to be sent.
        // We schedule an uplink as soon as possible to flush the server.
        OnTxNextPacketTimerEvent( );
    }
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( mcpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
                //GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 0 : 1 );
            }
            break;
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( mcpsIndication->BufferSize == 4 ) &&
                    ( mcpsIndication->Buffer[0] == 0x01 ) &&
                    ( mcpsIndication->Buffer[1] == 0x01 ) &&
                    ( mcpsIndication->Buffer[2] == 0x01 ) &&
                    ( mcpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSizeBackup = AppDataSize;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn( false );
#endif
                    //GpsStop( );
                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = AppDataSizeBackup;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                    //GpsStart( );
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = mcpsIndication->BufferSize;

                    AppData[0] = 4;
                    for( uint8_t i = 1; i < MIN( AppDataSize, LORAWAN_APP_DATA_MAX_SIZE ); i++ )
                    {
                        AppData[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    break;
                case 6: // (ix)
                    {
                        MlmeReq_t mlmeReq;

                        // Disable TestMode and revert back to normal operation
                        IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                        AppPort = LORAWAN_APP_PORT;
                        AppDataSize = AppDataSizeBackup;
                        ComplianceTest.DownLinkCounter = 0;
                        ComplianceTest.Running = false;

                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                        LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( REGION_EU868 )
                        LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                        //GpsStart( );

                        mlmeReq.Type = MLME_JOIN;

                        mlmeReq.Req.Join.DevEui = DevEui;
                        mlmeReq.Req.Join.AppEui = AppEui;
                        mlmeReq.Req.Join.AppKey = AppKey;
                        mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

                        if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                        {
                            DeviceState = DEVICE_STATE_SLEEP;
                        }
                        else
                        {
                            DeviceState = DEVICE_STATE_CYCLE;
                        }
                    }
                    break;
                case 7: // (x)
                    {
                        if( mcpsIndication->BufferSize == 3 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        else if( mcpsIndication->BufferSize == 7 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW_1;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            mlmeReq.Req.TxCw.Frequency = ( uint32_t )( ( mcpsIndication->Buffer[3] << 16 ) | ( mcpsIndication->Buffer[4] << 8 ) | mcpsIndication->Buffer[5] ) * 100;
                            mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        ComplianceTest.State = 1;
                    }
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }
    }

    // Switch LED 2 ON for each received downlink
    //GpioWrite( &Led2, 0 );
    //TimerStart( &Led2Timer );
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                DeviceState = DEVICE_STATE_SEND;
            }
            else
            {
                // Join was not successful. Try to join again
                MlmeReq_t mlmeReq;
                mlmeReq.Type = MLME_JOIN;
                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

                if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                {
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                else
                {
                    DeviceState = DEVICE_STATE_CYCLE;
                }
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
            }
            break;
        }
        default:
            break;
    }
    NextTx = true;
}

/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] mlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication( MlmeIndication_t *mlmeIndication )
{
    switch( mlmeIndication->MlmeIndication )
    {
        case MLME_SCHEDULE_UPLINK:
        {// The MAC signals that we shall provide an uplink as soon as possible
            OnTxNextPacketTimerEvent( );
            break;
        }
        default:
            break;
    }
}

/*============================================================================*/
/*                   PRIVATE FUNCTIONS		                                    */
/*============================================================================*/
void LoRaMAC_Init(void){

  BoardInitMcu( );
  
	LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
  LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
  LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
  LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
  LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;

  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN470 );

  //TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );	//Not added in Mei
	
	mibReq.Type = MIB_ADR;
	mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
	status = LoRaMacMibSetRequestConfirm( &mibReq );

	mibReq.Type = MIB_PUBLIC_NETWORK;
	mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
	status = LoRaMacMibSetRequestConfirm( &mibReq );

	mibReq.Type = MIB_DEVICE_CLASS;
	mibReq.Param.Class = LORAWAN_DEVICE_CLASS;
	status = LoRaMacMibSetRequestConfirm( &mibReq );	
	

#if defined( REGION_EU868 )
  LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );

//IDK what is this
#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )
	LoRaMacChannelAdd( 3, ( ChannelParams_t )LC4 );
	LoRaMacChannelAdd( 4, ( ChannelParams_t )LC5 );
	LoRaMacChannelAdd( 5, ( ChannelParams_t )LC6 );
	LoRaMacChannelAdd( 6, ( ChannelParams_t )LC7 );
	LoRaMacChannelAdd( 7, ( ChannelParams_t )LC8 );
	LoRaMacChannelAdd( 8, ( ChannelParams_t )LC9 );
	LoRaMacChannelAdd( 9, ( ChannelParams_t )LC10 );
	//Enabling this will stuck the network connection on Objenious network.
	//mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
	// mibReq.Param.Rx2DefaultChannel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
	//LoRaMacMibSetRequestConfirm( &mibReq );
	mibReq.Type = MIB_RX2_CHANNEL;
	mibReq.Param.Rx2Channel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
	LoRaMacMibSetRequestConfirm( &mibReq );
#endif

#endif
}



void LoRaMAC_Join(void){
#if( OVER_THE_AIR_ACTIVATION != 0 )
  MlmeReq_t mlmeReq;

  // Initialize LoRaMac device unique ID
  //BoardGetUniqueId( DevEui );

  mlmeReq.Type = MLME_JOIN;

  mlmeReq.Req.Join.DevEui = DevEui;
  mlmeReq.Req.Join.AppEui = AppEui;
  mlmeReq.Req.Join.AppKey = AppKey;
  mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

	if (NextTx == true){
		int ret = LoRaMacMlmeRequest( &mlmeReq );
		if( ret == LORAMAC_STATUS_OK )
		{
			// Join request was send successfully
			myprintf("OTAA-LoRaMAC join request SUCCESS\r\n");
			//leds_play_sequence(&led_state_JOIN, 1);
		}
		else if (ret == LORAMAC_STATUS_BUSY)
		{
			myprintf("OTAA-Join request ERROR : LoRaMAC is BUSY\r\n");
		}
		else
		{
			myprintf("OTAA-Join request ERROR : %d\r\n", ret);
		}
	}
#else
		// Choose a random device address if not already defined in Commissioning.h
		if( DevAddr == 0 )
		{
			// Random seed initialization
			srand1( BoardGetRandomSeed( ) );
		
			// Choose a random device address
			DevAddr = randr( 0, 0x01FFFFFF );
    }

		//static LoRaMacStatus_t status;
		
    mibReq.Type = MIB_NET_ID;
    mibReq.Param.NetID = LORAWAN_NETWORK_ID;
    status = LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_DEV_ADDR;
    mibReq.Param.DevAddr = DevAddr;
    status = LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_NWK_SKEY;
    mibReq.Param.NwkSKey = NwkSKey;
    status = LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_APP_SKEY;
    mibReq.Param.AppSKey = AppSKey;
    status = LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_NETWORK_JOINED;
    mibReq.Param.IsNetworkJoined = true;
    status = LoRaMacMibSetRequestConfirm( &mibReq );
#endif
}




int LoRaMAC_Send(void){
	
	McpsReq_t mcpsReq;
	LoRaMacTxInfo_t txInfo;	
	MibRequestConfirm_t mibReq;
	LoRaMacStatus_t status;

	mibReq.Type = MIB_NETWORK_JOINED;
	status = LoRaMacMibGetRequestConfirm( &mibReq );
	if( status != LORAMAC_STATUS_OK ) 
	{		
		return -1;
	}	
	
	if( mibReq.Param.IsNetworkJoined == false ) 
	{
		LoRaMAC_Join();
		return -1;
	}
	
	
	if( NextTx == true )
  {
		
		//PrepareTxFrame( AppPort );
		
		AppData[0] = aRxBuffer[0];
		AppData[1] = aRxBuffer[1];
		AppData[2] = aRxBuffer[2];
		AppData[3] = aRxBuffer[3];
		AppData[4] = 0xAA;
		
		/*
		AppData[0] = 0xAA;
		AppData[1] = 0xAA;
		AppData[2] = 0xAA;
		AppData[3] = 0xAA;
		AppData[4] = 0xAA;
		*/
		
		AppDataSize = 5;
		
if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    } 
		
		status = LoRaMacMcpsRequest( &mcpsReq ); 
		
		if (status == LORAMAC_STATUS_OK ){
			frame_count++;
			UpLinkCounter++;
			//myprintf("Frame %lu Sent Success\r\n", (unsigned long) frame_count);
			return 0;
		} else if (status == LORAMAC_STATUS_BUSY){
			//myprintf("LoRaMAC Status Busy\r\n");
			return -1;
		}
		else
		{
			//myprintf("Frame ERROR (%d)(/%u)\r\n", status, frame_count);
			return -1;
		}
		
  }
}

