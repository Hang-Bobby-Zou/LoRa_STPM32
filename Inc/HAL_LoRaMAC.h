
#include <stdint.h>

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           242

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];


/*!
 * Device states
 */
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP,
		DEVICE_STATE_IDLE
}DeviceState;


void LoRaMAC_Init(void);

void LoRaMAC_Join(void);

int LoRaMAC_Send(void);

void LoRaMAC(void);

