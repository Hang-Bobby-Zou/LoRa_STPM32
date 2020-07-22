

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

