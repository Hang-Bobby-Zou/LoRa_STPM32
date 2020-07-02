/*============================================================================*/
/*                   STANDARD INCLUDE FILES                                   */
/*============================================================================*/
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>		// To include uintXX_t type
#include <stdbool.h>	// To include the bool type

bool STPM32_Init(void);
bool SendMessage(uint32_t ReadAddress, uint8_t* ReceiveMessage ,uint32_t SendAddress, uint8_t* SendMessage);
bool ReadMsgOnly (uint32_t ReadAddress, uint8_t* ReadMessage);

