/*============================================================================*/
/*                   STANDARD INCLUDE FILES                                   */
/*============================================================================*/
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>		// To include uintXX_t type
#include <stdbool.h>	// To include the bool type

bool LoRa_Init(void);
uint8_t LoRa_ReadReg(uint8_t Address);
void LoRa_WriteReg(uint8_t Address, uint8_t* WriteData);
