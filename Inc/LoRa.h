/*============================================================================*/
/*                   STANDARD INCLUDE FILES                                   */
/*============================================================================*/
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>		// To include uintXX_t type
#include <stdbool.h>	// To include the bool type

bool LoRa_Init(void);

bool LoRa_is_detected(void);
void LoRa_SetOpMode(uint8_t OperatingMode);
void LoRa_SetFskMode(uint8_t ModulationMode);
void LoRa_SetRFFrequency(uint32_t freq);
void LoRa_SetRFPower(uint8_t power);
void LoRa_SetSpreadingFactor(uint8_t factor);
void LoRa_SetErrorCoding(uint8_t value);
void LoRa_SetPacketCrcOn(bool enable);
void LoRa_SetSignalBandwidth(uint8_t bw);
void LoRa_SetImplicitHeaderOn(bool enable);
void LoRa_SetPayloadLength(uint8_t value);
void LoRa_SetSymbTimeout(unsigned int value);
void LoRa_SetMobileNode(bool enable);
void LoRa_SetReceiveMode(void);


int LoRa_SendData(uint8_t* buffer, uint8_t offset, uint8_t size);

uint8_t LoRa_ReadReg(uint8_t Address);
void LoRa_WriteReg(uint8_t Address, uint8_t WriteData);

//OpMode
#define SLEEP 				0x00
#define	STANDBY				0x01
#define	FSTX					0x02
#define	TX						0x03
#define	FSRX					0x04	
#define RXCONTINUOUS	0x05
#define RXSINGLE			0x06
#define CAD						0x07

//FskMode
#define FSK_Mode			0x00
#define LoRa_Mode			0x80

//Reg Address
#define RegFifo								0x00
#define RegOpMode							0x01
// Unused 0x02
// Unused 0x03
// Unused 0x04
// Unused 0x05
#define RegFrfMsb							0x06
#define RegFrfMid							0x07
#define RegFrfLsb							0x08
#define RegPaConfig						0x09
#define RegPaRamp							0x0A
#define RegOc									0x0B
#define RegLna								0x0C
#define RegFifoAddrPtr				0x0D
#define RegFifoTxBaseAddr 		0x0E
#define RegFifoRxBaseAddr 		0x0F
#define RegFifoRxCurrentAddr	0x10
#define RegIrqFlagsMask				0x11
#define RegIrqFlags						0x12
#define RegRxNbBytes					0x13
#define RegRxHeaderCntValueMsb	0x14
#define RegRxHeaderCntValueLsb	0x15
#define RegRxPacketCntValueMsb	0x16
#define RegRxPacketCntValueLsb 	0x17
#define RegModemStat					0x18
#define RegPktSnrValue				0x19
#define RegPktRssiValue				0x1A
#define RegRssiValue					0x1B
#define RegHopChannel					0x1C
#define RegModemConfig1				0x1D
#define RegModemConfig2				0x1E
#define RegSymbTimeoutLsb			0x1F
#define RegPreambleMsb				0x20
#define RegPreambleLsb				0x21
#define RegPayloadLength			0x22
#define RegMaxPayloadLength		0x23
#define RegHopPeriod					0x24
#define RegFifoRxByteAddr			0x25
#define RegModemConfig3				0x26
// Reserved 0x27
#define RegFeiMsb							0x28
#define RegFeiMid							0x29
#define RegFeiLsb							0x2A
// Reserved 0x2B
#define RegRssiWideband				0x2C
// Reserved 0x2D
// Reserved 0x2E
// Reserved 0x2F
// Reserved 0x3D
#define RegDetectOptimize			0x31
// Reserved 0x32
#define RegInvertIQ						0x33
// Reserved 0x34
// Reserved 0x35
// Reserved 0x36
#define RegDetectionThreshold	0x37
// Reserved 0x38
#define RegSyncWord						0x39
// Reserved 0x3A
// Reserved 0x3B
// Reserved 0x3C
// Reserved 0x3D
// Reserved 0x3F
#define RegDioMapping1				0x40
#define RegDioMapping2				0x41
#define RegVersion						0x42
// Reserved 0x43
// Unused 0x44
// Unused 0x45
// Unused 0x46
// Unused 0x47
// Unused 0x48
// Unused 0x49
// Unused 0x4A
#define RegTcxo								0x4B
#define RegPaDac							0x4D
#define RegFormerTemp					0x5B
// Unused 0x5C
// Unused 0x5D
// Unused 0x5E
// Unused 0x5F
#define RegAgcRef							0x61
#define RegAgcThresh1					0x62
#define RegAgcThresh2					0x63
#define RegAgcThresh3					0x64
// Unused 0x65
// Unused 0x66
// Unused 0x67
// Unused 0x68
// Unused 0x69
// Unused 0x6A
// Unused 0x6B
// Unused 0x6C
// Unused 0x6D
// Unused 0x6E
// Unused 0x6F
#define RegPll								0x70






