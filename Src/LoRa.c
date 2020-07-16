/**
 * \addtogroup LoRa
 * @{
 * \file LoRa.c
 * \author Hang Bobby Zou
 * \brief LoRa Code library
 */
 
/*============================================================================*/
/*                   STANDARD INCLUDE FILES                                   */
/*============================================================================*/
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>		// To include uintXX_t type
#include <stdbool.h>	// To include the bool type
#include <stdio.h>

/*============================================================================*/
/*                   INCLUDE FILES                                            */
/*============================================================================*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "main.h"
#include "usart.h"
#include "HAL_spi.h"
#include "LoRa.h"

uint8_t SPI2_RxBuffer[2] = {0};
uint8_t SPI2_TxBuffer[2] = {0};

bool LoRa_Init(void){
	
	/*
		The first byte is the address byte. It is comprises:
		` A wnr bit, which is 1 for write access and 0 for read access
		` Then 7 bits of address, MSB first
	*/
	// Enable Power LoRa
	HAL_GPIO_WritePin(CMD_PWR_LORA_GPIO_Port, CMD_PWR_LORA_Pin, GPIO_PIN_SET);
	
	// Set NSS pin to default high
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	
	// Do a NRESET cycle and kept NREST 1278 high for the reset of the time.
	HAL_GPIO_WritePin(NRST_1278_GPIO_Port, NRST_1278_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(NRST_1278_GPIO_Port, NRST_1278_Pin, GPIO_PIN_SET);
	
	
	// Assmue read LoRa ID: 0(MSB)+0x42
	USART3_PINSET_TX();
	
	// Wait until the ID of the LoRa chip is read from address 0x42
	if (LoRa_is_detected() != true)
		Error_Handler();
	
	//Set OpMode: SLEEP
	LoRa_SetOpMode(SLEEP);
	
	//Set FskMode: LoRa Mode
	LoRa_SetFskMode(LoRa_Mode);
	
	//Set OpMode: STANDBY
	LoRa_SetOpMode(STANDBY);
	
	//Set Port Mapping of DIO1-3 & DIO4-5
	LoRa_WriteReg(RegDioMapping1, 0x00);
	LoRa_WriteReg(RegDioMapping2, 0x00);
	
	//Set RF Freq
	//470.3MHz ~ 510MHZ
	LoRa_SetRFFrequency(470300000);
	
	//Set RF Power ??
	//LoRa_SetRFPower(0x00);
	
	//Set spreading factor ??
	//LoRa_SetSpreadingFactor(0x07);
	
	uint8_t buffer[4] = {0};
	
	buffer[0] = 0x01;
	buffer[1] = 0x01;
	buffer[2] = 0x01;
	buffer[3] = 0xAA;
	
	
	for(int i = 0; i< 100; i++){
		LoRa_SendData(buffer, 0, sizeof(buffer));
		
		HAL_Delay(100);
		
	}

	USART3_PINSET_RX();
	
	return true;
}


int LoRa_SendData(uint8_t* buffer, uint8_t offset, uint8_t size){

	LoRa_SetOpMode(STANDBY);
	
	LoRa_WriteReg(RegHopPeriod, 0);		//Disable hopper period
	
	LoRa_WriteReg(RegDioMapping1, 0x40);
	LoRa_WriteReg(RegDioMapping2, 0x00);
	
	LoRa_WriteReg(RegIrqFlags, 0xFF);
	LoRa_WriteReg(RegIrqFlagsMask, 0x08);
	
	LoRa_WriteReg(RegPayloadLength, size);
	LoRa_WriteReg(RegFifoTxBaseAddr, 0);
	LoRa_WriteReg(RegFifoAddrPtr, 0);
	
	
	HAL_GPIO_WritePin(NRST_1278_GPIO_Port, NRST_1278_Pin, GPIO_PIN_RESET);
		uint8_t buff[1] = {0x80};
		if(HAL_SPI_Transmit(&hspi2, &buff[0], 1, 5) != HAL_OK){
			Error_Handler();
		}
		
		for (int i = 0; i < size; i++) {
			if(HAL_SPI_Transmit(&hspi2, &buffer[i+offset], 1, 5) != HAL_OK){
				Error_Handler();
			}
		}

		HAL_GPIO_WritePin(NRST_1278_GPIO_Port, NRST_1278_Pin, GPIO_PIN_SET);
	
	LoRa_SetOpMode(TX);
	
	int timeover = 1000;
	
	while (timeover-- > 0){
		if ((LoRa_ReadReg(RegIrqFlags) & 0x08) == 0x08){
			LoRa_SetReceiveMode(); 
			return 0;
		}
		
		HAL_Delay(3);
	}
	LoRa_SetReceiveMode(); 
	return -1;
	
	
}


void LoRa_SetReceiveMode(void){
	
	LoRa_SetOpMode(STANDBY);
	
	//LoRa_WriteReg(RegHopPeriod, 0);		??
	
	LoRa_WriteReg(RegDioMapping1, 0x00);
	LoRa_WriteReg(RegDioMapping2, 0x00);
	
	LoRa_WriteReg(RegIrqFlags, 0xFF);
	LoRa_WriteReg(RegIrqFlagsMask, 0x40);
	
	LoRa_SetOpMode(RXSINGLE);
	
}



/**
* @brief Set LoRa chip Low data rate on/off
* @param Parameter: on/off
* @retval None
*/
void LoRa_SetMobileNode(bool enable){
	uint8_t data;
	
	data = LoRa_ReadReg(RegModemConfig3);
	
	data &= 0xF7;
	
	data |= (enable ? 1 : 0) << 3;
	
	LoRa_WriteReg(RegModemConfig3, data);
	
}

/**
* @brief Set LoRa chip operation time out
* @param Parameter: time out value
* @retval None
*/
void LoRa_SetSymbTimeout(unsigned int value){
	uint8_t buffer[2];
	
	buffer[0] = LoRa_ReadReg(RegModemConfig2);
	
	buffer[1] = LoRa_ReadReg(RegSymbTimeoutLsb);
	
	buffer[0] &= 0xFC;
	buffer[0] |= value >> 8;
	
	buffer[1] = value & 0xFF;
	
	LoRa_WriteReg(RegModemConfig2,buffer[0]);
	LoRa_WriteReg(RegSymbTimeoutLsb, buffer[1]);
	
}

/**
* @brief Set LoRa chip payload length
* @param Parameter: length of payload
* @retval None
*/
void LoRa_SetPayloadLength(uint8_t value){
	LoRa_WriteReg(RegPayloadLength, value);
}

/**
* @brief Set LoRa chip implicit header on/off
* @param Parameter: on/off
* @retval None
*/
void LoRa_SetImplicitHeaderOn(bool enable){
	uint8_t data;
	
	data = LoRa_ReadReg(RegModemConfig1);
	
	data &= 0xFE;
	
	data |= (enable ? 1 : 0);
	
	LoRa_WriteReg(RegModemConfig1, data);
}


/**
* @brief Set LoRa chip bandwidth
* @param Parameter: bandwidth
* @retval None
*/
void LoRa_SetSignalBandwidth(uint8_t bw){
	uint8_t data;
	
	data = LoRa_ReadReg(RegModemConfig1);
	
	data &= 0x0F;
	
	data |= bw << 4;
	
	LoRa_WriteReg(RegModemConfig1, data);
	
}

/**
* @brief Set LoRa chip crc on/off
* @param Parameter: enable/disable
* @retval None
*/
void LoRa_SetPacketCrcOn(bool enable){
	uint8_t data;
	
	data = LoRa_ReadReg(RegModemConfig2);
	
	data &= 0xFB;
	
	data |= (enable ? 1:0) << 2;
	
	LoRa_WriteReg(RegModemConfig2, data);
	
}

/**
* @brief Set LoRa chip error coding rate
* @param Parameter: error code rate
* @retval None
*/
void LoRa_SetErrorCoding(uint8_t value){
	uint8_t data;
	
	data = LoRa_ReadReg(RegModemConfig1);
	
	data &= 0xF1;
	
	data |= value << 1;
	
	LoRa_WriteReg(RegModemConfig1, data);
	
}

/**
* @brief Set LoRa chip spreading factor
* @param Parameter: spreading factor
* @retval None
*/
void LoRa_SetSpreadingFactor(uint8_t factor){
	uint8_t data;
	
	data = LoRa_ReadReg(RegModemConfig2);
	
	data &= 0x0F;
	
	data |= factor<<4;
	
	LoRa_WriteReg(RegModemConfig2, data);
	
}

/**
* @brief Set LoRa chip RF Power
* @param Parameter: power
* @retval None
*/
void LoRa_SetRFPower(uint8_t power){
	//LoRa_WriteReg(RegPaConfig, 0x00);
	//LoRa_WriteReg(RegPaDac,0x04);
	
}

/**
* @brief Set LoRa chip RF Frequency
* @param Parameter: freq
* @retval None
*/
void LoRa_SetRFFrequency(uint32_t freq){
	uint32_t value = (uint32_t)((double)freq / 61.03515625);
	
	uint8_t buffer[3];
	
	buffer[0] = (uint8_t)(value >> 16 & 0xFF);
	
	buffer[1] = (uint8_t)(value >> 8 & 0xFF);
	
	buffer[2] = (uint8_t)(value >> 0 & 0xFF);
	
	LoRa_WriteReg(RegFrfMsb, buffer[0]);
	LoRa_WriteReg(RegFrfMid, buffer[1]);
	LoRa_WriteReg(RegFrfLsb, buffer[2]);
	
}

/**
* @brief Set LoRa chip modulation mode by writing to RegOpMode
* @param Parameter: Modulation Mode (LoRa.h)
* @retval None
*/
void LoRa_SetFskMode(uint8_t ModulationMode){
	uint8_t opModePrev;
	
	opModePrev = LoRa_ReadReg(RegOpMode);
	
	opModePrev &= 0x7F;
	opModePrev |= ModulationMode;
	
	LoRa_WriteReg(RegOpMode, opModePrev);
}


/**
* @brief Set LoRa chip operating mode by writing to RegOpMode
* @param Parameter: Operating Mode (LoRa.h)
* @retval None
*/
void LoRa_SetOpMode(uint8_t OperatingMode){
	uint8_t opModePrev;
	
	opModePrev = LoRa_ReadReg(RegOpMode);
	
	opModePrev &= 0xF8;
	
	opModePrev |= OperatingMode;
	
	LoRa_WriteReg(RegOpMode, opModePrev);
	
}

/**
* @brief Check if LoRa chip is detected by reading its ID
* @param Parameter: None
* @retval True/False after 3 trials
*/
bool LoRa_is_detected(void){
	int count = 0;
	
	do{	
		myprintf("Checking LoRa Device ID\r\n\r\n");
		
		SPI2_TxBuffer[0] = RegVersion;
		
		SPI2_RxBuffer[0] = LoRa_ReadReg(SPI2_TxBuffer[0]);
		
		myprintf("SPI2 ID: %x %x \r\n\r\n", SPI2_RxBuffer[0], SPI2_RxBuffer[1]);

		count++;
	} while (SPI2_RxBuffer[0] != 0x12 && count < 3);
	
	if (count < 3){
		return true;
	} else {
		return false;
	}
}


















uint8_t LoRa_ReadReg(uint8_t Address){
	uint8_t ReturnVal;
	
	uint8_t Addr[1] = {Address};
	Addr[0] = Addr[0] & 0x7F;
	
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
		
	HAL_Delay(1);
		
	if(HAL_SPI_Transmit(&hspi2, &Addr[0], 1, 5) != HAL_OK){
		Error_Handler();
	}
		
	if(HAL_SPI_Receive(&hspi2, &ReturnVal,1, 5) != HAL_OK){
		Error_Handler();
	}
		
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	
	return ReturnVal;
	
}

void LoRa_WriteReg(uint8_t Address, uint8_t WriteData){

	uint8_t Addr[1] = {Address};
	Addr[0] = Addr[0] | 0x80;
	
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
		
	HAL_Delay(1);
		
	if(HAL_SPI_Transmit(&hspi2, &Addr[0], 1, 5) != HAL_OK){
		Error_Handler();
	}
	
	if(HAL_SPI_Transmit(&hspi2, &WriteData, 1, 5) != HAL_OK){
		Error_Handler();
	}

	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
	
}


