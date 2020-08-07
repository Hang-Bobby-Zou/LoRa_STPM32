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
/*                   PRIVATE INCLUDES                                         */
/*============================================================================*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "main.h"
#include "usart.h"
#include "HAL_spi.h"
#include "LoRa.h"
#include "HAL_LoRaMAC.h"
#include "sx1276.h"

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
	
	//LoRa_SetOpMode(TX);

	return true;
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
	uint8_t SPI2_RxBuffer[2] = {0};
	uint8_t SPI2_TxBuffer[2] = {0};
	
	do{	
		DEBUG("Checking LoRa Device ID...");
		
		SPI2_TxBuffer[0] = RegVersion;
		
		SPI2_RxBuffer[0] = LoRa_ReadReg(SPI2_TxBuffer[0]);

		DEBUG("SPI2 ID: %x %x ", SPI2_RxBuffer[0], SPI2_RxBuffer[1]);

		count++;
	} while (SPI2_RxBuffer[0] != 0x12 && count < 3);
	
	if (count < 3){
		return true;
	} else {
		WARN("LoRa is not detected...");
		return false;
	}
	
}

/**
* @brief Read from a specific register from sx1276
* @param The address of register to be read
* @retval uin8_t: the register value of the register
*/
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

/**
* @brief Write to a specific register from sx1276
* @param The address of register to be written
* @retval none
*/
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
