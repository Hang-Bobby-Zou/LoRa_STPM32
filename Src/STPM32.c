/**
 * \addtogroup STPM32
 * @{
 * \file STPM32.c
 * \author Hang Bobby Zou
 * \brief STPM32 HAL Code library
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
#include "STPM32.h"
#include "STPM32_AddressMap.h"
#include "ext_flash.h"

/*============================================================================*/
/*                   CRC DEFINES			                                    */
/*============================================================================*/
#define u8 unsigned char
#define CRC_8 (0x07)
#define STPM3x_FRAME_LEN (5)
static char CRC_u8Checksum;
extern uint16_t FlashPointer;

/*============================================================================*/
/*                   PRIVATE DEFINES			                                    */
/*============================================================================*/
//SYN timing defines, all Min.
//#define t_ltch			20 	//ns, Time between de-selection and latch
//#define t_lpw				4 	//us, Latch pulse width
//#define t_w					4		//us, Time between two consecutive latch pulses
//#define t_rel				40	//ns, Time between pulse and selection

// Power-on procedure TYPICAL timing
#define t_if				10	//ms, Time for interface choice locking
#define t_startup 	35	//ms, Time between power on and reset
#define t_rpw				1		//ms, Reset pulse width
#define t_scs				1		//ms, Delay from SYN to SCS


/*============================================================================*/
/*                   PRIVATE FUNCTION PROTOTYPES		                          */
/*============================================================================*/
static u8 CalcCRC8(u8 *pBuf);
static void Crc8Calc (u8 u8Data);
void FRAME_for_UART_mode(u8 *pBuf);
static u8 byteReverse(u8 n);

uint8_t PH_Period								[5] = {0};
uint8_t CH1_RMS									[5] = {0};
uint8_t C1_PHA									[5] = {0};

uint8_t PH1_Active_Energy				[5] = {0};
uint8_t PH1_Fundamental_Energy	[5] = {0};
uint8_t PH1_Reactive_Energy			[5] = {0};
uint8_t PH1_Apparent_Energy			[5] = {0};
		
uint8_t PH1_Active_Power				[5] = {0};
uint8_t PH1_Fundamental_Power		[5] = {0};
uint8_t PH1_Reactive_Power			[5] = {0};
uint8_t	PH1_Apparent_RMS_Power	[5] = {0};

uint8_t Total_Active_Energy			[5] = {0};
uint8_t Total_Fundamental_Energy[5] = {0};
uint8_t Total_Reactive_Energy		[5] = {0};
uint8_t Total_Apparent_Energy		[5] = {0};

/*============================================================================*/
/*                   STPM32 Calculation functions		                          */
/*============================================================================*/
/**
* @brief Calculate and Print the Freq of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_Freq(void){
	uint8_t FlashBuffer [8] = {0};
	
	FlashBuffer [0] = dsp_reg1;
	FlashBuffer [1] = FlashPointer >> 8;
	FlashBuffer [2] = (uint8_t) FlashPointer;
	FlashBuffer [3] = PH_Period[0];
	FlashBuffer [4] = PH_Period[1];
	FlashBuffer [5] = PH_Period[2];
	FlashBuffer [6] = PH_Period[3];
	FlashBuffer [7] = PH_Period[4];
	
	ext_flash_write(FlashPointer + FlashAddr_Freq_Raw, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint16_t freq_raw;

	freq_raw = freq_raw | (uint16_t) PH_Period[1] << 8;
	freq_raw = freq_raw | (uint16_t) PH_Period[2];
	
	static double *freq;
	*freq	= 1.0 / (freq_raw * P_CLK);
	
	ext_flash_write(FlashPointer + FlashAddr_Freq_Real, (char*) freq, 8);
	ext_flash_last_write_or_erase_done();
	
	if( *freq < Freq_Low_Threshold || *freq > Freq_High_Threshold){
		WARN("ERROR: Freq error: %f Hz\r\n\r\n", *freq);		//4 decimal numbers
	} else {
		STPM32_INFO("Freq: %f Hz\r\n\r\n", *freq);		//4 decimal numbers
	}
}

/**
* @brief Calculate and Print the RMS voltage
*		 of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_V1_RMS(void){	
	uint8_t FlashBuffer [8] = {0};
	
	FlashBuffer [0] = dsp_reg14;
	FlashBuffer [1] = FlashPointer >> 8;
	FlashBuffer [2] = (uint8_t) FlashPointer;
	FlashBuffer [3] = CH1_RMS[0];
	FlashBuffer [4] = CH1_RMS[1];
	FlashBuffer [5] = CH1_RMS[2];
	FlashBuffer [6] = CH1_RMS[3];
	FlashBuffer [7] = CH1_RMS[4];
	
	ext_flash_write(FlashPointer + FlashAddr_RMS_Raw, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	static double *V1_RMS;
	
	uint16_t V1_RMS_raw = 0x0000;
	V1_RMS_raw = V1_RMS_raw | ((uint16_t) CH1_RMS[1] << 8);
	V1_RMS_raw = V1_RMS_raw | (uint16_t) CH1_RMS[0];
	V1_RMS_raw = V1_RMS_raw & 0x7FFF; 				//Mask the most significant bit.
	
	*V1_RMS = (double) V1_RMS_raw * (double) V_ref * (1.0 + (double) R1/ (double) R2) / ( (double) cal_v * (double) A_v * 32768.0);	

	ext_flash_write(FlashPointer + FlashAddr_V1_RMS_Real, (char*) V1_RMS, 8);
	ext_flash_last_write_or_erase_done();
	
	STPM32_INFO("V1= %lf Volts\r\n",*V1_RMS);
}

/**
* @brief Calculate and Print the RMS current
*		 of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_C1_RMS(void){	
	
	uint16_t C1_RMS_raw = 0x0000;
	C1_RMS_raw = C1_RMS_raw | ((uint16_t) CH1_RMS[1] >> 7);
	C1_RMS_raw = C1_RMS_raw | ((uint16_t) CH1_RMS[2] << 1);
	C1_RMS_raw = C1_RMS_raw | ((uint16_t) CH1_RMS[3] << 9);

	static double *C1_RMS;
	*C1_RMS = (double) C1_RMS_raw * (double) V_ref / ((double) cal_i * (double) A_i * 131072.0 * (double) k_s * (double) k_int);
	
	ext_flash_write(FlashPointer + FlashAddr_C1_RMS_Real, (char*) C1_RMS, 8);
	ext_flash_last_write_or_erase_done();
	
	STPM32_INFO("C1= %lf Amps\r\n",*C1_RMS);
}

/**
* @brief Calculate and Print the phase delay of voltage and current
*		 for the fundamental harmonic of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_Phase(void){
	uint8_t FlashBuffer [8] = {0};
	
	FlashBuffer [0] = dsp_reg17;
	FlashBuffer [1] = FlashPointer >> 8;
	FlashBuffer [2] = (uint8_t) FlashPointer;
	FlashBuffer [3] = C1_PHA[0];
	FlashBuffer [4] = C1_PHA[1];
	FlashBuffer [5] = C1_PHA[2];
	FlashBuffer [6] = C1_PHA[3];
	FlashBuffer [7] = C1_PHA[4];
	
	ext_flash_write(FlashPointer + FlashAddr_Phase_Raw, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint16_t C1_PHA_raw;
	
	C1_PHA_raw = C1_PHA_raw | (uint16_t) C1_PHA[3] << 8;
	C1_PHA_raw = C1_PHA_raw | (uint16_t) C1_PHA[2];
	C1_PHA_raw = C1_PHA_raw & 0x1FFE;

	static double *phase;
	*phase = (double) C1_PHA_raw / (double) F_CLK * (double) 50 * 360.0;
	
	ext_flash_write(FlashPointer + FlashAddr_Phase_Real, (char*) phase, 8);
	ext_flash_last_write_or_erase_done();
	
	STPM32_INFO("Phase = %lf degrees\r\n\r\n", *phase);
}

/**
* @brief Calculate and Print the active energy of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_Active_Energy(void){
	uint8_t FlashBuffer [8] = {0};
	
	FlashBuffer [0] = ph1_reg1;
	FlashBuffer [1] = FlashPointer >> 8;
	FlashBuffer [2] = (uint8_t) FlashPointer;
	FlashBuffer [3] = PH1_Active_Energy[0];
	FlashBuffer [4] = PH1_Active_Energy[1];
	FlashBuffer [5] = PH1_Active_Energy[2];
	FlashBuffer [6] = PH1_Active_Energy[3];
	FlashBuffer [7] = PH1_Active_Energy[4];
	
	ext_flash_write(FlashPointer + FlashAddr_Active_Energy_Raw, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t Active_Energy_raw = 0x0000;
	Active_Energy_raw = Active_Energy_raw | (uint16_t) PH1_Active_Energy[3] << 24;
	Active_Energy_raw = Active_Energy_raw | (uint16_t) PH1_Active_Energy[2] << 16;
	Active_Energy_raw = Active_Energy_raw | (uint16_t) PH1_Active_Energy[1] << 8;
	Active_Energy_raw = Active_Energy_raw | (uint16_t) PH1_Active_Energy[0];

	static double *Active_Energy;
	*Active_Energy = (double)Active_Energy_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);

	ext_flash_write(FlashPointer + FlashAddr_Active_Energy_Real, (char*) Active_Energy, 8);
	ext_flash_last_write_or_erase_done();
	
	STPM32_INFO("Active Energy = %lf Watts\r\n\r\n", *Active_Energy);
}

/**
* @brief Calculate and Print the fundamental energy of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_Funda_Energy(void){
	uint8_t FlashBuffer [8] = {0};
	
	FlashBuffer [0] = ph1_reg2;
	FlashBuffer [1] = FlashPointer >> 8;
	FlashBuffer [2] = (uint8_t) FlashPointer;
	FlashBuffer [3] = PH1_Fundamental_Energy[0];
	FlashBuffer [4] = PH1_Fundamental_Energy[1];
	FlashBuffer [5] = PH1_Fundamental_Energy[2];
	FlashBuffer [6] = PH1_Fundamental_Energy[3];
	FlashBuffer [7] = PH1_Fundamental_Energy[4];
	
	ext_flash_write(FlashPointer + FlashAddr_Funda_Energy_Raw, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t Funda_Energy_raw = 0x0000;
	Funda_Energy_raw = Funda_Energy_raw | (uint16_t) PH1_Fundamental_Energy[3] << 24;
	Funda_Energy_raw = Funda_Energy_raw | (uint16_t) PH1_Fundamental_Energy[2] << 16;
	Funda_Energy_raw = Funda_Energy_raw | (uint16_t) PH1_Fundamental_Energy[1] << 8;
	Funda_Energy_raw = Funda_Energy_raw | (uint16_t) PH1_Fundamental_Energy[0];

	static double *Funda_Energy;
	*Funda_Energy	= (double)Funda_Energy_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);

	ext_flash_write(FlashPointer + FlashAddr_Funda_Energy_Real, (char*) Funda_Energy, 8);
	ext_flash_last_write_or_erase_done();
	
	STPM32_INFO("Fundamental Energy = %lf Watts\r\n\r\n", *Funda_Energy);
}
/**
* @brief Calculate and Print the reactive energy of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_React_Energy(void){
	uint8_t FlashBuffer [8] = {0};
	
	FlashBuffer [0] = ph1_reg3;
	FlashBuffer [1] = FlashPointer >> 8;
	FlashBuffer [2] = (uint8_t) FlashPointer;
	FlashBuffer [3] = PH1_Reactive_Energy[0];
	FlashBuffer [4] = PH1_Reactive_Energy[1];
	FlashBuffer [5] = PH1_Reactive_Energy[2];
	FlashBuffer [6] = PH1_Reactive_Energy[3];
	FlashBuffer [7] = PH1_Reactive_Energy[4];
	
	ext_flash_write(FlashPointer + FlashAddr_React_Energy_Raw, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t React_Energy_raw = 0x0000;
	React_Energy_raw = React_Energy_raw | (uint16_t) PH1_Reactive_Energy[3] << 24;
	React_Energy_raw = React_Energy_raw | (uint16_t) PH1_Reactive_Energy[2] << 16;
	React_Energy_raw = React_Energy_raw | (uint16_t) PH1_Reactive_Energy[1] << 8;
	React_Energy_raw = React_Energy_raw | (uint16_t) PH1_Reactive_Energy[0];

	static double *React_Energy;
	*React_Energy = (double)React_Energy_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);
	
	ext_flash_write(FlashPointer + FlashAddr_React_Energy_Real, (char*) React_Energy, 8);
	ext_flash_last_write_or_erase_done();
	
	STPM32_INFO("Reactive Energy = %lf Watts\r\n\r\n", *React_Energy);
}
/**
* @brief Calculate and Print the apparent energy of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_App_Energy(void){
	uint8_t FlashBuffer [8] = {0};
	
	FlashBuffer [0] = ph1_reg4;
	FlashBuffer [1] = FlashPointer >> 8;
	FlashBuffer [2] = (uint8_t) FlashPointer;
	FlashBuffer [3] = PH1_Apparent_Energy[0];
	FlashBuffer [4] = PH1_Apparent_Energy[1];
	FlashBuffer [5] = PH1_Apparent_Energy[2];
	FlashBuffer [6] = PH1_Apparent_Energy[3];
	FlashBuffer [7] = PH1_Apparent_Energy[4];
	
	ext_flash_write(FlashPointer + FlashAddr_App_Energy_Raw, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t App_Energy_raw = 0x0000;
	App_Energy_raw = App_Energy_raw | (uint16_t) PH1_Apparent_Energy[3] << 24;
	App_Energy_raw = App_Energy_raw | (uint16_t) PH1_Apparent_Energy[2] << 16;
	App_Energy_raw = App_Energy_raw | (uint16_t) PH1_Apparent_Energy[1] << 8;
	App_Energy_raw = App_Energy_raw | (uint16_t) PH1_Apparent_Energy[0];

	static double *App_Energy;
	*App_Energy = (double)App_Energy_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);

	ext_flash_write(FlashPointer + FlashAddr_App_Energy_Real, (char*) App_Energy, 8);
	ext_flash_last_write_or_erase_done();
	
	STPM32_INFO("Apparent Energy = %lf Watts\r\n\r\n", *App_Energy);
}
/**
* @brief Calculate and Print the active power of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_Active_Pwr(void){
	uint8_t FlashBuffer [8] = {0};
	
	FlashBuffer [0] = ph1_reg5;
	FlashBuffer [1] = FlashPointer >> 8;
	FlashBuffer [2] = (uint8_t) FlashPointer;
	FlashBuffer [3] = PH1_Active_Power[0];
	FlashBuffer [4] = PH1_Active_Power[1];
	FlashBuffer [5] = PH1_Active_Power[2];
	FlashBuffer [6] = PH1_Active_Power[3];
	FlashBuffer [7] = PH1_Active_Power[4];
	
	ext_flash_write(FlashPointer + FlashAddr_Active_Power_Raw, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t Active_Pwr_raw = 0x00000000;
	Active_Pwr_raw = Active_Pwr_raw | (uint16_t) PH1_Active_Power[3] << 24;
	Active_Pwr_raw = Active_Pwr_raw | (uint16_t) PH1_Active_Power[2] << 16;
	Active_Pwr_raw = Active_Pwr_raw | (uint16_t) PH1_Active_Power[1] << 8;
	Active_Pwr_raw = Active_Pwr_raw | (uint16_t) PH1_Active_Power[0];

	Active_Pwr_raw = Active_Pwr_raw & 0x1FFFFFFF;

	static double *Active_Pwr;
	*Active_Pwr = (double)Active_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / (3600.0 * (double)D_CLK * (double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 131072.0);

	ext_flash_write(FlashPointer + FlashAddr_Active_Power_Real, (char*) Active_Pwr, 8);
	ext_flash_last_write_or_erase_done();
	
	STPM32_INFO("Active Power = %lf WattHrs\r\n\r\n", *Active_Pwr);
}
/**
* @brief Calculate and Print the fundamental power of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_Funda_Pwr(void){
	uint8_t FlashBuffer [8] = {0};
	
	FlashBuffer [0] = ph1_reg6;
	FlashBuffer [1] = FlashPointer >> 8;
	FlashBuffer [2] = (uint8_t) FlashPointer;
	FlashBuffer [3] = PH1_Fundamental_Power[0];
	FlashBuffer [4] = PH1_Fundamental_Power[1];
	FlashBuffer [5] = PH1_Fundamental_Power[2];
	FlashBuffer [6] = PH1_Fundamental_Power[3];
	FlashBuffer [7] = PH1_Fundamental_Power[4];
	
	ext_flash_write(FlashPointer + FlashAddr_Funda_Pwr_Raw, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t Funda_Pwr_raw = 0x00000000;
	Funda_Pwr_raw = Funda_Pwr_raw | (uint16_t) PH1_Fundamental_Power[3] << 24;
	Funda_Pwr_raw = Funda_Pwr_raw | (uint16_t) PH1_Fundamental_Power[2] << 16;
	Funda_Pwr_raw = Funda_Pwr_raw | (uint16_t) PH1_Fundamental_Power[1] << 8;
	Funda_Pwr_raw = Funda_Pwr_raw | (uint16_t) PH1_Fundamental_Power[0];

	Funda_Pwr_raw = Funda_Pwr_raw & 0x1FFFFFFF;

	static double *Funda_Pwr;
	*Funda_Pwr = (double)Funda_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / (3600.0 * (double)D_CLK * (double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 131072.0);
	
	ext_flash_write(FlashPointer + FlashAddr_Funda_Pwr_Real, (char*) Funda_Pwr, 8);
	ext_flash_last_write_or_erase_done();
	
	STPM32_INFO("Fundamental Power = %lf WattHrs\r\n\r\n", *Funda_Pwr);
}
/**
* @brief Calculate and Print the reactive power of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_React_Pwr(void){
	uint8_t FlashBuffer [8] = {0};
	
	FlashBuffer [0] = ph1_reg7;
	FlashBuffer [1] = FlashPointer >> 8;
	FlashBuffer [2] = (uint8_t) FlashPointer;
	FlashBuffer [3] = PH1_Reactive_Power[0];
	FlashBuffer [4] = PH1_Reactive_Power[1];
	FlashBuffer [5] = PH1_Reactive_Power[2];
	FlashBuffer [6] = PH1_Reactive_Power[3];
	FlashBuffer [7] = PH1_Reactive_Power[4];
	
	ext_flash_write(FlashPointer + FlashAddr_React_Pwr_Raw, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t React_Pwr_raw = 0x00000000;
	React_Pwr_raw = React_Pwr_raw | (uint16_t) PH1_Reactive_Power[3] << 24;
	React_Pwr_raw = React_Pwr_raw | (uint16_t) PH1_Reactive_Power[2] << 16;
	React_Pwr_raw = React_Pwr_raw | (uint16_t) PH1_Reactive_Power[1] << 8;
	React_Pwr_raw = React_Pwr_raw | (uint16_t) PH1_Reactive_Power[0];

	React_Pwr_raw = React_Pwr_raw & 0x1FFFFFFF;

	static double *React_Pwr;
	*React_Pwr = (double)React_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / (3600.0 * (double)D_CLK * (double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 131072.0);
	
	ext_flash_write(FlashPointer + FlashAddr_React_Pwr_Real, (char*) React_Pwr, 8);
	ext_flash_last_write_or_erase_done();
	
	STPM32_INFO("Reactive Power = %lf WattHrs\r\n\r\n", *React_Pwr);
}
/**
* @brief Calculate and Print the apparent RMS power of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_App_RMS_Pwr(void){
	uint8_t FlashBuffer [8] = {0};
	
	FlashBuffer [0] = ph1_reg8;
	FlashBuffer [1] = FlashPointer >> 8;
	FlashBuffer [2] = (uint8_t) FlashPointer;
	FlashBuffer [3] = PH1_Apparent_RMS_Power[0];
	FlashBuffer [4] = PH1_Apparent_RMS_Power[1];
	FlashBuffer [5] = PH1_Apparent_RMS_Power[2];
	FlashBuffer [6] = PH1_Apparent_RMS_Power[3];
	FlashBuffer [7] = PH1_Apparent_RMS_Power[4];
	
	ext_flash_write(FlashPointer + FlashAddr_App_RMS_Pwr_Raw, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t App_RMS_Pwr_raw = 0x00000000;
	App_RMS_Pwr_raw = App_RMS_Pwr_raw | (uint16_t) PH1_Apparent_RMS_Power[3] << 24;
	App_RMS_Pwr_raw = App_RMS_Pwr_raw | (uint16_t) PH1_Apparent_RMS_Power[2] << 16;
	App_RMS_Pwr_raw = App_RMS_Pwr_raw | (uint16_t) PH1_Apparent_RMS_Power[1] << 8;
	App_RMS_Pwr_raw = App_RMS_Pwr_raw | (uint16_t) PH1_Apparent_RMS_Power[0];

	App_RMS_Pwr_raw = App_RMS_Pwr_raw & 0x1FFFFFFF;

	static double *Apparent_RMS_Power;
	*Apparent_RMS_Power = (double)App_RMS_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / (3600.0 * (double)D_CLK * (double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 131072.0);
	
	ext_flash_write(FlashPointer + FlashAddr_App_RMS_Pwr_Real, (char*) Apparent_RMS_Power, 8);
	ext_flash_last_write_or_erase_done();
	
	STPM32_INFO("Apparent_RMS Power = %lf WattHrs\r\n\r\n", *Apparent_RMS_Power);
}

/**
* @brief Calculate and Print the total active energy of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_Tot_Active_Energy(void){
	uint8_t FlashBuffer [8] = {0};
	
	FlashBuffer [0] = tot_reg1;
	FlashBuffer [1] = FlashPointer >> 8;
	FlashBuffer [2] = (uint8_t) FlashPointer;
	FlashBuffer [3] = Total_Active_Energy[0];
	FlashBuffer [4] = Total_Active_Energy[1];
	FlashBuffer [5] = Total_Active_Energy[2];
	FlashBuffer [6] = Total_Active_Energy[3];
	FlashBuffer [7] = Total_Active_Energy[4];
	
	ext_flash_write(FlashPointer + FlashAddr_Tot_Active_Energy_Raw, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t Tot_Active_Pwr_raw = 0x00000000;
	Tot_Active_Pwr_raw = Tot_Active_Pwr_raw | (uint16_t) Total_Active_Energy[3] << 24;
	Tot_Active_Pwr_raw = Tot_Active_Pwr_raw | (uint16_t) Total_Active_Energy[2] << 16;
	Tot_Active_Pwr_raw = Tot_Active_Pwr_raw | (uint16_t) Total_Active_Energy[1] << 8;
	Tot_Active_Pwr_raw = Tot_Active_Pwr_raw | (uint16_t) Total_Active_Energy[0];

	static double *Tot_Active_Pwr;
	*Tot_Active_Pwr = (double)Tot_Active_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);
	
	ext_flash_write(FlashPointer + FlashAddr_Tot_Active_Energy_Real, (char*) Tot_Active_Pwr, 8);
	ext_flash_last_write_or_erase_done();
	
	STPM32_INFO("Total Active Energy = %lf Watts\r\n\r\n", *Tot_Active_Pwr);
}
/**
* @brief Calculate and Print the total fundamental energy of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_Tot_Funda_Energy(void){
	uint8_t FlashBuffer [8] = {0};
	
	FlashBuffer [0] = tot_reg2;
	FlashBuffer [1] = FlashPointer >> 8;
	FlashBuffer [2] = (uint8_t) FlashPointer;
	FlashBuffer [3] = Total_Fundamental_Energy[0];
	FlashBuffer [4] = Total_Fundamental_Energy[1];
	FlashBuffer [5] = Total_Fundamental_Energy[2];
	FlashBuffer [6] = Total_Fundamental_Energy[3];
	FlashBuffer [7] = Total_Fundamental_Energy[4];
	
	ext_flash_write(FlashPointer + FlashAddr_Tot_Funda_Energy_Raw, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t Tot_Funda_Pwr_raw = 0x00000000;
	Tot_Funda_Pwr_raw = Tot_Funda_Pwr_raw | (uint16_t) Total_Fundamental_Energy[3] << 24;
	Tot_Funda_Pwr_raw = Tot_Funda_Pwr_raw | (uint16_t) Total_Fundamental_Energy[2] << 16;
	Tot_Funda_Pwr_raw = Tot_Funda_Pwr_raw | (uint16_t) Total_Fundamental_Energy[1] << 8;
	Tot_Funda_Pwr_raw = Tot_Funda_Pwr_raw | (uint16_t) Total_Fundamental_Energy[0];

	static double *Tot_Funda_Pwr;
	*Tot_Funda_Pwr = (double)Tot_Funda_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);
	
	ext_flash_write(FlashPointer + FlashAddr_Tot_Funda_Energy_Real, (char*) Tot_Funda_Pwr, 8);
	ext_flash_last_write_or_erase_done();
	
	STPM32_INFO("Total Fundamental Energy = %lf Watts\r\n\r\n", *Tot_Funda_Pwr);
}
/**
* @brief Calculate and Print the total reactive energy of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_Tot_React_Energy(void){
	uint8_t FlashBuffer [8] = {0};
	
	FlashBuffer [0] = tot_reg3;
	FlashBuffer [1] = FlashPointer >> 8;
	FlashBuffer [2] = (uint8_t) FlashPointer;
	FlashBuffer [3] = Total_Reactive_Energy[0];
	FlashBuffer [4] = Total_Reactive_Energy[1];
	FlashBuffer [5] = Total_Reactive_Energy[2];
	FlashBuffer [6] = Total_Reactive_Energy[3];
	FlashBuffer [7] = Total_Reactive_Energy[4];
	
	ext_flash_write(FlashPointer + FlashAddr_Tot_React_Energy_Raw, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t Tot_React_Pwr_raw = 0x00000000;
	Tot_React_Pwr_raw = Tot_React_Pwr_raw | (uint16_t) Total_Reactive_Energy[3] << 24;
	Tot_React_Pwr_raw = Tot_React_Pwr_raw | (uint16_t) Total_Reactive_Energy[2] << 16;
	Tot_React_Pwr_raw = Tot_React_Pwr_raw | (uint16_t) Total_Reactive_Energy[1] << 8;
	Tot_React_Pwr_raw = Tot_React_Pwr_raw | (uint16_t) Total_Reactive_Energy[0];

	static double *Tot_React_Pwr;
	*Tot_React_Pwr = (double)Tot_React_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);
	
	ext_flash_write(FlashPointer + FlashAddr_Tot_React_Energy_Real, (char*) Tot_React_Pwr, 8);
	ext_flash_last_write_or_erase_done();
	
	STPM32_INFO("Total Reactive Energy = %f Watts\r\n\r\n", *Tot_React_Pwr);
}
/**
* @brief Calculate and Print the total apparent energy of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_Tot_App_Energy(void){
	uint8_t FlashBuffer [8] = {0};
	
	FlashBuffer [0] = tot_reg4;
	FlashBuffer [1] = FlashPointer >> 8;
	FlashBuffer [2] = (uint8_t) FlashPointer;
	FlashBuffer [3] = Total_Apparent_Energy[0];
	FlashBuffer [4] = Total_Apparent_Energy[1];
	FlashBuffer [5] = Total_Apparent_Energy[2];
	FlashBuffer [6] = Total_Apparent_Energy[3];
	FlashBuffer [7] = Total_Apparent_Energy[4];
	
	ext_flash_write(FlashPointer + FlashAddr_Tot_App_Energy_Raw, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t Tot_App_Pwr_raw = 0x00000000;
	Tot_App_Pwr_raw = Tot_App_Pwr_raw | (uint16_t) Total_Apparent_Energy[3] << 24;
	Tot_App_Pwr_raw = Tot_App_Pwr_raw | (uint16_t) Total_Apparent_Energy[2] << 16;
	Tot_App_Pwr_raw = Tot_App_Pwr_raw | (uint16_t) Total_Apparent_Energy[1] << 8;
	Tot_App_Pwr_raw = Tot_App_Pwr_raw | (uint16_t) Total_Apparent_Energy[0];

	static double *Tot_App_Pwr;
	*Tot_App_Pwr = (double)Tot_App_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);
	
	ext_flash_write(FlashPointer + FlashAddr_Tot_App_Energy_Real, (char*) Tot_App_Pwr, 8);
	ext_flash_last_write_or_erase_done();
	
	STPM32_INFO("Total Apparent Energy = %lf Watts\r\n\r\n", *Tot_App_Pwr);
}


/*============================================================================*/
/*                   STPM32 UpperLevel Functions		                          */
/*============================================================================*/
/**
  * @brief  Initialize STPM32 by toggle GPIO
	* @intval	NONE
  * @retval NONE
  */
bool STPM32_Init(void) {
		// Initializing STPM32
		/* Sequence: 	0. Before initializing
									1. Enable Pin set low
									2. Enable Pin set high
									3. Wait for t_startup time or PowrOK signal 
									4. Three SYN low signal of time t_rpw
		
									5. Configure all parameters (not a lot)
									6. Communicate:
											8.1 Tx: Read Address, Write Address, LS Data[7:0], MS Data[15:8], CRC Byte 
											8.2 Rx: Data[7:0], Data[15:8], Data[23:16], Data[31:24], CRC Byte
											8.2 Dummy read address 0xFF increments by one the internal read pointer
											8.4 Dummy write address 0xFF specifies that no writing is requested
									7. Other: 
											BREAK frame: if received, a break flag is set and the whole packet reception aborts
											IDLE frame: the receiver can recognize an IDLE frame
											LATCH: 	1. One SYN pulse with SCS set to high. 
															2. Write to channel latch bits before read (S/W Latchx in DSP_CR3).
															3. Writing auto-latch bit (S/W Auto Latch in DSP_CR3)
		*/
		//First set everything low.
		HAL_GPIO_WritePin(CTRL_EN_GPIO_Port, CTRL_EN_Pin, GPIO_PIN_SET);				// EN	High
		
		HAL_Delay(500);
		
		HAL_GPIO_WritePin(CTRL_EN_GPIO_Port, CTRL_EN_Pin, GPIO_PIN_RESET);			// EN	Low
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_RESET);	// SYNC Low
		HAL_GPIO_WritePin(CTRL_SCS_GPIO_Port, CTRL_SCS_Pin, GPIO_PIN_RESET);		// SCS Low
		
		HAL_Delay(100);
		//Then set SYNC and SCS to high before EN starts
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_SET);		// SYNC High
		HAL_GPIO_WritePin(CTRL_SCS_GPIO_Port,CTRL_SCS_Pin,GPIO_PIN_SET);				// SCS High
		
		HAL_Delay(100);
		//EN reset
		HAL_GPIO_WritePin(CTRL_EN_GPIO_Port, CTRL_EN_Pin, GPIO_PIN_SET);				// EN High
		
		
		/*SCS signal is used to reset communication peripheral and SYN signal is used to reset the DSP. 
			These reset pulses must be generated once, just after the power-on sequence. */
			
		//Startup reset
		HAL_Delay(t_startup);
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_RESET);
		HAL_Delay(t_rpw/2);
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_SET);
		HAL_Delay(t_rpw/2);
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_RESET);
		HAL_Delay(t_rpw/2);
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_SET);
		HAL_Delay(t_rpw/2);
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_RESET);
		HAL_Delay(t_rpw/2);
		HAL_GPIO_WritePin(CTRL_SYNC_GPIO_Port, CTRL_SYNC_Pin, GPIO_PIN_SET);
		
		HAL_Delay(t_scs);
		HAL_GPIO_WritePin(CTRL_SCS_GPIO_Port, CTRL_SCS_Pin, GPIO_PIN_RESET);
		HAL_Delay(t_rpw/2);
		HAL_GPIO_WritePin(CTRL_SCS_GPIO_Port, CTRL_SCS_Pin, GPIO_PIN_SET);
		// All hardware reset is done
		
		
		//Configure DSP_CR3, read address 0x04 (Row2), write 0xABCD to 0x05 
		uint8_t SentMsg [2] = {0};
		
		SentMsg[0] = 0x03;
		SentMsg[1] = 0x80;
		
		//for (int i = 0; i < 2; i++){
			SendMsgOnly (dsp_cr3 + 0x01, SentMsg);
		//}

		DEBUG("INIT done");
		return true;
}

/**
  * @brief  SendMessage to STPM32 only
	* @intval	Address in STPM32 register to be written
	*					Message to be send to the register
  * @retval True if success / False if fail
  */
bool SendMsgOnly (uint32_t SendAddress, uint8_t* SendMessage){
	/* 
		| ReadAddress | WriteAddress | LS Data [7:0] | MS Data [15:8] | CRC Byte |
		|		 0x00 		|		 Address	 | 	 Message[0]  |   Message[1]   |    --    |
	*/
	uint8_t ReadMessage[5] = {0};
	uint8_t Buffer[5] = {0};
	uint8_t CRCBuffer[5] = {0};
	
	Buffer[0] = 0x00; //This will automatically increment read pointer by 1
	Buffer[1] = SendAddress;
	Buffer[2] = SendMessage[1];
	Buffer[3] = SendMessage[0];
	
	CRCBuffer[0] = byteReverse(Buffer[0]);
	CRCBuffer[1] = byteReverse(Buffer[1]);
	CRCBuffer[2] = byteReverse(Buffer[2]);
	CRCBuffer[3] = byteReverse(Buffer[3]);
	Buffer[4] = byteReverse(CalcCRC8(CRCBuffer));
	
	if (USART1_RxFlag == 0){
		HAL_UART_Receive_IT(&huart1, (uint8_t*) ReadMessage, 5);
	}
	
	HAL_UART_Transmit(&huart1, (uint8_t*) Buffer, 5, 0xFFFF);

	HAL_Delay(1);
	
	return true;
}

/**
  * @brief  ReadMessage from STPM32 only
	* @intval	Address in STPM32 register to be read
	*					Message read from the register
  * @retval True if success / False if fail
  */
bool ReadMsgOnly (uint32_t ReadAddress, uint8_t* ReadMessage){
	/* 
		| ReadAddress | WriteAddress | LS Data [7:0] | MS Data [15:8] | CRC Byte |
		|		 0xFF 		|		  0xFF		 | 	 	 0xFF 		 |     0xFF				|    --    |
	*/
	uint8_t Buffer[5] = {0};
	uint8_t CRCBuffer[5] = {0};
	
	Buffer[0] = ReadAddress;
	Buffer[1] = 0xFF;
	Buffer[2] = 0xFF;
	Buffer[3] = 0xFF;
	
	CRCBuffer[0] = byteReverse(Buffer[0]);
	CRCBuffer[1] = byteReverse(Buffer[1]);
	CRCBuffer[2] = byteReverse(Buffer[2]);
	CRCBuffer[3] = byteReverse(Buffer[3]);
	Buffer[4] = byteReverse(CalcCRC8(CRCBuffer));
	
	
	if (USART1_RxFlag == 0){
		HAL_UART_Receive_IT(&huart1, (uint8_t*) ReadMessage, 5);
	}
	
	HAL_UART_Transmit(&huart1, (uint8_t*) Buffer, 5, 0xFFFF);
	
	HAL_Delay(1);	//Delay for 1 ms between each transmit to avoid any loss of data.
	
	return true;
}

/*===================================================================== */
/*					CRC Calculations 																						*/
/*===================================================================== */
static u8 CalcCRC8(u8 *pBuf)
{
	u8 i;
	CRC_u8Checksum = 0x00;
	for (i=0; i<STPM3x_FRAME_LEN-1; i++)
	{
	Crc8Calc(pBuf[i]);
	}
	return CRC_u8Checksum;
}

static void Crc8Calc (u8 u8Data){
	u8 loc_u8Idx;
	u8 loc_u8Temp;
	loc_u8Idx=0;
	while(loc_u8Idx<8)
	{
		loc_u8Temp = u8Data^CRC_u8Checksum;
		CRC_u8Checksum<<=1;
		if(loc_u8Temp&0x80)
		{
		CRC_u8Checksum^=CRC_8;
		}
		u8Data<<=1;
		loc_u8Idx++;
	}
}

void FRAME_for_UART_mode(u8 *pBuf){
	u8 temp[4],x,CRC_on_reversed_buf;
	for (x=0;x<(STPM3x_FRAME_LEN-1);x++){
		temp[x] = byteReverse(pBuf[x]);
	}
	CRC_on_reversed_buf = CalcCRC8(temp);
	pBuf[4] = byteReverse(CRC_on_reversed_buf);
}

static u8 byteReverse(u8 n){
	n = ((n >> 1) & 0x55) | ((n << 1) & 0xaa);
	n = ((n >> 2) & 0x33) | ((n << 2) & 0xcc);
	n = ((n >> 4) & 0x0F) | ((n << 4) & 0xF0);
	return n;
}
