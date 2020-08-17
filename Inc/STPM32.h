/*============================================================================*/
/*                   STANDARD INCLUDE FILES                                   */
/*============================================================================*/
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>		// To include uintXX_t type
#include <stdbool.h>	// To include the bool type

#define R1									810000
#define R2 									470

#define F_CLK								125000
#define P_CLK								0.000008		//  1 / F_CLK
#define D_CLK								7812.5 

#define V_ref 							1.18

#define A_v 								2
#define A_i 								2
#define cal_v 							0.875
#define cal_i								0.875

#define k_s									0.0024
#define k_int								1

#define Freq_Low_Threshold	45.0
#define Freq_High_Threshold	65.0

#define V1_Low_Threshold 		200.0
#define V1_High_Threshold		270.0

void CalcPrint_Freq(void);
void CalcPrint_V1_RMS(void);
void CalcPrint_C1_RMS(void);
void CalcPrint_Phase(void);
void CalcPrint_Active_Energy(void);
void CalcPrint_Funda_Energy(void);
void CalcPrint_React_Energy(void);
void CalcPrint_App_Energy(void);
void CalcPrint_Active_Pwr(void);
void CalcPrint_Funda_Pwr(void);
void CalcPrint_React_Pwr(void);
void CalcPrint_App_RMS_Pwr(void);
void CalcPrint_Tot_Active_Energy(void);
void CalcPrint_Tot_Funda_Energy(void);
void CalcPrint_Tot_React_Energy(void);
void CalcPrint_Tot_App_Energy(void);

void RT_CalcPrint_V1_RMS( void );
void RT_CalcPrint_C1_RMS( void );
void RT_CalcPrint_Tot_Active_Energy(void);
void RT_CalcPrint_Active_Pwr(void);



bool STPM32_Init(void);
bool SendMessage(uint32_t ReadAddress, uint8_t* ReceiveMessage ,uint32_t SendAddress, uint8_t* SendMessage);
bool ReadMsgOnly (uint32_t ReadAddress, uint8_t* ReadMessage);
bool SendMsgOnly (uint32_t SendAddress, uint8_t* SendMessage);
bool Read_Reg(uint32_t ReadAddress, uint8_t* ReturnBuffer);
