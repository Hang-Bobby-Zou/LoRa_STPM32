/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "usart.h"
#include "spi.h"
#include "ext_flash.h"
#include "stdio.h"
#include "ext_flash_tb.h"
#include "STPM32.h"
#include "STPM32_AddressMap.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R1					810000
#define R2 					470

#define V_ref 			1.18
#define k_int 			0.8155773

#define A_v 				2
#define A_i 				2
#define cal_v 			0.875
#define cal_i				0.875

#define k_s					0.0024
#define k_int				1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint8_t ReadBuffer[5] = {0};
uint8_t RxBuffer[5] = {0};
uint8_t i[1] = {0x2E};
double freq;
double phase;
int count = 0;







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

UBaseType_t USART1_Priority;
UBaseType_t USART3_Priority;
UBaseType_t SPI1_Priority;
UBaseType_t SPI2_Priority;

//uint32_t 	flash_pointer = 0;
//uint32_t 	flash_sector_pointer = 0;
//uint8_t 	flash_buffer[4096] = {0};
//uint32_t 	flash_count = 0;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId IDLEHandle;
osThreadId USART1Handle;
osThreadId USART3Handle;
osThreadId SPI2Handle;
osSemaphoreId myBinarySem01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void uint8_cpy(uint8_t* dest, uint8_t* src, uint8_t size);
void CalcPrint_Freq(void);
void CalcPrint_RMS(void);
void CalcPrint_Phase(void);
void CalcPrint_Active_Energy(void);
void CalcPrint_Funda_Energy(void);
void CalcPrint_React_Energy(void);
void CalcPrint_App_Energy(void);
void CalcPrint_Active_Pwr(void);
void CalcPrint_Funda_Pwr(void);
void CalcPrint_React_Pwr(void);
void CalcPrint_App_RMS_Pwr(void);
void CalcPrint_Tot_Active_Pwr(void);
void CalcPrint_Tot_Funda_Pwr(void);
void CalcPrint_Tot_React_Pwr(void);
void CalcPrint_Tot_App_Pwr(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void defaultIDLE(void const * argument);
void StartUSART1(void const * argument);
void StartUSART3(void const * argument);
void StartSPI2(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of IDLE */
  osThreadDef(IDLE, defaultIDLE, osPriorityIdle, 0, 128);
  IDLEHandle = osThreadCreate(osThread(IDLE), NULL);

  /* definition and creation of USART1 */
  osThreadDef(USART1, StartUSART1, osPriorityNormal, 0, 128);
  USART1Handle = osThreadCreate(osThread(USART1), NULL);

  /* definition and creation of USART3 */
  osThreadDef(USART3, StartUSART3, osPriorityBelowNormal, 0, 128);
  USART3Handle = osThreadCreate(osThread(USART3), NULL);

  /* definition and creation of SPI2 */
  osThreadDef(SPI2, StartSPI2, osPriorityHigh, 0, 128);
  SPI2Handle = osThreadCreate(osThread(SPI2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_defaultIDLE */
/**
* @brief Function implementing the IDLE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_defaultIDLE */
void defaultIDLE(void const * argument)
{
  /* USER CODE BEGIN defaultIDLE */
  /* Infinite loop */
  for(;;)
  {
    //Do nothing
		osDelay(1);
  }
  /* USER CODE END defaultIDLE */
}

/* USER CODE BEGIN Header_StartUSART1 */
/**
* @brief Function implementing the USART1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUSART1 */
void StartUSART1(void const * argument)
{
  /* USER CODE BEGIN StartUSART1 */
  USART1_Priority = uxTaskPriorityGet( NULL );
	/* Infinite loop */
  for(;;)
  {		
	
		if (i[0] > 0x8A){
			i[0] = 0x2E;
		}
		
		//i[0] = 0x54;
		

		if (USART1_RxFlag == 1){
		 	RxBuffer[0] = ReadBuffer[0];
		 	RxBuffer[1] = ReadBuffer[1];
		 	RxBuffer[2] = ReadBuffer[2];
		 	RxBuffer[3] = ReadBuffer[3];
			RxBuffer[4] = ReadBuffer[4];
			
		 	//myprintf("\r\n");
		 	//USART3_PINSET_TX();
		 	//myprintf("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], RxBuffer[0], RxBuffer[1], RxBuffer[2], RxBuffer[3], RxBuffer[4]);
		 	//USART3_PINSET_RX();
			
			
			
			if (count == 10){
			 	// USART3_PINSET_TX();
				if (i[0] == 0x2E){
					myprintf("Copying PH_Period\r\n");
					uint8_cpy(PH_Period, RxBuffer, 5);
				} else if (i[0] == 0x4E){
					myprintf("Copying C1_PHA\r\n");
					uint8_cpy(C1_PHA, RxBuffer, 5);
				} else if (i[0] == dsp_reg14){
					myprintf("Copying CH1_RMS\r\n");
					uint8_cpy(CH1_RMS,RxBuffer,5);
				} else if (i[0] == ph1_reg1){
					myprintf("Copying PH1_Active_Energy\r\n");
					uint8_cpy(PH1_Active_Energy, RxBuffer, 5);
				} else if (i[0] == ph1_reg2){
					myprintf("Copying PH1_Fundamental_Energy\r\n");
					uint8_cpy(PH1_Fundamental_Energy, RxBuffer, 5);
				} else if (i[0] == ph1_reg3){
					myprintf("Copying PH1_Reactive_Energy\r\n");
					uint8_cpy(PH1_Reactive_Energy, RxBuffer, 5);
				} else if (i[0] == ph1_reg4){
					myprintf("Copying PH1_Apparent_Energy\r\n");
					uint8_cpy(PH1_Apparent_Energy, RxBuffer,5);
				} else if (i[0] == ph1_reg5){
					myprintf("Copying PH1_Active_Power\r\n");
					uint8_cpy(PH1_Active_Power, RxBuffer, 5);
				} else if (i[0] == ph1_reg6){
					myprintf("Copying PH1_Fundamental_Power\r\n");
					uint8_cpy(PH1_Fundamental_Power, RxBuffer, 5);
				} else if (i[0] == ph1_reg7){
					myprintf("Copying PH1_Reactive_Power\r\n");
					uint8_cpy(PH1_Reactive_Power, RxBuffer, 5);
				} else if (i[0] == ph1_reg8){
					myprintf("Copying PH1_Apparent_RMS_Power\r\n");
					uint8_cpy(PH1_Apparent_RMS_Power, RxBuffer, 5);
				} else if (i[0] == tot_reg1){
					myprintf("Copying Total_Active_Energy\r\n");
					uint8_cpy(Total_Active_Energy, RxBuffer, 5);
				} else if (i[0] == tot_reg2){
					myprintf("Copying Total_Fundamental_Energy\r\n");
					uint8_cpy(Total_Fundamental_Energy, RxBuffer, 5);
				} else if (i[0] == tot_reg3){
					myprintf("Copying Total_Reactive_Energy\r\n");
					uint8_cpy(Total_Reactive_Energy, RxBuffer, 5);
				} else if (i[0] == tot_reg4){
					myprintf("Copying Total_Apparent_Energy\r\n");
					uint8_cpy(Total_Apparent_Energy, RxBuffer, 5);
				}
				// USART3_PINSET_RX();
				
				
				if (i[0] == 0x2E){
					CalcPrint_Freq();
				} else if (i[0] == 0x48){
					CalcPrint_RMS();
				} else if (i[0] == 0x4E){
					CalcPrint_Phase();
				} else if (i[0] == 0x54){
					CalcPrint_Active_Energy();
				} else if (i[0] == 0x56){
					CalcPrint_Funda_Energy();
				} else if (i[0] == 0x58){
					CalcPrint_React_Energy();
				} else if (i[0] == 0x5A){
					CalcPrint_App_Energy();
				} else if (i[0] == 0x5C){
					CalcPrint_Active_Pwr();
				} else if (i[0] == 0x5E){
					CalcPrint_Funda_Pwr();
				} else if (i[0] == 0x60){
					CalcPrint_React_Pwr();
				} else if (i[0] == 0x62){
					CalcPrint_App_RMS_Pwr();
				} else if (i[0] == 0x84){
					CalcPrint_Tot_Active_Pwr();
				} else if (i[0] == 0x86){
					CalcPrint_Tot_Funda_Pwr();
				} else if (i[0] == 0x88){
					CalcPrint_Tot_React_Pwr();
				} else if (i[0] == 0x8A){
					CalcPrint_Tot_App_Pwr();
				}
				
				i[0] += 0x02;
			 	count = 0;
			}
			count++;
			
			//i[0] += 0x02;

		 	USART1_RxFlag = 0;
			
		 	//vTaskDelay(pdMS_TO_TICKS( 1 ));				//Block delay
		}
		
		ReadMsgOnly(i[0],ReadBuffer);
		
		//xTicksToDelay(pdMS_TO_TICKS( 1000 ));	//Runing delay
		osDelay(1);
  }
  /* USER CODE END StartUSART1 */
}

/* USER CODE BEGIN Header_StartUSART3 */
/**
* @brief Function implementing the USART3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUSART3 */
void StartUSART3(void const * argument)
{
  /* USER CODE BEGIN StartUSART3 */
  USART3_Priority = uxTaskPriorityGet( NULL );
	uint8_t aRxBuffer[8];
	/* Infinite loop */
  for(;;)
  {		
		 HAL_UART_Receive_IT(&huart3, aRxBuffer, 1);

		 if (USART3_RxFlag == 1){
		 	//vTaskSuspend(USART1Handle);
			
			USART3_PINSET_TX();
			if (aRxBuffer[0] == dsp_reg14){
				myprintf("Reading: CH1_RMS\r\n");
				myprintf("%x | %x | %x | %x | %x \r\n", CH1_RMS[0], CH1_RMS[1], CH1_RMS[2], CH1_RMS[3], CH1_RMS[4]);
			} else if (aRxBuffer[0] == ph1_reg1){
				myprintf("Reading: PH1_Active_Energy\r\n");
				myprintf("%x | %x | %x | %x | %x \r\n", PH1_Active_Energy[0],PH1_Active_Energy[1],PH1_Active_Energy[2],PH1_Active_Energy[3],PH1_Active_Energy[4]);
			} else if (aRxBuffer[0] == ph1_reg2){
				myprintf("Reading: PH1_Fundamental_Energy\r\n");
				myprintf("%x | %x | %x | %x | %x \r\n", PH1_Fundamental_Energy[0], PH1_Fundamental_Energy[1], PH1_Fundamental_Energy[2], PH1_Fundamental_Energy[3], PH1_Fundamental_Energy[4]);
			} else if (aRxBuffer[0] == ph1_reg3){
				myprintf("Reading: PH1_Reactive_Energy\r\n");
				myprintf("%x | %x | %x | %x | %x \r\n", PH1_Reactive_Energy[0], PH1_Reactive_Energy[1], PH1_Reactive_Energy[2], PH1_Reactive_Energy[3], PH1_Reactive_Energy[4]);
			} else if (aRxBuffer[0] == ph1_reg4){
				myprintf("Reading: PH1_Apparent_Energy\r\n");
				myprintf("%x | %x | %x | %x | %x \r\n", PH1_Apparent_Energy[0], PH1_Apparent_Energy[1], PH1_Apparent_Energy[2], PH1_Apparent_Energy[3], PH1_Apparent_Energy[4]);
			} else if (aRxBuffer[0] == ph1_reg5){
				myprintf("Reading: PH1_Active_Power\r\n");
				myprintf("%x | %x | %x | %x | %x \r\n", PH1_Active_Power[0], PH1_Active_Power[1], PH1_Active_Power[2], PH1_Active_Power[3], PH1_Active_Power[4]);
			} else if (aRxBuffer[0] == ph1_reg6){
				myprintf("Reading: PH1_Fundamental_Power\r\n");
				myprintf("%x | %x | %x | %x | %x \r\n", PH1_Fundamental_Power[0], PH1_Fundamental_Power[1], PH1_Fundamental_Power[2], PH1_Fundamental_Power[3], PH1_Fundamental_Power[4]);
			} else if (aRxBuffer[0] == ph1_reg7){
				myprintf("Reading: PH1_Reactive_Power\r\n");
				myprintf("%x | %x | %x | %x | %x \r\n", PH1_Reactive_Power[0], PH1_Reactive_Power[1], PH1_Reactive_Power[2], PH1_Reactive_Power[3], PH1_Reactive_Power[4]);
			} else if (aRxBuffer[0] == ph1_reg8){
				myprintf("Reading: PH1_Apparent_RMS_Power\r\n");
				myprintf("%x | %x | %x | %x | %x \r\n", PH1_Apparent_RMS_Power[0], PH1_Apparent_RMS_Power[1], PH1_Apparent_RMS_Power[2], PH1_Apparent_RMS_Power[3], PH1_Apparent_RMS_Power[4]);
			} else if (aRxBuffer[0] == tot_reg1){
				myprintf("Reading: Total_Active_Energy\r\n");
				myprintf("%x | %x | %x | %x | %x \r\n", Total_Active_Energy[0], Total_Active_Energy[1], Total_Active_Energy[2], Total_Active_Energy[3], Total_Active_Energy[4]);
			} else if (aRxBuffer[0] == tot_reg2){
				myprintf("Reading: Total_Fundamental_Energy\r\n");
				myprintf("%x | %x | %x | %x | %x \r\n", Total_Fundamental_Energy[0], Total_Fundamental_Energy[1], Total_Fundamental_Energy[2], Total_Fundamental_Energy[3], Total_Fundamental_Energy[4]);
			} else if (aRxBuffer[0] == tot_reg3){
				myprintf("Reading: Total_Reactive_Energy\r\n");
				myprintf("%x | %x | %x | %x | %x \r\n", Total_Reactive_Energy[0], Total_Reactive_Energy[1], Total_Reactive_Energy[2], Total_Reactive_Energy[3], Total_Reactive_Energy[4]);
			} else if (aRxBuffer[0] == tot_reg4){
				myprintf("Reading: Total_Apparent_Energy\r\n");
				myprintf("%x | %x | %x | %x | %x \r\n", Total_Apparent_Energy[0], Total_Apparent_Energy[1], Total_Apparent_Energy[2], Total_Apparent_Energy[3], Total_Apparent_Energy[4]);
			} else {
				myprintf(" Not a valid address \r\n");
			}
			USART3_PINSET_RX();
			
			
		 	//USART3_PINSET_TX();
		 	//HAL_UART_Transmit(&huart3, aRxBuffer, 8, 0xFFFF);
		 	//USART3_PINSET_RX();
			
			USART3_RxFlag = 0;
		 	//vTaskResume(USART1Handle);
		 }

		osDelay(1); //This delay is in ms
  }
  /* USER CODE END StartUSART3 */
}

/* USER CODE BEGIN Header_StartSPI2 */
/**
* @brief Function implementing the SPI2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSPI2 */
void StartSPI2(void const * argument)
{
  /* USER CODE BEGIN StartSPI2 */
  SPI2_Priority = uxTaskPriorityGet( NULL );
	/* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSPI2 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */






void CalcPrint_Freq(void){
	uint16_t CalcBuffer1 = PH_Period[1];
	uint16_t CalcBuffer2 = PH_Period[0];
	CalcBuffer1 = CalcBuffer1 << 8;

	CalcBuffer1 = CalcBuffer1 + CalcBuffer2;

	freq = 1.0 / (CalcBuffer1 * 0.000008);
	
	USART3_PINSET_TX();
	myprintf("Freq: %4f Hz\r\n\r\n", freq);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
}

void CalcPrint_RMS(void){
	uint16_t CalcBuffer1 = CH1_RMS[1];
	uint16_t CalcBuffer2 = CH1_RMS[0];
	
	CalcBuffer1 = CalcBuffer1 << 8;
	CalcBuffer1 = CalcBuffer1 + CalcBuffer2;
	
	uint16_t V1_RMS = CalcBuffer1 & 0x7FFF;
	
	uint16_t CalcBuffer3 = CH1_RMS[1];
	uint16_t C1_RMS = CalcBuffer3 >> 7;
	
	uint16_t CalcBuffer4 = CH1_RMS[2];
	uint16_t CalcBuffer5 = CH1_RMS[3];
	
	C1_RMS += CalcBuffer4 << 1;
	C1_RMS += CalcBuffer5 << 9;
	
	uint16_t C1_RMS_Out = C1_RMS * V_ref / (cal_i * A_i * 131072 * k_s * k_int);
	uint16_t V1_RMS_Out = V1_RMS * V_ref * (1 + R1/R2) / (cal_v * A_v * 32768); 
	
	USART3_PINSET_TX();
	myprintf("C1= %d Amps | V1= %d Volts\r\n\r\n",C1_RMS_Out, V1_RMS_Out);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
}

void CalcPrint_Phase(void){
	phase = C1_PHA[3] / 125000 * 50 * 360;
	
	USART3_PINSET_TX();
	myprintf("Phase = %4f degrees\r\n\r\n", phase);
	USART3_PINSET_RX();
	
		HAL_Delay(1);
}


void CalcPrint_Active_Energy(void){
	uint32_t CalcBuffer1 = PH1_Active_Energy[3];
	uint32_t CalcBuffer2 = PH1_Active_Energy[2];
	uint32_t CalcBuffer3 = PH1_Active_Energy[1];
	uint32_t CalcBuffer4 = PH1_Active_Energy[0];
	
	uint32_t CalcBuffer = CalcBuffer1 << 24;
	
	CalcBuffer += (CalcBuffer2 << 16);
	CalcBuffer += (CalcBuffer3 << 8);
	CalcBuffer += (CalcBuffer4);
	
	USART3_PINSET_TX();
	myprintf("Active Energy = %lu Joules\r\n\r\n", CalcBuffer);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
}

void CalcPrint_Funda_Energy(void){
	uint32_t CalcBuffer1 = PH1_Fundamental_Energy[3];
	uint32_t CalcBuffer2 = PH1_Fundamental_Energy[2];
	uint32_t CalcBuffer3 = PH1_Fundamental_Energy[1];
	uint32_t CalcBuffer4 = PH1_Fundamental_Energy[0];
	
	uint32_t CalcBuffer = CalcBuffer1 << 24;
	
	CalcBuffer += (CalcBuffer2 << 16);
	CalcBuffer += (CalcBuffer3 << 8);
	CalcBuffer += (CalcBuffer4);
	
	USART3_PINSET_TX();
	myprintf("Fundamental Energy = %lu Joules\r\n\r\n", CalcBuffer);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
	
}

void CalcPrint_React_Energy(void){
	uint32_t CalcBuffer1 = PH1_Reactive_Energy[3];
	uint32_t CalcBuffer2 = PH1_Reactive_Energy[2];
	uint32_t CalcBuffer3 = PH1_Reactive_Energy[1];
	uint32_t CalcBuffer4 = PH1_Reactive_Energy[0];
	
	uint32_t CalcBuffer = CalcBuffer1 << 24;
	
	CalcBuffer += (CalcBuffer2 << 16);
	CalcBuffer += (CalcBuffer3 << 8);
	CalcBuffer += (CalcBuffer4);
	
	USART3_PINSET_TX();
	myprintf("Reactive Energy = %lu Joules\r\n\r\n", CalcBuffer);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
}

void CalcPrint_App_Energy(void){
	uint32_t CalcBuffer1 = PH1_Apparent_Energy[3];
	uint32_t CalcBuffer2 = PH1_Apparent_Energy[2];
	uint32_t CalcBuffer3 = PH1_Apparent_Energy[1];
	uint32_t CalcBuffer4 = PH1_Apparent_Energy[0];
	
	uint32_t CalcBuffer = CalcBuffer1 << 24;
	
	CalcBuffer += (CalcBuffer2 << 16);
	CalcBuffer += (CalcBuffer3 << 8);
	CalcBuffer += (CalcBuffer4);
	
	USART3_PINSET_TX();
	myprintf("Apparent Energy = %lu Joules\r\n\r\n", CalcBuffer);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
}

void CalcPrint_Active_Pwr(void){
	uint32_t CalcBuffer1 = (PH1_Active_Power[3] & 0x1F);
	uint32_t CalcBuffer2 = PH1_Active_Power[2];
	uint32_t CalcBuffer3 = PH1_Active_Power[1];
	uint32_t CalcBuffer4 = PH1_Active_Power[0];
	
	uint32_t CalcBuffer = CalcBuffer1 << 24;
	
	CalcBuffer += (CalcBuffer2 << 16);
	CalcBuffer += (CalcBuffer3 << 8);
	CalcBuffer += (CalcBuffer4);
	
	USART3_PINSET_TX();
	myprintf("Active Power = %lu Watts\r\n\r\n", CalcBuffer);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
}

void CalcPrint_Funda_Pwr(void){
	uint32_t CalcBuffer1 = (PH1_Fundamental_Power[3] & 0x1F);
	uint32_t CalcBuffer2 = PH1_Fundamental_Power[2];
	uint32_t CalcBuffer3 = PH1_Fundamental_Power[1];
	uint32_t CalcBuffer4 = PH1_Fundamental_Power[0];
	
	uint32_t CalcBuffer = CalcBuffer1 << 24;
	
	CalcBuffer += (CalcBuffer2 << 16);
	CalcBuffer += (CalcBuffer3 << 8);
	CalcBuffer += (CalcBuffer4);
	
	USART3_PINSET_TX();
	myprintf("Fundamental Power = %lu Watts\r\n\r\n", CalcBuffer);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
}

void CalcPrint_React_Pwr(void){
	uint32_t CalcBuffer1 = (PH1_Reactive_Power[3] & 0x1F);
	uint32_t CalcBuffer2 = PH1_Reactive_Power[2];
	uint32_t CalcBuffer3 = PH1_Reactive_Power[1];
	uint32_t CalcBuffer4 = PH1_Reactive_Power[0];
	
	uint32_t CalcBuffer = CalcBuffer1 << 24;
	
	CalcBuffer += (CalcBuffer2 << 16);
	CalcBuffer += (CalcBuffer3 << 8);
	CalcBuffer += (CalcBuffer4);
	
	USART3_PINSET_TX();
	myprintf("Reactive Power = %lu Watts\r\n\r\n", CalcBuffer);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
}

void CalcPrint_App_RMS_Pwr(void){
	uint32_t CalcBuffer1 = (PH1_Apparent_RMS_Power[3] & 0x1F);
	uint32_t CalcBuffer2 = PH1_Apparent_RMS_Power[2];
	uint32_t CalcBuffer3 = PH1_Apparent_RMS_Power[1];
	uint32_t CalcBuffer4 = PH1_Apparent_RMS_Power[0];
	
	uint32_t CalcBuffer = CalcBuffer1 << 24;
	
	CalcBuffer += (CalcBuffer2 << 16);
	CalcBuffer += (CalcBuffer3 << 8);
	CalcBuffer += (CalcBuffer4);
	
	USART3_PINSET_TX();
	myprintf("Apparent_RMS Power = %lu Watts\r\n\r\n", CalcBuffer);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
}

void CalcPrint_Tot_Active_Pwr(void){
	uint32_t CalcBuffer1 = Total_Active_Energy[3];
	uint32_t CalcBuffer2 = Total_Active_Energy[2];
	uint32_t CalcBuffer3 = Total_Active_Energy[1];
	uint32_t CalcBuffer4 = Total_Active_Energy[0];
	
	uint32_t CalcBuffer = CalcBuffer1 << 24;
	
	CalcBuffer += (CalcBuffer2 << 16);
	CalcBuffer += (CalcBuffer3 << 8);
	CalcBuffer += (CalcBuffer4);
	
	USART3_PINSET_TX();
	myprintf("Total Active Energy = %lu Joules\r\n\r\n", CalcBuffer);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
}

void CalcPrint_Tot_Funda_Pwr(void){
	uint32_t CalcBuffer1 = Total_Fundamental_Energy[3];
	uint32_t CalcBuffer2 = Total_Fundamental_Energy[2];
	uint32_t CalcBuffer3 = Total_Fundamental_Energy[1];
	uint32_t CalcBuffer4 = Total_Fundamental_Energy[0];
	
	uint32_t CalcBuffer = CalcBuffer1 << 24;
	
	CalcBuffer += (CalcBuffer2 << 16);
	CalcBuffer += (CalcBuffer3 << 8);
	CalcBuffer += (CalcBuffer4);
	
	USART3_PINSET_TX();
	myprintf("Total Fundamental Energy = %lu Joules\r\n\r\n", CalcBuffer);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
}

void CalcPrint_Tot_React_Pwr(void){
	uint32_t CalcBuffer1 = Total_Reactive_Energy[3];
	uint32_t CalcBuffer2 = Total_Reactive_Energy[2];
	uint32_t CalcBuffer3 = Total_Reactive_Energy[1];
	uint32_t CalcBuffer4 = Total_Reactive_Energy[0];
	
	uint32_t CalcBuffer = CalcBuffer1 << 24;
	
	CalcBuffer += (CalcBuffer2 << 16);
	CalcBuffer += (CalcBuffer3 << 8);
	CalcBuffer += (CalcBuffer4);
	
	USART3_PINSET_TX();
	myprintf("Total Reactive Energy = %lu Joules\r\n\r\n", CalcBuffer);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
}

void CalcPrint_Tot_App_Pwr(void){
	uint32_t CalcBuffer1 = Total_Apparent_Energy[3];
	uint32_t CalcBuffer2 = Total_Apparent_Energy[2];
	uint32_t CalcBuffer3 = Total_Apparent_Energy[1];
	uint32_t CalcBuffer4 = Total_Apparent_Energy[0];
	
	uint32_t CalcBuffer = CalcBuffer1 << 24;
	
	CalcBuffer += (CalcBuffer2 << 16);
	CalcBuffer += (CalcBuffer3 << 8);
	CalcBuffer += (CalcBuffer4);
	
	USART3_PINSET_TX();
	myprintf("Total Apparent Energy = %lu Joules\r\n\r\n", CalcBuffer);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
}


void uint8_cpy(uint8_t dest[], uint8_t src[], uint8_t size){
	for (int i = 0; i < size; i++){
		dest[i] = src[i];
	}
}





/**
  * @brief Rx Transfer completed callbacks
  * @param huart: uart handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	
	if (huart == &huart1){
		USART1_RxFlag = 1;
	}
	
	if (huart == &huart3){
		USART3_RxFlag = 1;
	}
}

/**
  * @brief Tx Transfer completed callbacks
  * @param huart: uart handle
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	
	if (huart == &huart1){
		USART1_TxFlag = 1;
	}
	
	if (huart == &huart3){
		USART3_TxFlag = 1;
	}
	
}

/**
  * @brief Tx Transfer completed callbacks
  * @param hspi: spi handle
  * @retval None
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	/* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
	
	if (hspi == &hspi1){
		SPI1_TxFlag = 1;
	}
	
	if (hspi == &hspi2){
		SPI2_TxFlag = 1;
	}
}

/**
  * @brief Tx Rransfer completed callbacks
  * @param hspi: spi handle
  * @retval None
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
	/* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
	
	if (hspi == &hspi1){
		SPI1_RxFlag = 1;
	}
	
	if (hspi == &hspi2){
		SPI2_RxFlag = 1;
	}
}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
