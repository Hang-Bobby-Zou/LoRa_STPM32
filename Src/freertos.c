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
#include "HAL_spi.h"
#include "ext_flash.h"
#include "stdio.h"
#include "ext_flash_tb.h"
#include "STPM32.h"
#include "STPM32_AddressMap.h"
#include "LoRa.h"
#include "HAL_LoRaMAC.h"
#include "Commissioning.h"
#include "LoRaMac.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R1									810000
#define R2 									470

#define F_CLK								125000
#define P_CLK								0.000008		//  1 / F_CLK
#define D_CLK								8712.5 


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


#define LORAMAC_SEND_RETRY_COUNT_MAX 48


#define DelayMsPoll(x) { for (uint32_t j = 0; j < x; j++) {for (uint32_t i = 0; i < 8000; i++) {  }}}	
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t ReadBuffer[5] = {0};
uint8_t HAL_RxBuffer[5] = {0};
uint8_t i[1] = {0x2E};
int count = 0;
uint16_t FlashPointer = 0x00;

uint8_t aRxBuffer[8] = {0};

static int USART3_RxFlag = 0;
static int USART3_TxFlag = 0;

uint8_t loramac_send_retry_count = 0;

int LoRa_Block_Time = 10000;

//Raw data from STPM32 defines
static uint8_t PH_Period								[5] = {0};
static uint8_t CH1_RMS									[5] = {0};
static uint8_t C1_PHA									[5] = {0};

static uint8_t PH1_Active_Energy				[5] = {0};
static uint8_t PH1_Fundamental_Energy	[5] = {0};
static uint8_t PH1_Reactive_Energy			[5] = {0};
static uint8_t PH1_Apparent_Energy			[5] = {0};
		
static uint8_t PH1_Active_Power				[5] = {0};
static uint8_t PH1_Fundamental_Power		[5] = {0};
static uint8_t PH1_Reactive_Power			[5] = {0};
static uint8_t	PH1_Apparent_RMS_Power	[5] = {0};

static uint8_t Total_Active_Energy			[5] = {0};
static uint8_t Total_Fundamental_Energy[5] = {0};
static uint8_t Total_Reactive_Energy		[5] = {0};
static uint8_t Total_Apparent_Energy		[5] = {0};

UBaseType_t USART1_Priority;
UBaseType_t USART3_Priority;
UBaseType_t SPI1_Priority;
UBaseType_t SPI2_Priority;


extern LoRaMacFlags_t LoRaMacFlags;


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
  osThreadDef(USART3, StartUSART3, osPriorityAboveNormal, 0, 256);
  USART3Handle = osThreadCreate(osThread(USART3), NULL);

  /* definition and creation of SPI2 */
  osThreadDef(SPI2, StartSPI2, osPriorityHigh, 0, 256);
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
		//DEBUG("IDLE\r\n");
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
/*	  
		//To cycle the register address pointer
		if (i[0] > 0x8A){
			i[0] = 0x2E;
			
			FlashPointer += 0x08;
			myprintf("Blocking USART1 for 3 seconds\r\n\r\n");
			
			vTaskDelay (pdMS_TO_TICKS( 3000 ));	//If walks around for 1 term, then block itself for 1 sec for users to read something
		}
		
		if (FlashPointer > 0x00FFFF){
			FlashPointer = 0x000000;
			
			USART3_PINSET_TX();
			myprintf("!!!Flash Full!!!\r\n");
			myprintf("...Overwriting Previous Info...\r\n");
			USART3_PINSET_RX();
		}

		if (USART1_RxFlag == 1){
		 	HAL_RxBuffer[0] = ReadBuffer[0];
		 	HAL_RxBuffer[1] = ReadBuffer[1];
		 	HAL_RxBuffer[2] = ReadBuffer[2];
		 	HAL_RxBuffer[3] = ReadBuffer[3];
			HAL_RxBuffer[4] = ReadBuffer[4];

			if (count == 10){		//Wait for the third iteration so the data is stable
			 	// USART3_PINSET_TX();
					if (i[0] == dsp_reg1){
						myprintf("Copying PH_Period\r\n");
						uint8_cpy(PH_Period, HAL_RxBuffer, 5);
						CalcPrint_Freq();
					
					} else if (i[0] == dsp_reg17){
						myprintf("Copying C1_PHA\r\n");
						uint8_cpy(C1_PHA, HAL_RxBuffer, 5);
						CalcPrint_Phase();
						
					} else if (i[0] == dsp_reg14){
						myprintf("Copying CH1_RMS\r\n");
						uint8_cpy(CH1_RMS,HAL_RxBuffer,5);
						CalcPrint_V1_RMS();
						CalcPrint_C1_RMS();
						
					} else if (i[0] == ph1_reg1){
						myprintf("Copying PH1_Active_Energy\r\n");
						uint8_cpy(PH1_Active_Energy, HAL_RxBuffer, 5);
						CalcPrint_Active_Energy();
						
					} else if (i[0] == ph1_reg2){
						myprintf("Copying PH1_Fundamental_Energy\r\n");
						uint8_cpy(PH1_Fundamental_Energy, HAL_RxBuffer, 5);
						//CalcPrint_Funda_Energy();
						
					} else if (i[0] == ph1_reg3){
						myprintf("Copying PH1_Reactive_Energy\r\n");
						uint8_cpy(PH1_Reactive_Energy, HAL_RxBuffer, 5);
						CalcPrint_React_Energy();
						
					} else if (i[0] == ph1_reg4){
						myprintf("Copying PH1_Apparent_Energy\r\n");
						uint8_cpy(PH1_Apparent_Energy, HAL_RxBuffer,5);
						CalcPrint_App_Energy();
						
					} else if (i[0] == ph1_reg5){
						myprintf("Copying PH1_Active_Power\r\n");
						uint8_cpy(PH1_Active_Power, HAL_RxBuffer, 5);
						CalcPrint_Active_Pwr();
						
					} else if (i[0] == ph1_reg6){
						myprintf("Copying PH1_Fundamental_Power\r\n");
						uint8_cpy(PH1_Fundamental_Power, HAL_RxBuffer, 5);
						//CalcPrint_Funda_Pwr();
						
					} else if (i[0] == ph1_reg7){
						myprintf("Copying PH1_Reactive_Power\r\n");
						uint8_cpy(PH1_Reactive_Power, HAL_RxBuffer, 5);
						CalcPrint_React_Pwr();
						
					} else if (i[0] == ph1_reg8){
						myprintf("Copying PH1_Apparent_RMS_Power\r\n");
						uint8_cpy(PH1_Apparent_RMS_Power, HAL_RxBuffer, 5);
						CalcPrint_App_RMS_Pwr();
						
					} else if (i[0] == tot_reg1){
						myprintf("Copying Total_Active_Energy\r\n");
						uint8_cpy(Total_Active_Energy, HAL_RxBuffer, 5);
						//CalcPrint_Tot_Active_Energy();
						
					} else if (i[0] == tot_reg2){
						myprintf("Copying Total_Fundamental_Energy\r\n");
						uint8_cpy(Total_Fundamental_Energy, HAL_RxBuffer, 5);
						//CalcPrint_Tot_Funda_Energy();
						
					} else if (i[0] == tot_reg3){
						myprintf("Copying Total_Reactive_Energy\r\n");
						uint8_cpy(Total_Reactive_Energy, HAL_RxBuffer, 5);
						//CalcPrint_Tot_React_Energy();
						
					} else if (i[0] == tot_reg4){
						myprintf("Copying Total_Apparent_Energy\r\n");
						uint8_cpy(Total_Apparent_Energy, HAL_RxBuffer, 5);
						CalcPrint_Tot_App_Energy();
						
					}
				// USART3_PINSET_RX();
				
				i[0] += 0x02;
				
				count = 0;
			}

			count++;

		 	USART1_RxFlag = 0;
		}
		
		ReadMsgOnly(i[0],ReadBuffer);
		
		//xTicksToDelay(pdMS_TO_TICKS( 1000 ));		//Runing delay
*/
		
		//vTaskDelay (pdMS_TO_TICKS( 1000 ));

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
	//HAL_UART_Receive_IT(&huart3, aRxBuffer, 4); 
	//myprintf("USART3 Running\r\n");
	/* Infinite loop */
  for(;;)
  {		
		//HAL_NVIC_EnableIRQ(TIM7_IRQn);
		/*
		if (USART3_RxFlag == 1){

			char data[8] = {0};

			uint32_t addr = 0;
			
			addr = addr | aRxBuffer[0] << 24;
			addr = addr | aRxBuffer[1] << 16;
			addr = addr | aRxBuffer[2] << 8;
			addr = addr | aRxBuffer[3];
			
			ext_flash_read(addr, data, 8);
			
			
			USART3_PINSET_TX();
			myprintf("Reading addr: %x ,Data: %x %x %x %x %x %x %x %x \r\n\r\n",addr,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
			USART3_PINSET_RX();
			
			
			USART3_RxFlag = 0;
		}
*/

/*
		if (USART3_RxFlag == 1){
			//vTaskSuspend(SPI2Handle);
			
			USART3_PINSET_TX();
			HAL_UART_Transmit(&huart3, aRxBuffer, 4, 0xFFFF);
			USART3_PINSET_RX();
			
			//LoRa_ForceSetIDLE();
			
			USART3_RxFlag = 0;
			HAL_UART_Receive_IT(&huart3, aRxBuffer, 4); 
			
			//vTaskResume(SPI2Handle);
		}
*/
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

	myprintf("LoRaMAC Init...\r\n");

	LoRaMAC_Init();
	
	myprintf("LoRaMAC Init Done. \r\n");
	
	myprintf("LoRaMAC Join...\r\n");

	LoRaMAC_Join();
	
	myprintf("LoRaMAC Join Done. \r\n");

	/* Infinite loop */
  for(;;)
  {
			if (LoRa_CheckStateIDLE() == true){
				if(LoRaMAC_Send() == -1){ //If send was not successful
					if (loramac_send_retry_count < LORAMAC_SEND_RETRY_COUNT_MAX){
						loramac_send_retry_count ++;
						myprintf("LoRaMAC Send Failed, retrying for %d time...\r\n", loramac_send_retry_count);
						TimerIrqHandler();
					}
					
				} else {	
					myprintf("\r\nLoRaMAC Send Succeed! Blocking for %d miliseconds...\r\n", LoRa_Block_Time);
					loramac_send_retry_count = 0;
					
					//TimerIrqHandler();					
					
				}
			}
			osDelay(5000);
			//HAL_Delay(5000);
			//vTaskDelay(pdMS_TO_TICKS( LoRa_Block_Time));
	}
		//osDelay(1);	
  /* USER CODE END StartSPI2 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief Rx Transfer completed callbacks
  * @param huart: uart handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(huart);
	
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
  //UNUSED(huart);
	
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
	
	ext_flash_write(FlashPointer + FlashAddr_Freq, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint16_t freq_raw;

	freq_raw = freq_raw | (uint16_t) PH_Period[1] << 8;
	freq_raw = freq_raw | (uint16_t) PH_Period[2];
	double freq = 1.0 / (freq_raw * P_CLK);
	
	if( freq < Freq_Low_Threshold || freq > Freq_High_Threshold){
		USART3_PINSET_TX();
		myprintf("ERROR: Freq error: %f Hz\r\n\r\n", freq);		//4 decimal numbers
		USART3_PINSET_RX();
	} else {
		USART3_PINSET_TX();
		myprintf("Freq: %f Hz\r\n\r\n", freq);		//4 decimal numbers
		USART3_PINSET_RX();
	}
	
	HAL_Delay(1);
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
	
	ext_flash_write(FlashPointer + FlashAddr_RMS, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	
	static double V1_RMS = 0;
	
	uint16_t V1_RMS_raw = 0x0000;
	V1_RMS_raw = V1_RMS_raw | ((uint16_t) CH1_RMS[1] << 8);
	V1_RMS_raw = V1_RMS_raw | (uint16_t) CH1_RMS[0];
	V1_RMS_raw = V1_RMS_raw & 0x7FFF; 				//Mask the most significant bit.
	
	V1_RMS = (double) V1_RMS_raw * (double) V_ref * (1.0 + (double) R1/ (double) R2) / ( (double) cal_v * (double) A_v * 32768.0);	

	USART3_PINSET_TX();
	myprintf("V1= %lf Volts\r\n",V1_RMS);
	USART3_PINSET_RX();

}

/**
* @brief Calculate and Print the RMS current
*		 of the power line.
* @param Parameter: None
* @retval None
*/
void CalcPrint_C1_RMS(void){	
	static double C1_RMS = 0;
	
	uint16_t C1_RMS_raw = 0x0000;
	C1_RMS_raw = C1_RMS_raw | ((uint16_t) CH1_RMS[1] >> 7);
	C1_RMS_raw = C1_RMS_raw | ((uint16_t) CH1_RMS[2] << 1);
	C1_RMS_raw = C1_RMS_raw | ((uint16_t) CH1_RMS[3] << 9);

	C1_RMS = (double) C1_RMS_raw * (double) V_ref / ((double) cal_i * (double) A_i * 131072.0 * (double) k_s * (double) k_int);
	
	USART3_PINSET_TX();
	myprintf("C1= %lf Amps\r\n",C1_RMS);
	USART3_PINSET_RX();
	
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
	
	ext_flash_write(FlashPointer + FlashAddr_Phase, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint16_t C1_PHA_raw;
	
	C1_PHA_raw = C1_PHA_raw | (uint16_t) C1_PHA[3] << 8;
	C1_PHA_raw = C1_PHA_raw | (uint16_t) C1_PHA[2];
	C1_PHA_raw = C1_PHA_raw & 0x1FFE;

	double phase = (double) C1_PHA_raw / (double) F_CLK * (double) 50 * 360.0;
	
	USART3_PINSET_TX();
	myprintf("Phase = %lf degrees\r\n\r\n", phase);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
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
	
	ext_flash_write(FlashPointer + FlashAddr_Active_Energy, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	
	uint32_t Active_Energy_raw = 0x0000;
	Active_Energy_raw = Active_Energy_raw | (uint16_t) PH1_Active_Energy[3] << 24;
	Active_Energy_raw = Active_Energy_raw | (uint16_t) PH1_Active_Energy[2] << 16;
	Active_Energy_raw = Active_Energy_raw | (uint16_t) PH1_Active_Energy[1] << 8;
	Active_Energy_raw = Active_Energy_raw | (uint16_t) PH1_Active_Energy[0];

	double Active_Energy = (double)Active_Energy_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);

	USART3_PINSET_TX();
	myprintf("Active Energy = %lf Watts\r\n\r\n", Active_Energy);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
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
	
	ext_flash_write(FlashPointer + FlashAddr_Funda_Energy, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t Funda_Energy_raw = 0x0000;
	Funda_Energy_raw = Funda_Energy_raw | (uint16_t) PH1_Fundamental_Energy[3] << 24;
	Funda_Energy_raw = Funda_Energy_raw | (uint16_t) PH1_Fundamental_Energy[2] << 16;
	Funda_Energy_raw = Funda_Energy_raw | (uint16_t) PH1_Fundamental_Energy[1] << 8;
	Funda_Energy_raw = Funda_Energy_raw | (uint16_t) PH1_Fundamental_Energy[0];

	double Funda_Energy = (double)Funda_Energy_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);


	USART3_PINSET_TX();
	myprintf("Fundamental Energy = %lf Watts\r\n\r\n", Funda_Energy);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
	
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
	
	ext_flash_write(FlashPointer + FlashAddr_React_Energy, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t React_Energy_raw = 0x0000;
	React_Energy_raw = React_Energy_raw | (uint16_t) PH1_Reactive_Energy[3] << 24;
	React_Energy_raw = React_Energy_raw | (uint16_t) PH1_Reactive_Energy[2] << 16;
	React_Energy_raw = React_Energy_raw | (uint16_t) PH1_Reactive_Energy[1] << 8;
	React_Energy_raw = React_Energy_raw | (uint16_t) PH1_Reactive_Energy[0];

	double React_Energy = (double)React_Energy_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);
	
	USART3_PINSET_TX();
	myprintf("Reactive Energy = %lf Watts\r\n\r\n", React_Energy);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
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
	
	ext_flash_write(FlashPointer + FlashAddr_App_Energy, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t App_Energy_raw = 0x0000;
	App_Energy_raw = App_Energy_raw | (uint16_t) PH1_Apparent_Energy[3] << 24;
	App_Energy_raw = App_Energy_raw | (uint16_t) PH1_Apparent_Energy[2] << 16;
	App_Energy_raw = App_Energy_raw | (uint16_t) PH1_Apparent_Energy[1] << 8;
	App_Energy_raw = App_Energy_raw | (uint16_t) PH1_Apparent_Energy[0];

	double App_Energy = (double)App_Energy_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);


	USART3_PINSET_TX();
	myprintf("Apparent Energy = %lf Watts\r\n\r\n", App_Energy);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
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
	
	ext_flash_write(FlashPointer + FlashAddr_Active_Power, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t Active_Pwr_raw = 0x00000000;
	Active_Pwr_raw = Active_Pwr_raw | (uint16_t) PH1_Active_Power[3] << 24;
	Active_Pwr_raw = Active_Pwr_raw | (uint16_t) PH1_Active_Power[2] << 16;
	Active_Pwr_raw = Active_Pwr_raw | (uint16_t) PH1_Active_Power[1] << 8;
	Active_Pwr_raw = Active_Pwr_raw | (uint16_t) PH1_Active_Power[0];

	Active_Pwr_raw = Active_Pwr_raw & 0x1FFFFFFF;

	double Active_Pwr = (double)Active_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / (3600.0 * (double)D_CLK * (double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 131072.0);

	USART3_PINSET_TX();
	myprintf("Active Power = %lf WattHrs\r\n\r\n", Active_Pwr);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
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
	
	ext_flash_write(FlashPointer + FlashAddr_Funda_Pwr, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t Funda_Pwr_raw = 0x00000000;
	Funda_Pwr_raw = Funda_Pwr_raw | (uint16_t) PH1_Fundamental_Power[3] << 24;
	Funda_Pwr_raw = Funda_Pwr_raw | (uint16_t) PH1_Fundamental_Power[2] << 16;
	Funda_Pwr_raw = Funda_Pwr_raw | (uint16_t) PH1_Fundamental_Power[1] << 8;
	Funda_Pwr_raw = Funda_Pwr_raw | (uint16_t) PH1_Fundamental_Power[0];

	Funda_Pwr_raw = Funda_Pwr_raw & 0x1FFFFFFF;

	double Funda_Pwr = (double)Funda_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / (3600.0 * (double)D_CLK * (double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 131072.0);
	
	USART3_PINSET_TX();
	myprintf("Fundamental Power = %lf WattHrs\r\n\r\n", Funda_Pwr);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
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
	
	ext_flash_write(FlashPointer + FlashAddr_React_Pwr, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t React_Pwr_raw = 0x00000000;
	React_Pwr_raw = React_Pwr_raw | (uint16_t) PH1_Reactive_Power[3] << 24;
	React_Pwr_raw = React_Pwr_raw | (uint16_t) PH1_Reactive_Power[2] << 16;
	React_Pwr_raw = React_Pwr_raw | (uint16_t) PH1_Reactive_Power[1] << 8;
	React_Pwr_raw = React_Pwr_raw | (uint16_t) PH1_Reactive_Power[0];

	React_Pwr_raw = React_Pwr_raw & 0x1FFFFFFF;

	double React_Pwr = (double)React_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / (3600.0 * (double)D_CLK * (double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 131072.0);
	
	USART3_PINSET_TX();
	myprintf("Reactive Power = %lf WattHrs\r\n\r\n", React_Pwr);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
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
	
	ext_flash_write(FlashPointer + FlashAddr_App_RMS_Pwr, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t App_RMS_Pwr_raw = 0x00000000;
	App_RMS_Pwr_raw = App_RMS_Pwr_raw | (uint16_t) PH1_Apparent_RMS_Power[3] << 24;
	App_RMS_Pwr_raw = App_RMS_Pwr_raw | (uint16_t) PH1_Apparent_RMS_Power[2] << 16;
	App_RMS_Pwr_raw = App_RMS_Pwr_raw | (uint16_t) PH1_Apparent_RMS_Power[1] << 8;
	App_RMS_Pwr_raw = App_RMS_Pwr_raw | (uint16_t) PH1_Apparent_RMS_Power[0];

	App_RMS_Pwr_raw = App_RMS_Pwr_raw & 0x1FFFFFFF;

	double Apparent_RMS_Power = (double)App_RMS_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / (3600.0 * (double)D_CLK * (double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 131072.0);
	
	USART3_PINSET_TX();
	myprintf("Apparent_RMS Power = %lf WattHrs\r\n\r\n", Apparent_RMS_Power);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
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
	
	ext_flash_write(FlashPointer + FlashAddr_Tot_Active_Energy, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t Tot_Active_Pwr_raw = 0x00000000;
	Tot_Active_Pwr_raw = Tot_Active_Pwr_raw | (uint16_t) Total_Active_Energy[3] << 24;
	Tot_Active_Pwr_raw = Tot_Active_Pwr_raw | (uint16_t) Total_Active_Energy[2] << 16;
	Tot_Active_Pwr_raw = Tot_Active_Pwr_raw | (uint16_t) Total_Active_Energy[1] << 8;
	Tot_Active_Pwr_raw = Tot_Active_Pwr_raw | (uint16_t) Total_Active_Energy[0];

	double Tot_Active_Pwr = (double)Tot_Active_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);
	
	USART3_PINSET_TX();
	myprintf("Total Active Energy = %lf Watts\r\n\r\n", Tot_Active_Pwr);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
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
	
	ext_flash_write(FlashPointer + FlashAddr_Tot_Funda_Energy, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t Tot_Funda_Pwr_raw = 0x00000000;
	Tot_Funda_Pwr_raw = Tot_Funda_Pwr_raw | (uint16_t) Total_Fundamental_Energy[3] << 24;
	Tot_Funda_Pwr_raw = Tot_Funda_Pwr_raw | (uint16_t) Total_Fundamental_Energy[2] << 16;
	Tot_Funda_Pwr_raw = Tot_Funda_Pwr_raw | (uint16_t) Total_Fundamental_Energy[1] << 8;
	Tot_Funda_Pwr_raw = Tot_Funda_Pwr_raw | (uint16_t) Total_Fundamental_Energy[0];

	double Tot_Funda_Pwr = (double)Tot_Funda_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);
	
	USART3_PINSET_TX();
	myprintf("Total Fundamental Energy = %lf Watts\r\n\r\n", Tot_Funda_Pwr);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
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
	
	ext_flash_write(FlashPointer + FlashAddr_Tot_React_Energy, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t Tot_React_Pwr_raw = 0x00000000;
	Tot_React_Pwr_raw = Tot_React_Pwr_raw | (uint16_t) Total_Reactive_Energy[3] << 24;
	Tot_React_Pwr_raw = Tot_React_Pwr_raw | (uint16_t) Total_Reactive_Energy[2] << 16;
	Tot_React_Pwr_raw = Tot_React_Pwr_raw | (uint16_t) Total_Reactive_Energy[1] << 8;
	Tot_React_Pwr_raw = Tot_React_Pwr_raw | (uint16_t) Total_Reactive_Energy[0];

	double Tot_React_Pwr = (double)Tot_React_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);
	
	USART3_PINSET_TX();
	myprintf("Total Reactive Energy = %f Watts\r\n\r\n", Tot_React_Pwr);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
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
	
	ext_flash_write(FlashPointer + FlashAddr_Tot_App_Energy, (char*) FlashBuffer, 8);
	ext_flash_last_write_or_erase_done();
	
	uint32_t Tot_App_Pwr_raw = 0x00000000;
	Tot_App_Pwr_raw = Tot_App_Pwr_raw | (uint16_t) Total_Apparent_Energy[3] << 24;
	Tot_App_Pwr_raw = Tot_App_Pwr_raw | (uint16_t) Total_Apparent_Energy[2] << 16;
	Tot_App_Pwr_raw = Tot_App_Pwr_raw | (uint16_t) Total_Apparent_Energy[1] << 8;
	Tot_App_Pwr_raw = Tot_App_Pwr_raw | (uint16_t) Total_Apparent_Energy[0];

	double Tot_App_Pwr = (double)Tot_App_Pwr_raw * ((double)V_ref * (double)V_ref * (1.0 + (double)R1/(double)R2)) / ((double)k_int * (double)A_v * (double)A_i * (double)k_s * (double)cal_v * (double)cal_i * 268435456.0);
	
	USART3_PINSET_TX();
	myprintf("Total Apparent Energy = %lf Watts\r\n\r\n", Tot_App_Pwr);
	USART3_PINSET_RX();
	
	HAL_Delay(1);
}


/**
* @brief Copy the src string into dest string
* @param Pointer: dest[], src[] 
* @param Parameter: size of string
* @retval None
*/
void uint8_cpy(uint8_t dest[], uint8_t src[], uint8_t size){
	for (int i = 0; i < size; i++){
		dest[i] = src[i];
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
