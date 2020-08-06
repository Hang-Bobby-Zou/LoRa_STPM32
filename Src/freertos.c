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
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LORAMAC_SEND_RETRY_COUNT_MAX 48
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

int LoRa_DL_Flag = 0;

extern uint8_t *LoRa_RxBuf;

//Raw data from STPM32 defines

extern uint8_t PH_Period								[5];
extern uint8_t CH1_RMS									[5];
extern uint8_t C1_PHA										[5];

extern uint8_t PH1_Active_Energy				[5];
extern uint8_t PH1_Fundamental_Energy		[5];
extern uint8_t PH1_Reactive_Energy			[5];
extern uint8_t PH1_Apparent_Energy			[5];
		
extern uint8_t PH1_Active_Power					[5];
extern uint8_t PH1_Fundamental_Power		[5];
extern uint8_t PH1_Reactive_Power				[5];
extern uint8_t	PH1_Apparent_RMS_Power	[5];

extern uint8_t Total_Active_Energy			[5];
extern uint8_t Total_Fundamental_Energy	[5];
extern uint8_t Total_Reactive_Energy		[5];
extern uint8_t Total_Apparent_Energy		[5];

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
	HAL_UART_Receive_IT(&huart3, aRxBuffer, 4); 
	//myprintf("USART3 Running\r\n");
	/* Infinite loop */
  for(;;)
  {		
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


		if (USART3_RxFlag == 1){
			
			myprintf("USART3 Receive: %x %x %x %x \r\n", aRxBuffer[0], aRxBuffer[1], aRxBuffer[2], aRxBuffer[3]);
			
			USART3_RxFlag = 0;
			HAL_UART_Receive_IT(&huart3, aRxBuffer, 4); 
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

	DEBUG("LoRaMAC Init...");
	LoRaMAC_Init();
	DEBUG("LoRaMAC Init Done");
	
	DEBUG("LoRaMAC Join...");
	LoRaMAC_Join();
	DEBUG("LoRaMAC Join Done");
	
	/* Infinite loop */
  for(;;)
  {
		myprintf("\r\n");
		INFO("Entering LoRa Task\r\n");
		HAL_NVIC_EnableIRQ(TIM7_IRQn);	//Enable TIM7 Irq since it is disabled while other task is running
		if (LoRa_CheckStateIDLE() == true){
			if(LoRaMAC_Send() == -1){ //If send was not successful
				if (loramac_send_retry_count < LORAMAC_SEND_RETRY_COUNT_MAX){
					loramac_send_retry_count ++;
					WARN("LoRaMAC Send Failed, retrying for %d time...", loramac_send_retry_count);
				}			
			} else {	
				INFO("LoRaMAC Send Succeed!");
				loramac_send_retry_count = 0;
			}
		}
		DelayMsPoll(10000);
		
		if (LoRa_DL_Flag == 1){
			INFO("LoRa DownLink received!");
			INFO("LoRa DownLink buffer: %x %x %x %x",LoRa_RxBuf[0], LoRa_RxBuf[1], LoRa_RxBuf[2], LoRa_RxBuf[3]);
			LoRa_DL_Flag = 0;
		} else if (LoRa_DL_Flag == 0){
			INFO("LoRa DownLink no new message");
		}
		
		INFO("Blocking LoRa Task for %d miliseconds.", LoRa_Block_Time);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		vTaskDelay(pdMS_TO_TICKS( LoRa_Block_Time));
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
