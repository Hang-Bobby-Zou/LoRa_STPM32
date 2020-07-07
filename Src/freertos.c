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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint8_t ReadBuffer[5] = {0};
uint8_t RxBuffer[5] = {0};
uint8_t i[1] = {0x48};

uint8_t CH1_RMS									[5] = {0};
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
			i[0] = 0x48;
		}
		
		if (USART1_RxFlag == 1){
			RxBuffer[0] = ReadBuffer[0];
			RxBuffer[1] = ReadBuffer[1];
			RxBuffer[2] = ReadBuffer[2];
			RxBuffer[3] = ReadBuffer[3];
			RxBuffer[4] = ReadBuffer[4];
			
			if (i[0] == dsp_reg14){
				uint8_cpy(CH1_RMS,RxBuffer,5);
			} else if (i[0] == ph1_reg1){
				uint8_cpy(PH1_Active_Energy, RxBuffer, 5);
			} else if (i[0] == ph1_reg2){
				uint8_cpy(PH1_Fundamental_Energy, RxBuffer, 5);
			} else if (i[0] == ph1_reg3){
				uint8_cpy(PH1_Reactive_Energy, RxBuffer, 5);
			} else if (i[0] == ph1_reg4){
				uint8_cpy(PH1_Apparent_Energy, RxBuffer,5 );
			} else if (i[0] == ph1_reg5){
				uint8_cpy(PH1_Active_Power, RxBuffer, 5);
			} else if (i[0] == ph1_reg6){
				uint8_cpy(PH1_Fundamental_Power, RxBuffer, 5);
			} else if (i[0] == ph1_reg7){
				uint8_cpy(PH1_Reactive_Power, RxBuffer, 5);
			} else if (i[0] == ph1_reg8){
				uint8_cpy(PH1_Apparent_RMS_Power, RxBuffer, 5);
			} else if (i[0] == tot_reg1){
				uint8_cpy(Total_Active_Energy, RxBuffer, 5);
			} else if (i[0] == tot_reg2){
				uint8_cpy(Total_Fundamental_Energy, RxBuffer, 5);
			} else if (i[0] == tot_reg3){
				uint8_cpy(Total_Reactive_Energy, RxBuffer, 5);
			} else if (i[0] == tot_reg4){
				uint8_cpy(Total_Apparent_Energy, RxBuffer, 5);
			}
			
			USART3_PINSET_TX();
			myprintf("Address : %x \r\nData: %x | %x | %x | %x | %x \r\n", i[0], RxBuffer[0], RxBuffer[1], RxBuffer[2], RxBuffer[3], RxBuffer[4]);
			USART3_PINSET_RX();
			
			
			USART1_RxFlag = 0;
			i[0] += 2;
			
			vTaskDelay(pdMS_TO_TICKS( 1000 ));
		}
		
		ReadMsgOnly(i[0],ReadBuffer);
		
		//xTicksToDelay(pdMS_TO_TICKS( 1000 ));
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





void uint8_cpy(uint8_t dest[], uint8_t src[], uint8_t size){
	for (int i = 0; i<size; i++){
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
