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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RXBUFFERSIZE 8
#define TXBUFFERSIZE 8


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t aRxBuffer[RXBUFFERSIZE];
uint8_t SendBuffer[1];
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId IDLEHandle;
osThreadId SP1Handle;
osThreadId USART1Handle;
osThreadId USART3Handle;
osThreadId USBHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void defaultIDLE(void const * argument);
void StartSPI1(void const * argument);
void StartUSART1(void const * argument);
void StartUSART3(void const * argument);
void StartUSB(void const * argument);

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

  /* definition and creation of SP1 */
  osThreadDef(SP1, StartSPI1, osPriorityBelowNormal, 0, 128);
  SP1Handle = osThreadCreate(osThread(SP1), NULL);

  /* definition and creation of USART1 */
  osThreadDef(USART1, StartUSART1, osPriorityBelowNormal, 0, 128);
  USART1Handle = osThreadCreate(osThread(USART1), NULL);

  /* definition and creation of USART3 */
  osThreadDef(USART3, StartUSART3, osPriorityNormal, 0, 128);
  USART3Handle = osThreadCreate(osThread(USART3), NULL);

  /* definition and creation of USB */
  osThreadDef(USB, StartUSB, osPriorityBelowNormal, 0, 128);
  USBHandle = osThreadCreate(osThread(USB), NULL);

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
			SendBuffer[0] = 0x42;	// Ascii Table Charather : A
			/* Start the transmission process */
			/* While the UART in reception process, user can transmit data through "SendBuffer" buffer */
			if(HAL_UART_Transmit_IT(&huart3, (uint8_t*)SendBuffer, sizeof(SendBuffer))!= HAL_OK)
			{
					Error_Handler();
			}
			while(huart3.gState == HAL_UART_STATE_BUSY_TX){
			}
		osDelay(95);
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
    osDelay(1);
  }
  /* USER CODE END defaultIDLE */
}

/* USER CODE BEGIN Header_StartSPI1 */
/**
* @brief Function implementing the SP1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSPI1 */
void StartSPI1(void const * argument)
{
  /* USER CODE BEGIN StartSPI1 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSPI1 */
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
  /* Infinite loop */
  for(;;)
  {
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
  /* Infinite loop */
  for(;;)
  {
		  SendBuffer[0] = 0x41;	// Ascii Table Charather : A
			/* Start the transmission process */
			/* While the UART in reception process, user can transmit data through "SendBuffer" buffer */
			if(HAL_UART_Transmit_IT(&huart3, (uint8_t*)SendBuffer, sizeof(SendBuffer))!= HAL_OK)
			{
					Error_Handler();
			}
			while(huart3.gState == HAL_UART_STATE_BUSY_TX){
			}
			// Parallel Computing prioirty still need to be verified
			/*#### Verified ####*/
			
			
		osDelay(100); //This delay is in ms
  }
  /* USER CODE END StartUSART3 */
}

/* USER CODE BEGIN Header_StartUSB */
/**
* @brief Function implementing the USB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUSB */
void StartUSB(void const * argument)
{
  /* USER CODE BEGIN StartUSB */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUSB */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
