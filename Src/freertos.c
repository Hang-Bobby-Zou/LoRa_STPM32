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
#include "stdio.h"

#include "ext_flash.h"
#include "ext_flash_tb.h"
#include "STPM32.h"
#include "LoRa.h"
#include "HAL_LoRaMAC.h"
#include "HAL_spi.h"
#include "tim.h"
#include "usart.h"

#include "LoRaMac.h"
#include "Commissioning.h"
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

/* USART1(STPM32) Variables */
uint8_t 	ReadBuffer[5] 	= {0};
uint8_t	 	HAL_RxBuffer[5] = {0};
uint8_t 	i[1] 						= {0x2E};
int 			count 					= 0;
uint16_t 	FlashPointer 		= 0x00;
#define STPM32_Block_Time	90000

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


/* USART3(RS485) Variables */
uint8_t aRxBuffer[128] = {0};


/* SPI2(LoRa) Variables */
#define LORAMAC_SEND_RETRY_COUNT_MAX 48
uint8_t loramac_send_retry_count = 0;
int LoRa_Block_Time = 120000;
int LoRa_DL_Flag = 0;
extern uint8_t *LoRa_RxBuf;
//extern LoRaMacFlags_t LoRaMacFlags;
uint8_t LoRa_Sendtype = 0x10;
char LoRa_UL_Buffer[8];
uint32_t LoRa_UL_Addr = 0x00;

/* SYSTEM Variables */
static int USART3_RxFlag = 0;
static int USART3_TxFlag = 0;

UBaseType_t USART1_Priority;
UBaseType_t USART3_Priority;
UBaseType_t SPI1_Priority;
UBaseType_t SPI2_Priority;


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
  osThreadDef(USART1, StartUSART1, osPriorityAboveNormal, 0, 256);
  USART1Handle = osThreadCreate(osThread(USART1), NULL);

  /* definition and creation of USART3 */
  osThreadDef(USART3, StartUSART3, osPriorityNormal, 0, 256);
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
			INFO("Blocking STPM32 for %d miliseconds",STPM32_Block_Time);
			
			vTaskDelay (pdMS_TO_TICKS( STPM32_Block_Time ));	//If walks around for 1 term, then block itself
		}
		
		if (FlashPointer > 0x00FFFF){
			FlashPointer = 0x000000;
			
			USART3_PINSET_TX();
			INFO("!!!Flash Full!!!");
			INFO("...Overwriting Previous Info...\r\n");
			USART3_PINSET_RX();
		}

		if (USART1_RxFlag == 1){
		 	
			//INFO("FlashPointer: %x", FlashPointer);
			
			HAL_RxBuffer[0] = ReadBuffer[0];
		 	HAL_RxBuffer[1] = ReadBuffer[1];
		 	HAL_RxBuffer[2] = ReadBuffer[2];
		 	HAL_RxBuffer[3] = ReadBuffer[3];
			HAL_RxBuffer[4] = ReadBuffer[4];

			if (count == 5){		//Wait for the 5 th iteration so the data is stable
					if (i[0] == dsp_reg1){
						uint8_cpy(PH_Period, HAL_RxBuffer, 5);
						CalcPrint_Freq();
						INFO("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], PH_Period[0], PH_Period[1], PH_Period[2], PH_Period[3], PH_Period[4]);
					} else if (i[0] == dsp_reg17){
						uint8_cpy(C1_PHA, HAL_RxBuffer, 5);
						CalcPrint_Phase();
						INFO("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], C1_PHA[0], C1_PHA[1], C1_PHA[2], C1_PHA[3], C1_PHA[4]);
					} else if (i[0] == dsp_reg14){
						uint8_cpy(CH1_RMS,HAL_RxBuffer,5);
						CalcPrint_V1_RMS();
						CalcPrint_C1_RMS();
						INFO("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], CH1_RMS[0], CH1_RMS[1], CH1_RMS[2], CH1_RMS[3], CH1_RMS[4]);
					} else if (i[0] == ph1_reg1){
						uint8_cpy(PH1_Active_Energy, HAL_RxBuffer, 5);
						CalcPrint_Active_Energy();
						INFO("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], PH1_Active_Energy[0], PH1_Active_Energy[1], PH1_Active_Energy[2], PH1_Active_Energy[3], PH1_Active_Energy[4]);
					} else if (i[0] == ph1_reg2){
						uint8_cpy(PH1_Fundamental_Energy, HAL_RxBuffer, 5);
						CalcPrint_Funda_Energy();
						INFO("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], PH1_Fundamental_Energy[0], PH1_Fundamental_Energy[1], PH1_Fundamental_Energy[2], PH1_Fundamental_Energy[3], PH1_Fundamental_Energy[4]);
					} else if (i[0] == ph1_reg3){
						uint8_cpy(PH1_Reactive_Energy, HAL_RxBuffer, 5);
						CalcPrint_React_Energy();
						INFO("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], PH1_Reactive_Energy[0], PH1_Reactive_Energy[1], PH1_Reactive_Energy[2], PH1_Reactive_Energy[3], PH1_Reactive_Energy[4]);
					} else if (i[0] == ph1_reg4){
						uint8_cpy(PH1_Apparent_Energy, HAL_RxBuffer,5);
						CalcPrint_App_Energy();
						INFO("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], PH1_Apparent_Energy[0], PH1_Apparent_Energy[1], PH1_Apparent_Energy[2], PH1_Apparent_Energy[3], PH1_Apparent_Energy[4]);
					} else if (i[0] == ph1_reg5){
						uint8_cpy(PH1_Active_Power, HAL_RxBuffer, 5);
						CalcPrint_Active_Pwr();
						INFO("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], PH1_Active_Power[0], PH1_Active_Power[1], PH1_Active_Power[2], PH1_Active_Power[3], PH1_Active_Power[4]);
					} else if (i[0] == ph1_reg6){
						uint8_cpy(PH1_Fundamental_Power, HAL_RxBuffer, 5);
						CalcPrint_Funda_Pwr();
						INFO("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], PH1_Fundamental_Power[0], PH1_Fundamental_Power[1], PH1_Fundamental_Power[2], PH1_Fundamental_Power[3], PH1_Fundamental_Power[4]);
					} else if (i[0] == ph1_reg7){
						uint8_cpy(PH1_Reactive_Power, HAL_RxBuffer, 5);
						CalcPrint_React_Pwr();
						INFO("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], PH1_Reactive_Power[0], PH1_Reactive_Power[1], PH1_Reactive_Power[2], PH1_Reactive_Power[3], PH1_Reactive_Power[4]);
					} else if (i[0] == ph1_reg8){
						uint8_cpy(PH1_Apparent_RMS_Power, HAL_RxBuffer, 5);
						CalcPrint_App_RMS_Pwr();
						INFO("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], PH1_Apparent_RMS_Power[0], PH1_Apparent_RMS_Power[1], PH1_Apparent_RMS_Power[2], PH1_Apparent_RMS_Power[3], PH1_Apparent_RMS_Power[4]);
					} else if (i[0] == tot_reg1){
						uint8_cpy(Total_Active_Energy, HAL_RxBuffer, 5);
						CalcPrint_Tot_Active_Energy();
						INFO("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], Total_Active_Energy[0], Total_Active_Energy[1], Total_Active_Energy[2], Total_Active_Energy[3], Total_Active_Energy[4]);
					} else if (i[0] == tot_reg2){
						uint8_cpy(Total_Fundamental_Energy, HAL_RxBuffer, 5);
						CalcPrint_Tot_Funda_Energy();
						INFO("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], Total_Fundamental_Energy[0], Total_Fundamental_Energy[1], Total_Fundamental_Energy[2], Total_Fundamental_Energy[3], Total_Fundamental_Energy[4]);
					} else if (i[0] == tot_reg3){
						uint8_cpy(Total_Reactive_Energy, HAL_RxBuffer, 5);
						CalcPrint_Tot_React_Energy();
						INFO("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], Total_Reactive_Energy[0], Total_Reactive_Energy[1], Total_Reactive_Energy[2], Total_Reactive_Energy[3], Total_Reactive_Energy[4]);
					} else if (i[0] == tot_reg4){
						uint8_cpy(Total_Apparent_Energy, HAL_RxBuffer, 5);
						CalcPrint_Tot_App_Energy();
						INFO("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], Total_Apparent_Energy[0], Total_Apparent_Energy[1], Total_Apparent_Energy[2], Total_Apparent_Energy[3], Total_Apparent_Energy[4]);
					}
				
				i[0] += 0x02;
				
				count = 0;
			}

			count++;

		 	USART1_RxFlag = 0;
		}
		
		ReadMsgOnly(i[0],ReadBuffer);
		vTaskDelay (pdMS_TO_TICKS( 100 ));
		//osDelay(1);
	*/
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
	HAL_UART_Receive_IT(&huart3, aRxBuffer, 128); 
	/* Infinite loop */
  for(;;)
  {		
	
		if (USART3_RxFlag == 1){
			
			/*
			//Get all aRxBuffer values into local values
			uint8_t CommandByte 	= aRxBuffer[0];
			uint8_t Message1			=	aRxBuffer[1];
			uint8_t Message2			=	aRxBuffer[2];
			uint8_t Message3			=	aRxBuffer[3];
			uint8_t Message4			=	aRxBuffer[4];
			uint8_t Message5			=	aRxBuffer[5];
			uint8_t Message6			=	aRxBuffer[6];
			uint8_t EndByte 			= aRxBuffer[7];
			
			
			if (EndByte == 0xAA){
				
				DEBUG("RS485 Receive: %x %x %x %x %x %x %x %x\r\n", CommandByte, Message1, Message2, Message3, Message4, Message5, Message6, EndByte);
				INFO("RS485 Receive:");
				INFO("Command: %x", CommandByte);
				INFO("Message: %x %x %x %x %x %x", Message1, Message2, Message3, Message4, Message5, Message6);
				
				//Command Porcess
				if( CommandByte == 0x00 ){
					INFO("SYSTEM Status");
					
				} else if ( CommandByte == 0x01 ){
					INFO("STPM32 Status");
					
				} else if ( CommandByte == 0x02 ){
					INFO("Flash Status");
					
				} else if ( CommandByte == 0x03 ){
					INFO("LoRa Status");
					
				} else if ( CommandByte	== 0x10 ){
					INFO("Auto Rotate Raw Data Upload");
					
					LoRa_Sendtype = CommandByte;
					
				} else if ( CommandByte == 0x11 ){
					INFO("Auto Rotate Real Data Upload");
					
					LoRa_Sendtype = CommandByte;
					
				} else if ( CommandByte == 0x12 ){
					INFO("Raw Data Upload from a specific address");
					
					LoRa_UL_Addr = LoRa_UL_Addr | Message1 << 24;
					LoRa_UL_Addr = LoRa_UL_Addr | Message2 << 16;
					LoRa_UL_Addr = LoRa_UL_Addr | Message3 << 8;
					LoRa_UL_Addr = LoRa_UL_Addr | Message4;
					
					LoRa_Sendtype = CommandByte;
					
				} else if ( CommandByte == 0x13 ){
					INFO("Real Data Upload from a specific address");
					
					LoRa_UL_Addr = LoRa_UL_Addr | Message1 << 24;
					LoRa_UL_Addr = LoRa_UL_Addr | Message2 << 16;
					LoRa_UL_Addr = LoRa_UL_Addr | Message3 << 8;
					LoRa_UL_Addr = LoRa_UL_Addr | Message4;
					
					LoRa_Sendtype = CommandByte;
					
				} else if ( CommandByte == 0x14 ){
					INFO("Return specific content from flash");
					
					//Process address
					uint32_t addr = 0x000000;
			
					addr = addr | Message1 << 24;
					addr = addr | Message2 << 16;
					addr = addr | Message3 << 8;
					addr = addr | Message4;
					
					//Read data
					char flash_data[8] = {0};
					ext_flash_read(addr, flash_data, 8);

					INFO("Reading addr: %x ",addr);
					INFO("Data: %x %x %x %x %x %x %x %x", flash_data[0], flash_data[1], flash_data[2], flash_data[3], flash_data[4], flash_data[5], flash_data[6], flash_data[7]);
					
				} else if ( CommandByte == 0x15 ){
					INFO("Erase specific content from flash");
					
					//Process address
					uint32_t addr = 0x000000;
					
					addr = addr | Message1 << 24;
					addr = addr | Message2 << 16;
					addr = addr | Message3 << 8;
					addr = addr | Message4;
					
					//Do erase
					char EraseFF[8];
					EraseFF[0] = 0xFF; EraseFF[1] = 0xFF; EraseFF[2] = 0xFF; EraseFF[3] = 0xFF;
					EraseFF[4] = 0xFF; EraseFF[5] = 0xFF; EraseFF[6] = 0xFF; EraseFF[7] = 0xFF;
					ext_flash_write(addr, EraseFF, 8);
					ext_flash_last_write_or_erase_done();
					
					//Check erase
					char flash_data[8] = {0};
					
					ext_flash_read(addr, flash_data, 8);
					
					if (strcmp(flash_data, EraseFF) == 0) {
						INFO("Erase address %x done",addr);  
					} else {
						WARN("Erase address %x error", addr);
					}
					
				} else if (CommandByte == 0x16){ 
					INFO("Erase specific sector");
					
					//Process address
					uint32_t addr = 0x000000;
					
					addr = addr | Message1 << 8;
					addr = addr | Message2;
					
					if(addr <= 0x01FF){
						//Erase sector
						ext_flash_erase_sector( addr );
						ext_flash_last_write_or_erase_done();
					
						INFO("Erase sector %d done", addr);
					} else {
						WARN("Sector address %d invalid", addr);
					}
					
				} else if (CommandByte == 0x17){ 
					INFO("Erase specific block");
					
					//Process address
					uint32_t addr = Message1;
					
					//Erase block
					if (addr <= 0x1F){	
						ext_flash_erase_block( addr );
						ext_flash_last_write_or_erase_done();
					
						INFO("Erase block %d done", addr);
					} else {
						WARN("Block address %d invalid", addr);
					}
					
				} else if ( CommandByte == 0xFF ){
					INFO("Emergency Stop");
				}
				
				
			} else {
				WARN("RS485 End Byte Error");
			}
			*/
			
			if (aRxBuffer[0] == 0x41 && aRxBuffer[1] == 0X54){
				if(aRxBuffer[2] == 0x5A){
					//Reset Board
				}
				
				if(aRxBuffer[2] == 0x2B){
					
				}
				
				
			} else {
				WARN("Not AT Command");
			}
			
			
			USART3_RxFlag = 0;
			HAL_UART_Receive_IT(&huart3, aRxBuffer, 128); 
		}
		//osDelay(1); //This delay is in ms
		
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
	
	LoRa_UL_Addr = 0x000000;
	/* Infinite loop */
  for(;;)
  {
		/*
		HAL_NVIC_EnableIRQ(TIM7_IRQn);	//Enable TIM7 Irq since it is disabled while other task is running
		
		INFO("\r\nEntering LoRa Task\r\n");
		
		DelayMsPoll(1000);
		
				//Check LoRa Upload Mode
				if (LoRa_Sendtype == 0x10){						//Auto Rotate Raw
					INFO("LoRa: Auto Rotate Raw");
					
					LoRa_UL_Addr = 0x000000;
					
					while(LoRa_UL_Addr <= 0x0E0000){
						ext_flash_read(LoRa_UL_Addr + FlashPointer - 0x08, LoRa_UL_Buffer, 8);		//Here to avoid flash pointer advance before read
						
						INFO("Auto Rotate Raw, LoRa_UL_Addr = %x, LoRa_UL_Buffer: %x %x %x %x %x %x %x %x", LoRa_UL_Addr + FlashPointer - 0x08, LoRa_UL_Buffer[0], LoRa_UL_Buffer[1], LoRa_UL_Buffer[2], LoRa_UL_Buffer[3], LoRa_UL_Buffer[4], LoRa_UL_Buffer[5], LoRa_UL_Buffer[6], LoRa_UL_Buffer[7]);
						
						//LoRa Send Code
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
						
						LoRa_UL_Addr += 0x010000;
					}
					
					INFO("Blocking LoRa Task for %d miliseconds.", LoRa_Block_Time);
					HAL_NVIC_DisableIRQ(TIM7_IRQn);
					vTaskDelay(pdMS_TO_TICKS( LoRa_Block_Time));
					
				} else if (LoRa_Sendtype == 0x11){		//Auto Rotate Real
					INFO("LoRa: Auto Rotate Real");
					
					LoRa_UL_Addr = 0x100000;
					
					while(LoRa_UL_Addr <= 0x1F0000){
						ext_flash_read(LoRa_UL_Addr + FlashPointer - 0x08, LoRa_UL_Buffer, 8);		//Here to avoid flash pointer advance before read
						
						INFO("Auto Rotate Raw, LoRa_UL_Addr = %x, LoRa_UL_Buffer: %x %x %x %x %x %x %x %x", LoRa_UL_Addr + FlashPointer - 0x08, LoRa_UL_Buffer[0], LoRa_UL_Buffer[1], LoRa_UL_Buffer[2], LoRa_UL_Buffer[3], LoRa_UL_Buffer[4], LoRa_UL_Buffer[5], LoRa_UL_Buffer[6], LoRa_UL_Buffer[7]);
						
						//LoRa Send Code
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
						
						LoRa_UL_Addr += 0x010000;
					}
					
					INFO("Blocking LoRa Task for %d miliseconds.", LoRa_Block_Time);
					HAL_NVIC_DisableIRQ(TIM7_IRQn);
					vTaskDelay(pdMS_TO_TICKS( LoRa_Block_Time));
					
				} else if (LoRa_Sendtype == 0x12 || LoRa_Sendtype == 0x13){		//Specific UL
					INFO("LoRa: Specific upload");
					ext_flash_read(LoRa_UL_Addr + FlashPointer - 0x08, LoRa_UL_Buffer, 8);
					
					INFO("Specific UL, LoRa_UL_Addr = %x, LoRa_UL_Buffer: %x %x %x %x %x %x %x %x", LoRa_UL_Addr + FlashPointer - 0x08, LoRa_UL_Buffer[0], LoRa_UL_Buffer[1], LoRa_UL_Buffer[2], LoRa_UL_Buffer[3], LoRa_UL_Buffer[4], LoRa_UL_Buffer[5], LoRa_UL_Buffer[6], LoRa_UL_Buffer[7]);
					
					//LoRa Send Code
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
					
				} else {															//Sendtype Invalid
					WARN("Sendtype Invalid");
				}
				*/
				/*
				//LoRa Send Code
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
				*/
	}


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
	UNUSED(USART3_TxFlag);
	
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
