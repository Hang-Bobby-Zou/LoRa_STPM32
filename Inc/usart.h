/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
#define  BUFFER_SIZE  (255)
void USAR_UART_IDLECallback(UART_HandleTypeDef *huart);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);

#define UART_BUF_SIZE 1
extern  uint8_t UTART_BUF[UART_BUF_SIZE];

#define REC_LENGTH 1
#define MAX_REC_LENGTH 2048

extern uint8_t RxBuffer[MAX_REC_LENGTH];
extern uint8_t RxFlag;
extern uint16_t RxCounter;
extern uint8_t RxTemp[REC_LENGTH];


#define LOGLEVEL LOGINFO

enum log_levels{
		LOGERROR 		= 3,
		LOGWARNING 	= 4,
		LOGINFO 		= 6,
};

#ifdef LOGLEVEL
#define ERROR(format,...)\
if (LOGLEVEL >= LOGERROR) {\
	myprintf("/r/n>>ERROR: "format, ##__VA_ARGS___);\
}
#define WARN(format,...)\
if (LOGLEVEL >= LOGWARNING) {\
	myprintf("/r/n>>WATNING: "format, ##__VA_ARGS___);\
}
#define INFO(format,...)\
if (LOGLEVEL >= LOGINFO) {\
	myprintf("/r/n>>INFO: "format, ##__VA_ARGS___);\
}
#else		//Not LOGLEVEL
#define ERROR(args...) do {;} while(0)
#define WARN(args...) do {;} while(0)
#define INFO(args...) do {;} while(0)
#endif //LOGLEVEL
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void myprintf(char *fmt,...);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
