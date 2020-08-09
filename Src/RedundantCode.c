/**
 * \addtogroup Redundant Code
 * @{
 * \file RedundantCode.c
 * \author Hang Bobby Zou
 * \brief Redundant Code Library
 */

/*============================================================================*/
/*										Explanations of RedundantCode.c													*/			
/*----------------------------------------------------------------------------*/
/*	This C file contains redundant Code that was used in the past							*/
/*	It is intended to save a copy as these are mostly test files							*/
/*	It also contains function prototypes that are easily forgotten						*/
/*============================================================================*/

//#include "main.h"
#include <stdlib.h>


//USART3 Read from flash code
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
		TimerStop( &TxTimeoutTimer );
		TimerSetValue( &TxTimeoutTimer, timeout );
		TimerStart( &TxTimeoutTimer );
*/
//__HAL_UART_FLUSH_DRREGISTER(&huart3);

//HAL_NVIC_DisableIRQ(USART3_IRQn);

//if (huart1.Instance == USART1){

//vTaskSuspend(USART1Handle);
//vTaskResume(USART1Handle);

//vTaskPrioritySet( USART3Handle, ( USART1_Priority - 2 ) );

//#define DelayMsPoll(x) { for (uint32_t j = 0; j < x; j++) {for (uint32_t i = 0; i < 8000; i++) {  }}}	


/* TIM7 init function
void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
*/



//This belongs to USART1
// // Save read data to ext_flash
// 			if (flash_count > 4096){
// 				if (flash_sector_pointer == 512)
// 					flash_sector_pointer = 0;
				
// 				ext_flash_write(flash_sector_pointer, (char*) flash_buffer, 4096);
// 				ext_flash_last_write_or_erase_done();
// 				flash_sector_pointer ++;
// 				flash_count = 0;
// 			}
			
// 			flash_buffer[0] 						= 0xFF;
			
// 			flash_buffer[1] 						= i[0];
// 			flash_buffer[2] 						= 0xFF;
// 			flash_buffer[flash_count+3] = RxBuffer[0];
// 			flash_buffer[flash_count+4] = RxBuffer[1];
// 			flash_buffer[flash_count+5] = RxBuffer[2];
// 			flash_buffer[flash_count+6] = RxBuffer[3];
// 			flash_buffer[flash_count+7] = RxBuffer[4];
			
// 			flash_count += 0x08;




// USART1 Pervious test code
// //This is for testing, let the program repeatedly read from STPM32
		// if (i[0] > 0x28){
		// 	i[0] = 0x00;
		// }
		
		// if (USART1_RxFlag == 1){
		//  	//***This exceutes when a Receive is complete***
		// 	// Get the info before been overwritten
			
			
			
		// 	//***Note: Somehow, CRC byte always comes one cycle late, but normally we ignore it.
		// 	USART3_PINSET_TX();
		//  	myprintf("Received! Read Address: %x | ReadBuffer: %x | %x | %x | %x | %x  \r\n",i[0], RxBuffer[0], RxBuffer[1], RxBuffer[2], RxBuffer[3], RxBuffer[4]);
		//  	USART3_PINSET_RX();
			
		// 	// Increment the read register by 2 and clear the flag to wait for the next operation.
		// 	i[0] += 0x02;
		// 	USART1_RxFlag = 0;
		// }
		
		// Calls read message only and get the buffer as soon as it returns
		// ReadMsgOnly(i[0],ReadBuffer);
		// Test End here



// if (i[0] == dsp_reg14){
// 				myprintf("Copying CH1_RMS\r\n");
// 				uint8_cpy(CH1_RMS,RxBuffer,5);
// 			} else if (i[0] == ph1_reg1){
// 				myprintf("Copying PH1_Active_Energy\r\n");
// 				uint8_cpy(PH1_Active_Energy, RxBuffer, 5);
// 			} else if (i[0] == ph1_reg2){
// 				myprintf("Copying PH1_Fundamental_Energy\r\n");
// 				uint8_cpy(PH1_Fundamental_Energy, RxBuffer, 5);
// 			} else if (i[0] == ph1_reg3){
// 				myprintf("Copying PH1_Reactive_Energy\r\n");
// 				uint8_cpy(PH1_Reactive_Energy, RxBuffer, 5);
// 			} else if (i[0] == ph1_reg4){
// 				myprintf("Copying PH1_Apparent_Energy\r\n");
// 				uint8_cpy(PH1_Apparent_Energy, RxBuffer,5);
// 			} else if (i[0] == ph1_reg5){
// 				myprintf("Copying PH1_Active_Power\r\n");
// 				uint8_cpy(PH1_Active_Power, RxBuffer, 5);
// 			} else if (i[0] == ph1_reg6){
// 				myprintf("Copying PH1_Fundamental_Power\r\n");
// 				uint8_cpy(PH1_Fundamental_Power, RxBuffer, 5);
// 			} else if (i[0] == ph1_reg7){
// 				myprintf("Copying PH1_Reactive_Power\r\n");
// 				uint8_cpy(PH1_Reactive_Power, RxBuffer, 5);
// 			} else if (i[0] == ph1_reg8){
// 				myprintf("Copying PH1_Apparent_RMS_Power\r\n");
// 				uint8_cpy(PH1_Apparent_RMS_Power, RxBuffer, 5);
// 			} else if (i[0] == tot_reg1){
// 				myprintf("Copying Total_Active_Energy\r\n");
// 				uint8_cpy(Total_Active_Energy, RxBuffer, 5);
// 			} else if (i[0] == tot_reg2){
// 				myprintf("Copying Total_Fundamental_Energy\r\n");
// 				uint8_cpy(Total_Fundamental_Energy, RxBuffer, 5);
// 			} else if (i[0] == tot_reg3){
// 				myprintf("Copying Total_Reactive_Energy\r\n");
// 				uint8_cpy(Total_Reactive_Energy, RxBuffer, 5);
// 			} else if (i[0] == tot_reg4){
// 				myprintf("Copying Total_Apparent_Energy\r\n");
// 				uint8_cpy(Total_Apparent_Energy, RxBuffer, 5);
// 			}


// bool Read_Reg(uint32_t ReadAddress, uint8_t* ReturnBuffer){
	
// 	uint8_t ReadBuffer[5] = {0};	
	
// 	while(1){
// 		if (USART1_RxFlag == 1){
// 			ReturnBuffer[0] = ReadBuffer[0];
// 			ReturnBuffer[1] = ReadBuffer[1];
// 			ReturnBuffer[2] = ReadBuffer[2];
// 			ReturnBuffer[3] = ReadBuffer[3];
// 			ReturnBuffer[4] = ReadBuffer[4];
			
// 			USART3_PINSET_TX();
// 			myprintf("Address: %x \r\nData: %x | %x | %x | %x | %x  \r\n",ReadAddress , ReturnBuffer[0], ReturnBuffer[1], ReturnBuffer[2], ReturnBuffer[3], ReturnBuffer[4]);
// 			USART3_PINSET_RX();
			
// 			USART1_RxFlag = 0;
// 			return true;
// 		}

// 		ReadMsgOnly(ReadAddress,ReadBuffer);
		
		
// 	}
	
// }





// bool SendMessage(uint32_t ReadAddress, uint8_t* ReadMessage ,uint32_t SendAddress, uint8_t* SendMessage) {
// 	/* 
// 		| ReadAddress | WriteAddress | LS Data [7:0] | MS Data [15:8] | CRC Byte |
// 		|		 0xFF 		|		 Address	 | 	 Message[0]  |   Message[1]   |    --    |
// 	*/
// 	uint8_t Buffer[5] = {0};
// 	uint8_t CRCBuffer[5] = {0};
	
// 	Buffer[0] = ReadAddress;
// 	Buffer[1] = SendAddress;
// 	Buffer[2] = SendMessage[0];
// 	Buffer[3] = SendMessage[1];
		
// 	CRCBuffer[0] = byteReverse(Buffer[0]);
// 	CRCBuffer[1] = byteReverse(Buffer[1]);
// 	CRCBuffer[2] = byteReverse(Buffer[2]);
// 	CRCBuffer[3] = byteReverse(Buffer[3]);
// 	Buffer[4] = byteReverse(CalcCRC8(CRCBuffer));
	
// 	HAL_UART_Receive_IT(&huart1, (uint8_t*) ReadMessage, 5);
	
// 	HAL_UART_Transmit(&huart1, (uint8_t*)Buffer, 5,0xFFFF);
	
// 	//HAL_UART_Receive_IT(&huart1, (uint8_t*) ReadMessage, 5);
	
	
// 	USART3_PINSET_TX();
// 	//myprintf("ReadMessage: %x | %x | %x | %x | %x  \r\n",ReadMessage[0], ReadMessage[1], ReadMessage[2], ReadMessage[3], ReadMessage[4]);
// 	myprintf("Init register configured");
// 	myprintf("\r\n");
// 	USART3_PINSET_RX();
	 
	
// 	return true;
// }



// USART3
// USART3_PINSET_TX();
			// 	if (aRxBuffer[0] == dsp_reg14){
			// 		myprintf("Reading: CH1_RMS\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", CH1_RMS[0], CH1_RMS[1], CH1_RMS[2], CH1_RMS[3], CH1_RMS[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg1){
			// 		myprintf("Reading: PH1_Active_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Active_Energy[0],PH1_Active_Energy[1],PH1_Active_Energy[2],PH1_Active_Energy[3],PH1_Active_Energy[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg2){
			// 		myprintf("Reading: PH1_Fundamental_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Fundamental_Energy[0], PH1_Fundamental_Energy[1], PH1_Fundamental_Energy[2], PH1_Fundamental_Energy[3], PH1_Fundamental_Energy[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg3){
			// 		myprintf("Reading: PH1_Reactive_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Reactive_Energy[0], PH1_Reactive_Energy[1], PH1_Reactive_Energy[2], PH1_Reactive_Energy[3], PH1_Reactive_Energy[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg4){
			// 		myprintf("Reading: PH1_Apparent_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Apparent_Energy[0], PH1_Apparent_Energy[1], PH1_Apparent_Energy[2], PH1_Apparent_Energy[3], PH1_Apparent_Energy[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg5){
			// 		myprintf("Reading: PH1_Active_Power\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Active_Power[0], PH1_Active_Power[1], PH1_Active_Power[2], PH1_Active_Power[3], PH1_Active_Power[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg6){
			// 		myprintf("Reading: PH1_Fundamental_Power\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Fundamental_Power[0], PH1_Fundamental_Power[1], PH1_Fundamental_Power[2], PH1_Fundamental_Power[3], PH1_Fundamental_Power[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg7){
			// 		myprintf("Reading: PH1_Reactive_Power\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Reactive_Power[0], PH1_Reactive_Power[1], PH1_Reactive_Power[2], PH1_Reactive_Power[3], PH1_Reactive_Power[4]);
			// 	} else if (aRxBuffer[0] == ph1_reg8){
			// 		myprintf("Reading: PH1_Apparent_RMS_Power\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", PH1_Apparent_RMS_Power[0], PH1_Apparent_RMS_Power[1], PH1_Apparent_RMS_Power[2], PH1_Apparent_RMS_Power[3], PH1_Apparent_RMS_Power[4]);
			// 	} else if (aRxBuffer[0] == tot_reg1){
			// 		myprintf("Reading: Total_Active_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", Total_Active_Energy[0], Total_Active_Energy[1], Total_Active_Energy[2], Total_Active_Energy[3], Total_Active_Energy[4]);
			// 	} else if (aRxBuffer[0] == tot_reg2){
			// 		myprintf("Reading: Total_Fundamental_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", Total_Fundamental_Energy[0], Total_Fundamental_Energy[1], Total_Fundamental_Energy[2], Total_Fundamental_Energy[3], Total_Fundamental_Energy[4]);
			// 	} else if (aRxBuffer[0] == tot_reg3){
			// 		myprintf("Reading: Total_Reactive_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", Total_Reactive_Energy[0], Total_Reactive_Energy[1], Total_Reactive_Energy[2], Total_Reactive_Energy[3], Total_Reactive_Energy[4]);
			// 	} else if (aRxBuffer[0] == tot_reg4){
			// 		myprintf("Reading: Total_Apparent_Energy\r\n");
			// 		myprintf("%x | %x | %x | %x | %x \r\n", Total_Apparent_Energy[0], Total_Apparent_Energy[1], Total_Apparent_Energy[2], Total_Apparent_Energy[3], Total_Apparent_Energy[4]);
			// 	} else {
			// 		myprintf(" Not a valid address \r\n");
			// 	}
			// USART3_PINSET_RX();



//USART3_PINSET_TX();
		 	//myprintf("Address : %x Data: %x | %x | %x | %x | %x \r\n\r\n", i[0], RxBuffer[0], RxBuffer[1], RxBuffer[2], RxBuffer[3], RxBuffer[4]);
		 	//USART3_PINSET_RX();

//External flash redundant code for different parameters
/*
int loading_parameters(char *params, bool check_new_value, uint32_t new_value)
{
	(void)(params);
	(void)(check_new_value);
	(void)(new_value);
//	INFO("Loading algo parameters...\n");
//	ext_flash_read(FLASH_PARAMETERS_ADDR, (char *)&params_algo, sizeof(params_algo));
	ext_flash_power_off();	//Need to be done manually to save power

	return 0;
}


int saving_parameters(char *params, bool check_new_value, uint32_t new_value)
{
	(void)(params);
	(void)(check_new_value);
	(void)(new_value);
//	INFO("Saving algo parameters...\n");
	ext_flash_erase_sector(FLASH_PARAMETERS_SECTOR);
//	ext_flash_write(FLASH_PARAMETERS_ADDR, (char *)&params_algo, sizeof(params_algo));
	ext_flash_power_off();	//Need to be done manually to save power

	return 0;
}


 * The purpose of this function is to set all parameters to their default values.
 * Default values will make in a default mode.
 * This mode will be overwritten by USB command to set the parameters.

void init_usecase_parameters(bool reset)
{
	loading_parameters(NULL, 0, 0);

	 * if the flash has no data, all the param's sector is set to FF
	 * then that means the user hasn't enter any parameters.
	 * So we set the product into a default mode.
	 *
//	if( (reset == true) || (params_algo.use_case == 0xFFFF) )
//	{
//		params_algo.use_case = 	ALGO_ALIVE + ALGO_CHOC;
//		params_algo.maxRange = 8;	//by default
//
//		params_algo.alive.mode = AUTO_REFRESH; 			//to read from config file
//		params_algo.alive.period = 60000; 				//ms, to read from config file
//		params_algo.alive.nb_temp_to_save = 1;			//by default, send directly the alive data
//		params_algo.alive.send_sensors_data = false;	//by default, do not send the sensors data
//
//		params_algo.choc.freq_mpu = 10;				//to read from config file
//		params_algo.choc.Gx_max_choc_pos = 5000;	//to read from config file
//		params_algo.choc.Gx_max_choc_neg = 5000;	//to read from config file
//		params_algo.choc.Gy_max_choc_pos = 5000;	//to read from config file
//		params_algo.choc.Gy_max_choc_neg = 5000;	//to read from config file
//		params_algo.choc.Gz_max_choc_pos = 5000;	//to read from config file
//		params_algo.choc.Gz_max_choc_neg = 5000;	//to read from config file
//		params_algo.choc.enable_inhibition = true;
//		params_algo.choc.inhibition = 150;			//ms, to read from config file
//		params_algo.choc.keep_gravity = true;
//
//		params_algo.movement.freq_mpu = 15;			//to read from config file
//		params_algo.movement.sensitivity = 10;		//to read from config file (nb of threshold excess during X)
//		params_algo.movement.count = 0;				//should be already set to 0
//		params_algo.movement.time_before_start = 2000;		//in ms, to read from config file
//		params_algo.movement.time_before_stop = 3000;		//in ms,to read from config file
//		params_algo.movement.Gx_max_mvt_pos = 300;			//to read from config file
//		params_algo.movement.Gx_max_mvt_neg = 300;			//to read from config file
//		params_algo.movement.Gy_max_mvt_pos = 300;			//to read from config file
//		params_algo.movement.Gy_max_mvt_neg = 300;			//to read from config file
//		params_algo.movement.Gz_max_mvt_pos = 300;			//to read from config file
//		params_algo.movement.Gz_max_mvt_neg = 300;			//to read from config file
//		params_algo.movement.product_is_moving = false;
//		params_algo.movement.send_trame_before = false;
//		params_algo.movement.send_trame_after =  false;
//		params_algo.movement.activity = true;
//		params_algo.movement.additionate_activity = true;
//		params_algo.movement.periodic_activity = true;
//		params_algo.movement.activity_resume_period = 1;	//in hour
//		params_algo.movement.send_resumed_activity = false;
//		params_algo.movement.timer_is_over = false;
//		params_algo.movement.activity_time = 0;
//		params_algo.movement.activity_ticks_beginning = 0;
//
//		params_algo.temperature.mode = AUTO_REFRESH + THRESHOLD;		//to read from config file
//		params_algo.temperature.delta_theta = 100;			//to read from config file
//		params_algo.temperature.nb_excess_threshold = 5; 	//to read from config file
//		params_algo.temperature.period_user = 30000;			//to read from config file
//		params_algo.temperature.period_fast = 5000;			//to read from config file
//		params_algo.temperature.period_ultrafast = 2000; 	//to read from config file
//		params_algo.temperature.temp_max = 2500;			//to read from config file
//		params_algo.temperature.temp_min = 2000;			//to read from config file
//		params_algo.temperature.nb_excess = 3;				//should be already set to 0
//		params_algo.temperature.nb_temp_to_save = 12;		//to read from config file
//
//		params_algo.tilt.mode = AUTO_REFRESH + THRESHOLD;	//to read from config file
//		params_algo.tilt.period = 10000;					//to read from config file
//		params_algo.tilt.threshold = 200;					//to read from config file
//		params_algo.tilt.pitch_alert_threshold = 10;		//in degree, to read from config file
//		params_algo.tilt.roll_alert_threshold = 10;			//in degree, to read from config file
//		params_algo.tilt.nb_excess_threshold = 5;			//to read from config file
//		params_algo.tilt.pitch_init = 0;					//should be already set to 0
//		params_algo.tilt.roll_init = 0;						//should be already set to 0
//		params_algo.tilt.init_pitch_roll = false;
//		params_algo.tilt.nb_excess = 0;						//should be already set to 0
//		params_algo.tilt.rotate_axes_around_y = TILT_NO_ROTATION;
//
//		params_algo.rotation.mode = AUTO_REFRESH + THRESHOLD;	//to read from config file
//		params_algo.rotation.period = 5000;					//in ms, to read from config file
//		params_algo.rotation.long_period = 15000;			//in ms, to read from config file
//		params_algo.rotation.threshold = 200;				//in mg, to read from config file
//		params_algo.rotation.nb_lap_before_sending = 5;
//		params_algo.rotation.quart_tour = 0;
//		params_algo.rotation.actual_zone = 0;
//		params_algo.rotation.previous_zone = 0;
//		params_algo.rotation.nb_tour = 0;
//		params_algo.rotation.nb_lap_reset_enabled = true;
//
//		params_algo.orientation.mode = AUTO_REFRESH + THRESHOLD;	//to read from config file
//		//FIXME:------------------------------
//		params_algo.orientation.Rx_alert = 90;			//to read from config file
//		params_algo.orientation.Ry_alert = -45;			//to read from config file
//		params_algo.orientation.Rz_alert = 90;			//to read from config file
//		//FIXME:------------------------------
//		params_algo.orientation.mesuration_length = 5;	//in s, to read from config file
//		params_algo.orientation.period = 30;			//in s, to read from config file
//		params_algo.orientation.threshold = 800;		//in mg, to read from config file
//		params_algo.orientation.initIsNeeded = false;
//		params_algo.orientation.stopRequested = false;
//
//		params_algo.sensor.mode = AUTO_REFRESH;		//to read from config file
//		params_algo.sensor.delta = 100;			//to read from config file
//		params_algo.sensor.nb_excess_threshold = 5; 	//to read from config file
//		params_algo.sensor.period_user = 30000;			//to read from config file
//		params_algo.sensor.period_fast = 5000;			//to read from config file
//		params_algo.sensor.period_ultrafast = 2000; 	//to read from config file
//		params_algo.sensor.water_level_max = 2500;			//to read from config file
//		params_algo.sensor.water_level_min = 2000;			//to read from config file
//		params_algo.sensor.water_pressure_max = 2500;			//to read from config file
//		params_algo.sensor.water_pressure_min = 2000;			//to read from config file
//		params_algo.sensor.nb_excess = 3;				//should be already set to 0
//	}
}



//==============================================================================

//==============================================================================
void vd_lib_flash_commands(char * s8p_Commande, int32_t s32_Param)
{
//    // Debug commands for FlashFS
//    static const s_def_DebugCmd s_lib_FlashCmd[] =
//    {
//        {"status",		&vd_lib_flash_status,		"                                       : Display status", 0u},
//        {"dump",		&vd_lib_flash_dump,			"<sector number>                        : Dump sector", 0u},
//        {"erase",		&vd_lib_flash_erase_sector,	"<sector number>                        : Erase sector", 0u},
//    };
//
//	vd_lib_Debug_ParseCmd(s_lib_FlashCmd, (sizeof(s_lib_FlashCmd)/sizeof(s_def_DebugCmd)), (uint8_t*)s8p_Commande);
}

//==============================================================================

//==============================================================================
//static void vd_lib_flash_status(char *s8p_Commande, int32_t s32_Param)
//{
//    UNUSED_PARAMETER(s8p_Commande);
//	INFO("Flash detected=%d; size=%lu bytes\n", ext_flash_is_detected(), ext_flash_Get_Size());
//}
//
////==============================================================================
//static void vd_lib_flash_dump(char *s8p_Commande, int32_t s32_Param)
//{
//	uint8_t u8_sector[EXT_FLASH_SECTOR_SIZE];
//	int32_t s32_noSector= 0;
//	uint32_t u32_addr= 0;
//	if ((s8p_Commande!=NULL) && (strlen(s8p_Commande)>0))
//	{
//		s32_noSector= (int32_t)atol(s8p_Commande);
//	}
//	u32_addr= s32_noSector*EXT_FLASH_SECTOR_SIZE;
//	INFO("Dumping sector #%ld, addr 0x%06x\n", s32_noSector, u32_addr);
//	memset(u8_sector, 0x55, sizeof(u8_sector));
//	ext_flash_read(u32_addr, (char*)u8_sector, EXT_FLASH_SECTOR_SIZE);
//#define	NB_BYTES_PER_LINE	32
//
//	for(uint32_t i=0;i<EXT_FLASH_SECTOR_SIZE;i+=NB_BYTES_PER_LINE)
//	{
//		INFO("0x%06x:", i);
//		for(uint32_t j=0;j<NB_BYTES_PER_LINE;j++)
//		{
//			INFO(" %02x", u8_sector[i+j]);
//		}
//		INFO("\n");
//	}
//}
//
////==============================================================================
//static void vd_lib_flash_erase_sector(char *s8p_Commande, int32_t s32_Param)
//{
//	if ((s8p_Commande!=NULL) && (strlen(s8p_Commande)>0))
//	{
//		int32_t s32_noSector= (int32_t)atol(s8p_Commande);
//		uint32_t u32_addr= s32_noSector*EXT_FLASH_SECTOR_SIZE;
//        INFO("Erasing sector #%ld, addr 0x%06x\n", s32_noSector, u32_addr);
//        ext_flash_erase_sector(u32_addr);
//	}
//}

*/


//Redundant HAL_LoRa functions
/*
int LoRa_SendData(uint8_t* buffer, uint8_t offset, uint8_t size){

	LoRa_SetOpMode(STANDBY);
	
	LoRa_WriteReg(RegHopPeriod, 0);		//Disable hopper period
	
	LoRa_WriteReg(RegDioMapping1, 0x40);
	LoRa_WriteReg(RegDioMapping2, 0x00);
	
	LoRa_WriteReg(RegIrqFlags, 0xFF);
	LoRa_WriteReg(RegIrqFlagsMask, 0x08);
	
	LoRa_WriteReg(RegPayloadLength, size);
	LoRa_WriteReg(RegFifoTxBaseAddr, 0);
	LoRa_WriteReg(RegFifoAddrPtr, 0);
	
	
	HAL_GPIO_WritePin(NRST_1278_GPIO_Port, NRST_1278_Pin, GPIO_PIN_RESET);
	uint8_t buff[1] = {0x80};
	if(HAL_SPI_Transmit(&hspi2, &buff[0], 1, 5) != HAL_OK){
		Error_Handler();
	}
		
	for (int i = 0; i < size; i++) {
		if(HAL_SPI_Transmit(&hspi2, &buffer[i+offset], 1, 5) != HAL_OK){
			Error_Handler();
		}
	}

	HAL_GPIO_WritePin(NRST_1278_GPIO_Port, NRST_1278_Pin, GPIO_PIN_SET);
	
	LoRa_SetOpMode(TX);
	
	int timeover = 1000;
	
	while (timeover-- > 0){
		if ((LoRa_ReadReg(RegIrqFlags) & 0x08) == 0x08){
			LoRa_SetReceiveMode(); 
			return 0;
		}
		
		HAL_Delay(3);
	}
	LoRa_SetReceiveMode(); 
	return -1;
}

void LoRa_SetReceiveMode(void){
	
	LoRa_SetOpMode(STANDBY);
	
	//LoRa_WriteReg(RegHopPeriod, 0);		??
	
	LoRa_WriteReg(RegDioMapping1, 0x00);
	LoRa_WriteReg(RegDioMapping2, 0x00);
	
	LoRa_WriteReg(RegIrqFlags, 0xFF);
	LoRa_WriteReg(RegIrqFlagsMask, 0x40);
	
	LoRa_SetOpMode(RXSINGLE);
	
}

**
* @brief Set LoRa chip Low data rate on/off
* @param Parameter: on/off
* @retval None
*
void LoRa_SetMobileNode(bool enable){
	uint8_t data;
	
	data = LoRa_ReadReg(RegModemConfig3);
	
	data &= 0xF7;
	
	data |= (enable ? 1 : 0) << 3;
	
	LoRa_WriteReg(RegModemConfig3, data);
	
}

**
* @brief Set LoRa chip operation time out
* @param Parameter: time out value
* @retval None
*
void LoRa_SetSymbTimeout(unsigned int value){
	uint8_t buffer[2];
	
	buffer[0] = LoRa_ReadReg(RegModemConfig2);
	
	buffer[1] = LoRa_ReadReg(RegSymbTimeoutLsb);
	
	buffer[0] &= 0xFC;
	buffer[0] |= value >> 8;
	
	buffer[1] = value & 0xFF;
	
	LoRa_WriteReg(RegModemConfig2,buffer[0]);
	LoRa_WriteReg(RegSymbTimeoutLsb, buffer[1]);
	
}

**
* @brief Set LoRa chip payload length
* @param Parameter: length of payload
* @retval None
*
void LoRa_SetPayloadLength(uint8_t value){
	LoRa_WriteReg(RegPayloadLength, value);
}

**
* @brief Set LoRa chip implicit header on/off
* @param Parameter: on/off
* @retval None
*
void LoRa_SetImplicitHeaderOn(bool enable){
	uint8_t data;
	
	data = LoRa_ReadReg(RegModemConfig1);
	
	data &= 0xFE;
	
	data |= (enable ? 1 : 0);
	
	LoRa_WriteReg(RegModemConfig1, data);
}

**
* @brief Set LoRa chip bandwidth
* @param Parameter: bandwidth
* @retval None
*
void LoRa_SetSignalBandwidth(uint8_t bw){
	uint8_t data;
	
	data = LoRa_ReadReg(RegModemConfig1);
	
	data &= 0x0F;
	
	data |= bw << 4;
	
	LoRa_WriteReg(RegModemConfig1, data);
	
}

**
* @brief Set LoRa chip crc on/off
* @param Parameter: enable/disable
* @retval None
*
void LoRa_SetPacketCrcOn(bool enable){
	uint8_t data;
	
	data = LoRa_ReadReg(RegModemConfig2);
	
	data &= 0xFB;
	
	data |= (enable ? 1:0) << 2;
	
	LoRa_WriteReg(RegModemConfig2, data);
	
}

**
* @brief Set LoRa chip error coding rate
* @param Parameter: error code rate
* @retval None
*
void LoRa_SetErrorCoding(uint8_t value){
	uint8_t data;
	
	data = LoRa_ReadReg(RegModemConfig1);
	
	data &= 0xF1;
	
	data |= value << 1;
	
	LoRa_WriteReg(RegModemConfig1, data);
	
}

**
* @brief Set LoRa chip spreading factor
* @param Parameter: spreading factor
* @retval None
*
void LoRa_SetSpreadingFactor(uint8_t factor){
	uint8_t data;
	
	data = LoRa_ReadReg(RegModemConfig2);
	
	data &= 0x0F;
	
	data |= factor<<4;
	
	LoRa_WriteReg(RegModemConfig2, data);
	
}

**
* @brief Set LoRa chip RF Power
* @param Parameter: power
* @retval None
*
void LoRa_SetRFPower(uint8_t power){
	//LoRa_WriteReg(RegPaConfig, 0x00);
	//LoRa_WriteReg(RegPaDac,0x04);
	
}

*/


/*============================================================================*/
/*                   END OF FILE                                              */
/*============================================================================*/
