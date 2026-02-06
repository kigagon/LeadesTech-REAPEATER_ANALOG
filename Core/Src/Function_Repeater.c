/*
 * Function_Repeater.c
 *
 *  Created on: Apr 13, 2025
 *      Author: root
 */


#include "main.h"
#include "Function_Repeater.h"
#include "function_Analog_ISO.h"
#include "flash.h"

uint8_t Rep_port_Charge_First_Mode[4];
uint8_t Rep_Out_Mode[4];
uint8_t Rep_ISO_In_Open;
float Rep_IN_B_ADC;

uint8_t Pre_IN_Short, Pre_OUT_Short;

#define Rep_Save_Num 20
uint8_t Rep_Save_Input_value[4][Rep_Save_Num];
uint8_t Rep_Save_Rep_port_open[4][Rep_Save_Num];
uint8_t Rep_Save_port_Fuse_Open[4][Rep_Save_Num];

int Rep_Sum_Input_value[4];
int Rep_Sum_port_open[4];
int Rep_Sum_Fuse_Open[4];


//Test mode
uint8_t IO_Tset_Mode = 0;
uint8_t Ch1_Tset_Mode = 0;

#define Ch1_Open_Cnt_Num	100
uint8_t Ch1_Open_Cnt[Ch1_Open_Cnt_Num];
int Ch1_Open_Cnt_Sum;
int Ch1_Open_int_Cnt_Sum;

uint8_t Ch1_Out_Open;
uint8_t Ch1_Out_Off_Cnt = 0;

#define Cnt_Num 30
uint16_t Ch1_Out_Cnt_Sum;
uint8_t Ch1_Out_Cnt[Cnt_Num];


void Run_Repeater_Mode(void){

	//감지기(중계기)는 릴레이를 비활성화 한다.
	if(ISO_MODE == ISO){
		Rep_Out_Relay_Mode = Relay_On;
		Repeater_ISO_OUT_Off_M_Set(2);
		HAL_Delay(100);
		Repeater_ISO_OUT_Off_M_Set(2);
	}

	Rep_Set_Out(Output_Port1,Output_Off);
	Rep_Set_Out(Output_Port2,Output_Off);
	Rep_Set_Out(Output_Port3,Output_Off);
	Rep_Set_Out(Output_Port4,Output_Off);

	Rep_TX_LED(LED_ON);
	Rep_ISO_LED(LED_ON);

	HAL_Delay(10);

	Rep_TX_LED(LED_OFF);
	Rep_ISO_LED(LED_OFF);

	HAL_GPIO_WritePin(a_CON_OUT_SET_r_DISCON1_GPIO_Port, a_CON_OUT_SET_r_DISCON1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(a_N_MODE_r_DISCON2_GPIO_Port, a_N_MODE_r_DISCON2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(a_SCL_r_DISCON3_GPIO_Port, a_SCL_r_DISCON3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(a_SDA_r_DISCON4_GPIO_Port, a_SDA_r_DISCON4_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(a_DIP_ADD7_r_IN1_SIG_GPIO_Port, a_DIP_ADD7_r_IN1_SIG_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(a_LED_BLUE_r_IN2_SIG_GPIO_Port, a_LED_BLUE_r_IN2_SIG_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(a_LED_RED_r_IN3_SIG_GPIO_Port, a_LED_RED_r_IN3_SIG_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(r_IN4_SIG_GPIO_Port, r_IN4_SIG_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(BOOT_R_N_MODE_GPIO_Port, BOOT_R_N_MODE_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, a_IN_A_ADC_r_CON_OUT_RSET_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(OPEN_SHORT_CON_GPIO_Port, OPEN_SHORT_CON_Pin, GPIO_PIN_SET);

	///////////////Start reading address of board//////////////////
	CCU_Address_tmp[0] =  ~(HAL_GPIO_ReadPin(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, a_IN_B_TEST_r_DIP_ADD0_Pin)) & 0x01;
	CCU_Address_tmp[1] =  ~(HAL_GPIO_ReadPin(a_MOSI_r_DIP_ADD1_GPIO_Port, a_MOSI_r_DIP_ADD1_Pin)) & 0x01;
	CCU_Address_tmp[2] =  ~(HAL_GPIO_ReadPin(a_MISO_DIP_ADD2_GPIO_Port, a_MISO_DIP_ADD2_Pin)) & 0x01;
	CCU_Address_tmp[3] =  ~(HAL_GPIO_ReadPin(a_SCK_r_DIP_ADD3_GPIO_Port, a_SCK_r_DIP_ADD3_Pin)) & 0x01;
	CCU_Address_tmp[4] =  ~(HAL_GPIO_ReadPin(a_SPI_CS_r_DIP_ADD4_GPIO_Port, a_SPI_CS_r_DIP_ADD4_Pin)) & 0x01;
	CCU_Address_tmp[5] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD2_r_DIP_ADD5_GPIO_Port, a_DIP_ADD2_r_DIP_ADD5_Pin)) & 0x01;
	CCU_Address_tmp[6] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD1_r_DIP_ADD6_GPIO_Port, a_DIP_ADD1_r_DIP_ADD6_Pin)) & 0x01;
	CCU_Address_tmp[7] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD0_r_DIP_ADD7_GPIO_Port, a_DIP_ADD0_r_DIP_ADD7_Pin)) & 0x01;

	CCU_Address = (CCU_Address_tmp[7] << 7)|(CCU_Address_tmp[6] << 6)|(CCU_Address_tmp[5] << 5)|(CCU_Address_tmp[4] << 4)
			  |(CCU_Address_tmp[3] << 3)|(CCU_Address_tmp[2] << 2)|(CCU_Address_tmp[1] << 1)|(CCU_Address_tmp[0] << 0);

	  ///////////////End reading address of board//////////////////
//
//	Rep_Output_value[0] = 1;
//	Rep_Output_value[1] = 1;
//	Rep_Output_value[2] = 1;
//	Rep_Output_value[3] = 1;
//	Rep_Set_Out(Output_Port1,Output_On);
//	Rep_Set_Out(Output_Port2,Output_On );
//	Rep_Set_Out(Output_Port3,Output_On );
//	Rep_Set_Out(Output_Port4,Output_On );

	for(int i = 0 ; i<4 ; i++){
		Rep_port_Charge_Setting[i] = Charge_Set_Off;
		Rep_port_Charge_Mode[i] = Charge_Mode_Off;
//		Rep_port_Charge_Mode[i] = Charge_Mode_On;
//		Rep_port_Charge_Mode[i] = Rep_port_Charge_Setting[i];
		Rep_port_Charge_First_Mode[i] = Charge_Mode_First_Ready;
	}

 Rep_Out_ISO_Pre_Mode = ISO_Normal;
 Rep_Out_ISO_Mode = ISO_Normal;
 Rep_ISO_In_Short = ISO_Normal;

 // 주소값이 0 이면 릴레이 테스트 동작을 한다.
	if(CCU_Address == 0){
	 if(ISO_MODE == ISO){
		while(1){
			HAL_Delay(100);
			Repeater_ISO_OUT_Off_M_Set(5);
			HAL_Delay(1000);
			Repeater_ISO_OUT_On_Set();
		}
	 }
	}

	if(ISO_MODE == ISO){
		//1. 콘덴서 충전을 기다린다.
		HAL_Delay(500);

		//2.이전 저장된 동작 을 확인 하여 릴레이를 동작한다.
		//Flash_Read_uint8(ISO_STATUS_ADDR, &Pre_IN_Short);
		Flash_Read_uint8(ISO_STATUS_ADDR + 8, &Pre_OUT_Short);
		//Flash_Write_uint8(ISO_STATUS_ADDR, 0);
		//Flash_Write_uint8(ISO_STATUS_ADDR + 8, 0);


		if(Pre_OUT_Short == 0xff){
			Pre_OUT_Short = 0;
			Flash_Erase(ISO_STATUS_ADDR);
			Flash_Write_uint8(ISO_STATUS_ADDR + 8, Pre_OUT_Short);
		}

		if(Pre_OUT_Short == 1){
			Repeater_ISO_OUT_Off_M_Set(5);
			Repeater_ISO_OUT_Off_M_Set(5);
			Pre_OUT_Short = 0;
			Rep_ISO_St = ISO_Error;
			Flash_Erase(ISO_STATUS_ADDR);
			Flash_Write_uint8(ISO_STATUS_ADDR + 8, Pre_OUT_Short);
		}
		else if(Pre_OUT_Short == 0){

			if(CCU_Address == 220){
				Rep_SIG24_ADC = Repeater_Rep_SIG24_ADC_Avr(5, 10);
				Rep_OUT_ADC = Repeater_Rep_OUT_ADC_Avr(5, 10);

				if(Rep_OUT_ADC < Comp_24V){
					Repeater_ISO_OUT_Off_M_Set(5);
				}
				else {
					if(Rep_SIG24_ADC < Comp_24V){
						Repeater_ISO_OUT_On_M_Set(5);
						HAL_Delay(100);
						Rep_SIG24_ADC = Repeater_Rep_SIG24_ADC_Avr(5, 10);
						 if(Rep_SIG24_ADC < Comp_24V){
							 Repeater_ISO_OUT_Off_M_Set(5);
							 Repeater_ISO_OUT_Off_M_Set(5);
							 Pre_OUT_Short = 1;
							 Flash_Erase(ISO_STATUS_ADDR);
							 Flash_Write_uint8(ISO_STATUS_ADDR + 8, Pre_OUT_Short);
						 }
					}
					else{
						Repeater_ISO_OUT_Off_M_Set(5);
					}
				}

			}
			else{
				Repeater_ISO_OUT_On_M_Set(5);
				HAL_Delay(100);
				Rep_SIG24_ADC = Repeater_Rep_SIG24_ADC_Avr(5, 10);
				 if(Rep_SIG24_ADC < Comp_24V){
					 Repeater_ISO_OUT_Off_M_Set(5);
					 Repeater_ISO_OUT_Off_M_Set(5);
					 Pre_OUT_Short = 1;
					 Flash_Erase(ISO_STATUS_ADDR);
					 Flash_Write_uint8(ISO_STATUS_ADDR + 8, Pre_OUT_Short);
				 }
			}
		}
	}

	while(1){

	  __HAL_UART_ENABLE_IT(&hlpuart2, UART_IT_ERR);
	  /* Enable the UART Data Register not empty Interrupt */
	  __HAL_UART_ENABLE_IT(&hlpuart2, UART_IT_RXNE);

		Check_Com();

		Rep_Read_Port();

		Check_Com();
		Check_Charge_Mode();
		Check_Com();
		Set_Out_Port();
		Check_Com();
		Check_Fuse_Open();
		Check_Com();

		if(Ch1_Tset_Mode == 1){
			Check_Ch1();
			HAL_Delay(1000);
		}
		if(ISO_MODE == ISO){
			if((Rep_ISO_St == ISO_Normal)|(Rep_ISO_St == ISO_Open)){
				Rep_Check_ISO();
			}
		}
		if(Rep_ISO_St == ISO_Error){
			Rep_ISO_LED(LED_ON);
		}
		else{
			Rep_ISO_LED(LED_OFF);
		}

	}
}



void Rep_TX_LED(uint8_t OnOff){
	if(OnOff == LED_ON){
		HAL_GPIO_WritePin(a_IN_A_TEST_r_LED_TXD_GPIO_Port, a_IN_A_TEST_r_LED_TXD_Pin, GPIO_PIN_RESET);
	}
	else if(OnOff == LED_OFF){
		HAL_GPIO_WritePin(a_IN_A_TEST_r_LED_TXD_GPIO_Port, a_IN_A_TEST_r_LED_TXD_Pin, GPIO_PIN_SET);
	}
}

void Rep_ISO_LED(uint8_t OnOff){
	if(OnOff == LED_ON){
		HAL_GPIO_WritePin(a_CON_IN_SET_r_LED_ISOERR_GPIO_Port, a_CON_IN_SET_r_LED_ISOERR_Pin, GPIO_PIN_RESET);
	}
	else if(OnOff == LED_OFF){
		HAL_GPIO_WritePin(a_CON_IN_SET_r_LED_ISOERR_GPIO_Port, a_CON_IN_SET_r_LED_ISOERR_Pin, GPIO_PIN_SET);
	}
}


void Rep_Set_Out(int Port , int Status){

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if(Port == Output_Port1){
		if(Status == Output_Off){
			HAL_GPIO_WritePin(a_OUT_ADC_r_OUT1_GPIO_Port,a_OUT_ADC_r_OUT1_Pin,GPIO_PIN_SET);
		}
		else if(Status == Output_On){
			if(Rep_Out_Mode[0] == Init_Set){
				Rep_Out_Mode[0] = Out_Set;

				GPIO_InitStruct.Pin = a_OUT_ADC_r_OUT1_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				HAL_GPIO_Init(a_OUT_ADC_r_OUT1_GPIO_Port, &GPIO_InitStruct);

			}
			HAL_GPIO_WritePin(a_OUT_ADC_r_OUT1_GPIO_Port,a_OUT_ADC_r_OUT1_Pin,GPIO_PIN_RESET);
		}
	}
	else if(Port == Output_Port2){
		if(Status == Output_Off){
			HAL_GPIO_WritePin(a_IR_LED_DAC_r_OUT2_GPIO_Port,a_IR_LED_DAC_r_OUT2_Pin,GPIO_PIN_SET);
		}
		else if(Status == Output_On){
			if(Rep_Out_Mode[1] == Init_Set){
				Rep_Out_Mode[1] = Out_Set;

				GPIO_InitStruct.Pin = a_IR_LED_DAC_r_OUT2_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				HAL_GPIO_Init(a_IR_LED_DAC_r_OUT2_GPIO_Port, &GPIO_InitStruct);
			}
			HAL_GPIO_WritePin(a_IR_LED_DAC_r_OUT2_GPIO_Port,a_IR_LED_DAC_r_OUT2_Pin,GPIO_PIN_RESET);
		}
	}
	else if(Port == Output_Port3){
		if(Status == Output_Off){
			HAL_GPIO_WritePin(a_op1_vout_r_OUT3_GPIO_Port,a_op1_vout_r_OUT3_Pin,GPIO_PIN_SET);
		}
		else if(Status == Output_On){
			if(Rep_Out_Mode[2] == Init_Set){
				Rep_Out_Mode[2] = Out_Set;

				GPIO_InitStruct.Pin = a_op1_vout_r_OUT3_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				HAL_GPIO_Init(a_op1_vout_r_OUT3_GPIO_Port, &GPIO_InitStruct);
			}
			HAL_GPIO_WritePin(a_op1_vout_r_OUT3_GPIO_Port,a_op1_vout_r_OUT3_Pin,GPIO_PIN_RESET);
		}
	}
	else if(Port == Output_Port4){
		if(Status == Output_Off){
			HAL_GPIO_WritePin(a_OUT_A_ADC_r_OUT4_GPIO_Port,a_OUT_A_ADC_r_OUT4_Pin,GPIO_PIN_SET);
		}
		else if(Status == Output_On){
			if(Rep_Out_Mode[3] == Init_Set){
				Rep_Out_Mode[3] = Out_Set;

				GPIO_InitStruct.Pin = a_OUT_A_ADC_r_OUT4_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				HAL_GPIO_Init(a_OUT_A_ADC_r_OUT4_GPIO_Port, &GPIO_InitStruct);
			}
			HAL_GPIO_WritePin(a_OUT_A_ADC_r_OUT4_GPIO_Port,a_OUT_A_ADC_r_OUT4_Pin,GPIO_PIN_RESET);
		}
	}
}


void Rep_Set_In(int Port , int Status){

	if(Port == Input_Port1){
		if(Status == Input_Off){
			HAL_GPIO_WritePin(a_DIP_ADD6_r_CH1_SW_GPIO_Port,a_DIP_ADD6_r_CH1_SW_Pin,GPIO_PIN_SET);
		}
		else if(Status == Input_On){
			HAL_GPIO_WritePin(a_DIP_ADD6_r_CH1_SW_GPIO_Port,a_DIP_ADD6_r_CH1_SW_Pin,GPIO_PIN_RESET);
		}
	}
	else if(Port == Input_Port2){
		if(Status == Input_Off){
			HAL_GPIO_WritePin(a_DIP_ADD5_r_CH2_SW_GPIO_Port,a_DIP_ADD5_r_CH2_SW_Pin,GPIO_PIN_SET);
		}
		else if(Status == Input_On){
			HAL_GPIO_WritePin(a_DIP_ADD5_r_CH2_SW_GPIO_Port,a_DIP_ADD5_r_CH2_SW_Pin,GPIO_PIN_RESET);
		}
	}
	else if(Port == Input_Port3){
		if(Status == Input_Off){
			HAL_GPIO_WritePin(a_DIP_ADD4_r_CH3_SW_GPIO_Port,a_DIP_ADD4_r_CH3_SW_Pin,GPIO_PIN_SET);
		}
		else if(Status == Input_On){
			HAL_GPIO_WritePin(a_DIP_ADD4_r_CH3_SW_GPIO_Port,a_DIP_ADD4_r_CH3_SW_Pin,GPIO_PIN_RESET);
		}
	}
	else if(Port == Input_Port4){
		if(Status == Input_Off){
			HAL_GPIO_WritePin(a_DIP_ADD3_r_CH4_SW_GPIO_Port,a_DIP_ADD3_r_CH4_SW_Pin,GPIO_PIN_SET);
		}
		else if(Status == Input_On){
			HAL_GPIO_WritePin(a_DIP_ADD3_r_CH4_SW_GPIO_Port,a_DIP_ADD3_r_CH4_SW_Pin,GPIO_PIN_RESET);
		}
	}
}

void Check_Charge_Mode(void){

	for(int i =0; i<4 ; ++i){
		if(Rep_Input_value[i] == 1){
			if((Rep_Pre_Input_value[i] == 0)&(Rep_port_Charge_Mode[i] == Charge_Mode_On)){
				if(Rep_port_Charge_First_Mode[i] == Charge_Mode_First_Ready){
					Rep_port_Charge_First_Mode[i] = Charge_Mode_First_On; //Timer Count On
				}
			}
		}
		else{
			if((Rep_Pre_Input_value[i] == 0)&(Rep_port_Charge_Mode[i] == Charge_Mode_On)){
				if(Rep_port_Charge_First_Mode[i] == Charge_Mode_First_On){

				}
				else if(Rep_port_Charge_First_Mode[i] == Charge_Mode_First_Off){
					Rep_port_Charge_First_Mode[i] = Charge_Mode_First_Ready;
				}
			}
		}

	}

}

void Check_Fuse_Open(void){

	for(int i=0;i<4;i++){
		if( Rep_Output_value[i] == 1 ){
			if(Rep_port_Fuse_Open_Mode[i] == Fuse_Short){
				if(Rep_port_Fuse_Open[i] == 1){
					Rep_Set_Out(i , Output_Off);
					Rep_port_Fuse_Open_Mode[i] = Fuse_Open;
					Rep_port_Fuse_Open_Cnt[i] = 0;
				}
			}
			else if(Rep_port_Fuse_Open_Mode[i] == Fuse_Open){

				if(Rep_port_Fuse_Open_Cnt[i]> Rep_port_Fuse_Open_Cnt_Max){
					Rep_Set_Out(i , Output_On);
					HAL_Delay(1);
					if(Rep_port_Fuse_Open[i] == 1){
						Rep_Set_Out(i , Output_Off);
						Rep_port_Fuse_Open_Mode[i] = Fuse_Open;
						Rep_port_Fuse_Open_Cnt[i] = 0;
					}
					else if(Rep_port_Fuse_Open[i] == 0){
						Rep_port_Fuse_Open_Mode[i] = Fuse_Short;
					}
				}
				//Rep_port_Fuse_Open_Cnt[i]++;
			}
		}
	}
}

void Rep_Read_Port(void){

	for(int i =0; i<4 ; i++){
			Rep_Pre_Input_value[i] = Rep_Input_value[i];
		}

	if(HAL_GPIO_ReadPin(a_OPAMP_IN_M_r_DC_24V_GPIO_Port, a_OPAMP_IN_M_r_DC_24V_Pin) == GPIO_PIN_RESET){
		Rep_V24_value = 1;
	}
	else{
		Rep_V24_value = 0;
	}

	for(int i=0; i<(Rep_Save_Num-1) ; i++){
		for(int j=0; j<4 ; j++){
			Rep_Save_Input_value[j][i] = Rep_Save_Input_value[j][i+1];
			Rep_Save_Rep_port_open[j][i] = Rep_Save_Rep_port_open[j][i+1];
			Rep_Save_port_Fuse_Open[j][i] = Rep_Save_port_Fuse_Open[j][i+1];
		}
	}

	if(Rep_V24_value == 1){

		if(HAL_GPIO_ReadPin(a_DIP_ADD7_r_IN1_SIG_GPIO_Port, a_DIP_ADD7_r_IN1_SIG_Pin) == GPIO_PIN_RESET){
			Rep_Save_Input_value[0][(Rep_Save_Num-1)] = 1;
		}
		else{
			Rep_Save_Input_value[0][(Rep_Save_Num-1)] =  0;
		}

		if(HAL_GPIO_ReadPin(a_LED_BLUE_r_IN2_SIG_GPIO_Port, a_LED_BLUE_r_IN2_SIG_Pin) == GPIO_PIN_RESET){
			Rep_Save_Input_value[1][(Rep_Save_Num-1)] = 1;
		}
		else{
			Rep_Save_Input_value[1][(Rep_Save_Num-1)] = 0;
		}

		if(HAL_GPIO_ReadPin(a_LED_RED_r_IN3_SIG_GPIO_Port, a_LED_RED_r_IN3_SIG_Pin) == GPIO_PIN_RESET){
			Rep_Save_Input_value[2][(Rep_Save_Num-1)] = 1 ;
		}
		else{
			Rep_Save_Input_value[2][(Rep_Save_Num-1)] = 0 ;
		}

		if(HAL_GPIO_ReadPin(r_IN4_SIG_GPIO_Port, r_IN4_SIG_Pin ) == GPIO_PIN_RESET){
			Rep_Save_Input_value[3][(Rep_Save_Num-1)] = 1 ;
		}
		else{
			Rep_Save_Input_value[3][(Rep_Save_Num-1)] = 0 ;
		}

		if(HAL_GPIO_ReadPin(a_CON_OUT_SET_r_DISCON1_GPIO_Port, a_CON_OUT_SET_r_DISCON1_Pin) == GPIO_PIN_RESET){
			Rep_Save_Rep_port_open[0][(Rep_Save_Num-1)] = 1;
		}
		else{
			Rep_Save_Rep_port_open[0][(Rep_Save_Num-1)] = 0;
		}

		if(HAL_GPIO_ReadPin(a_N_MODE_r_DISCON2_GPIO_Port, a_N_MODE_r_DISCON2_Pin) == GPIO_PIN_RESET){
			Rep_Save_Rep_port_open[1][(Rep_Save_Num-1)] = 1;
		}
		else{
			Rep_Save_Rep_port_open[1][(Rep_Save_Num-1)]= 0;
		}

		if(HAL_GPIO_ReadPin(a_SCL_r_DISCON3_GPIO_Port, a_SCL_r_DISCON3_Pin) == GPIO_PIN_RESET){
			Rep_Save_Rep_port_open[2][(Rep_Save_Num-1)] = 1;
		}
		else{
			Rep_Save_Rep_port_open[2][(Rep_Save_Num-1)] = 0;
		}

		if(HAL_GPIO_ReadPin(a_SDA_r_DISCON4_GPIO_Port, a_SDA_r_DISCON4_Pin) == GPIO_PIN_RESET){
			Rep_Save_Rep_port_open[3][(Rep_Save_Num-1)] = 1;
		}
		else{
			Rep_Save_Rep_port_open[3][(Rep_Save_Num-1)] = 0;
		}
		if(HAL_GPIO_ReadPin(a_CON_OUT_RSET_r_OCP1_GPIO_Port, a_CON_OUT_RSET_r_OCP1_Pin) == GPIO_PIN_RESET){
			Rep_Save_port_Fuse_Open[0][(Rep_Save_Num-1)] = 0;
		}
		else{
			Rep_Save_port_Fuse_Open[0][(Rep_Save_Num-1)] = 1;
		}

		if(HAL_GPIO_ReadPin(r_OCP2_GPIO_Port, r_OCP2_Pin) == GPIO_PIN_RESET){
			Rep_Save_port_Fuse_Open[1][(Rep_Save_Num-1)] = 0;
		}
		else{
			Rep_Save_port_Fuse_Open[1][(Rep_Save_Num-1)] = 1;
		}

		if(HAL_GPIO_ReadPin(a_IN_ADC_r_OCP3_GPIO_Port, a_IN_ADC_r_OCP3_Pin) == GPIO_PIN_RESET){
			Rep_Save_port_Fuse_Open[2][(Rep_Save_Num-1)] = 0;
		}
		else{
			Rep_Save_port_Fuse_Open[2][(Rep_Save_Num-1)] = 1;
		}

		if(HAL_GPIO_ReadPin(a_IN_B_ADC_r_OCP4_GPIO_Port, a_IN_B_ADC_r_OCP4_Pin) == GPIO_PIN_RESET){
			Rep_Save_port_Fuse_Open[3][(Rep_Save_Num-1)] = 0;
		}
		else{
			Rep_Save_port_Fuse_Open[3][(Rep_Save_Num-1)] = 1;
		}

		for(int j=0; j<4 ; j++){
			Rep_Sum_Input_value[j] = 0;
			Rep_Sum_port_open[j] = 0;
			Rep_Sum_Fuse_Open[j] = 0;
		}
		for(int i=0; i<4 ; i++){
			for(int j=0; j<Rep_Save_Num ; j++){
				Rep_Sum_Input_value[i] += Rep_Save_Input_value[i][j];
				Rep_Sum_port_open[i] += Rep_Save_Rep_port_open[i][j];
				Rep_Sum_Fuse_Open[i] += Rep_Save_port_Fuse_Open[i][j];
			}
		}
		for(int i=0; i<4 ; i++){
			if(Rep_Sum_Input_value[i] > (Rep_Save_Num / 2)){
				Rep_Input_value[i] = 1;
			}
			else{
				Rep_Input_value[i] = 0;
			}

			if(Rep_Sum_port_open[i] > (Rep_Save_Num / 2)){
				Rep_port_open[i] = 1;
			}
			else{
				Rep_port_open[i] = 0;
			}

			if(Rep_Sum_Fuse_Open[i] > (Rep_Save_Num / 2)){
				Rep_port_Fuse_Open[i] = 1;
			}
			else{
				Rep_port_Fuse_Open[i] = 0;
			}
		}
	}

}
/*
void Rep_Read_Port(void){

	for(int i =0; i<4 ; i++){
			Rep_Pre_Input_value[i] = Rep_Input_value[i];
		}

	if(HAL_GPIO_ReadPin(a_OPAMP_IN_M_r_DC_24V_GPIO_Port, a_OPAMP_IN_M_r_DC_24V_Pin) == GPIO_PIN_RESET){
		Rep_V24_value = 1;
	}
	else{
		Rep_V24_value = 0;
	}

	if(Rep_V24_value == 1){

		if(HAL_GPIO_ReadPin(a_DIP_ADD7_r_IN1_SIG_GPIO_Port, a_DIP_ADD7_r_IN1_SIG_Pin) == GPIO_PIN_RESET){
			Rep_Input_value[0] = 1;
		}
		else{
			Rep_Input_value[0] = 0;
		}

		if(HAL_GPIO_ReadPin(a_LED_BLUE_r_IN2_SIG_GPIO_Port, a_LED_BLUE_r_IN2_SIG_Pin) == GPIO_PIN_RESET){
			Rep_Input_value[1] = 1;
		}
		else{
			Rep_Input_value[1] = 0;
		}

		if(HAL_GPIO_ReadPin(a_LED_RED_r_IN3_SIG_GPIO_Port, a_LED_RED_r_IN3_SIG_Pin) == GPIO_PIN_RESET){
			Rep_Input_value[2] = 1;
		}
		else{
			Rep_Input_value[2] = 0;
		}

		if(HAL_GPIO_ReadPin(r_IN4_SIG_GPIO_Port, r_IN4_SIG_Pin ) == GPIO_PIN_RESET){
			Rep_Input_value[3] = 1;
		}
		else{
			Rep_Input_value[3] = 0;
		}

		if(HAL_GPIO_ReadPin(a_CON_OUT_SET_r_DISCON1_GPIO_Port, a_CON_OUT_SET_r_DISCON1_Pin) == GPIO_PIN_RESET){
			Rep_port_open[0] = 1;
		}
		else{
			Rep_port_open[0] = 0;
		}

		if(HAL_GPIO_ReadPin(a_N_MODE_r_DISCON2_GPIO_Port, a_N_MODE_r_DISCON2_Pin) == GPIO_PIN_RESET){
			Rep_port_open[1] = 1;
		}
		else{
			Rep_port_open[1] = 0;
		}

		if(HAL_GPIO_ReadPin(a_SCL_r_DISCON3_GPIO_Port, a_SCL_r_DISCON3_Pin) == GPIO_PIN_RESET){
			Rep_port_open[2] = 1;
		}
		else{
			Rep_port_open[2] = 0;
		}

		if(HAL_GPIO_ReadPin(a_SDA_r_DISCON4_GPIO_Port, a_SDA_r_DISCON4_Pin) == GPIO_PIN_RESET){
			Rep_port_open[3] = 1;
		}
		else{
			Rep_port_open[3] = 0;
		}
		if(HAL_GPIO_ReadPin(a_CON_OUT_RSET_r_OCP1_GPIO_Port, a_CON_OUT_RSET_r_OCP1_Pin) == GPIO_PIN_RESET){
			Rep_port_Fuse_Open[0] = 0;
		}
		else{
			Rep_port_Fuse_Open[0] = 1;
		}

		if(HAL_GPIO_ReadPin(r_OCP2_GPIO_Port, r_OCP2_Pin) == GPIO_PIN_RESET){
			Rep_port_Fuse_Open[1] = 0;
		}
		else{
			Rep_port_Fuse_Open[1] = 1;
		}

		if(HAL_GPIO_ReadPin(a_IN_ADC_r_OCP3_GPIO_Port, a_IN_ADC_r_OCP3_Pin) == GPIO_PIN_RESET){
			Rep_port_Fuse_Open[2] = 0;
		}
		else{
			Rep_port_Fuse_Open[2] = 1;
		}

		if(HAL_GPIO_ReadPin(a_IN_B_ADC_r_OCP4_GPIO_Port, a_IN_B_ADC_r_OCP4_Pin) == GPIO_PIN_RESET){
			Rep_port_Fuse_Open[3] = 0;
		}
		else{
			Rep_port_Fuse_Open[3] = 1;
		}
	}
}
*/

void Set_Out_Port(void){

	if(IO_Tset_Mode == 1){
		for(int i=0 ; i<4 ; i++){
			Rep_Output_value[i] = Rep_Input_value[i];
		}
	}
	if(Rep_Output_value[0] == 1 ){
		if(Rep_port_Fuse_Open_Mode[0] == Fuse_Short){
			Rep_Set_Out(Output_Port1,Output_On);
		}
	}
	else if(Rep_Output_value[0] == 0 ){
		Rep_Set_Out(Output_Port1,Output_Off);
	}

	if(Rep_Output_value[1] == 1 ){
		if(Rep_port_Fuse_Open_Mode[1] == Fuse_Short){
			Rep_Set_Out(Output_Port2,Output_On);
		}
	}
	else if(Rep_Output_value[1] == 0 ){
		Rep_Set_Out(Output_Port2,Output_Off);
	}

	if(Rep_Output_value[2] == 1 ){
		if(Rep_port_Fuse_Open_Mode[2] == Fuse_Short){
			Rep_Set_Out(Output_Port3,Output_On);
		}
	}
	else if(Rep_Output_value[2] == 0 ){
		Rep_Set_Out(Output_Port3,Output_Off);
	}

	if(Rep_Output_value[3] == 1 ){
		if(Rep_port_Fuse_Open_Mode[3] == Fuse_Short){
			Rep_Set_Out(Output_Port4,Output_On);
		}
	}
	else if(Rep_Output_value[3] == 0 ){
		Rep_Set_Out(Output_Port4,Output_Off);
	}

}

void Repeater_ISO_OUT_On_Set(void){

	if(Rep_Out_Relay_Mode == Relay_On){

	}
	else{
		Rep_Out_Relay_Mode = Relay_On;
		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, a_IN_A_ADC_r_CON_OUT_RSET_Pin, GPIO_PIN_RESET);

		HAL_Delay(5);

		HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, a_IN_A_ADC_r_CON_OUT_RSET_Pin, GPIO_PIN_RESET);
	}
}


void Repeater_ISO_OUT_Off_Set(void){

	if(Rep_Out_Relay_Mode == Relay_Off){

	}
	else {
		Rep_Out_Relay_Mode = Relay_Off;
		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, a_IN_A_ADC_r_CON_OUT_RSET_Pin, GPIO_PIN_SET);

		HAL_Delay(5);

		HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, a_IN_A_ADC_r_CON_OUT_RSET_Pin, GPIO_PIN_RESET);
	}
}

void Repeater_ISO_OUT_On_M_Set(int mset){

	if(Rep_Out_Relay_Mode == Relay_On){

	}
	else{
		Rep_Out_Relay_Mode = Relay_On;
		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, a_IN_A_ADC_r_CON_OUT_RSET_Pin, GPIO_PIN_RESET);

		HAL_Delay(mset);

		HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, a_IN_A_ADC_r_CON_OUT_RSET_Pin, GPIO_PIN_RESET);
	}
}

void Repeater_ISO_OUT_Off_M_Set(int mset){

	if(Rep_Out_Relay_Mode == Relay_Off){

	}
	else {
		Rep_Out_Relay_Mode = Relay_Off;
		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, a_IN_A_ADC_r_CON_OUT_RSET_Pin, GPIO_PIN_SET);

		HAL_Delay(mset);

		HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, a_IN_A_ADC_r_CON_OUT_RSET_Pin, GPIO_PIN_RESET);
	}
}

float Repeater_Rep_IN_B_ADC(void){

	float Rep_ADC;

	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	Rep_ADC = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);

	return Rep_ADC;
}

float Repeater_Rep_OUT_B_ADC(void){

	float Rep_ADC;

	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	Rep_ADC = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);

	return Rep_ADC;
}

float Repeater_Rep_SIG24_ADC(void){

	float Rep_ADC;

	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	Rep_ADC = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);

	return Rep_ADC;
}

float Repeater_Rep_SIG24_ADC_Avr(int Num, int Hold_Time){

	float ADC_Tmp[Num], Return_val;

	for(int i=0; i<Num ; i++){
		ADC_Tmp[i] = Repeater_Rep_SIG24_ADC();
		HAL_Delay(Hold_Time);
		//Check_Com();
	}
	Return_val = 0;
	for(int i=0; i<Num ; i++){
		Return_val = Return_val + ADC_Tmp[i];
	}

	Return_val = Return_val / Num;

	return Return_val;

}

float Repeater_Rep_OUT_ADC(void){

	float Rep_ADC;

	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	Rep_ADC= HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);

	return Rep_ADC;
}

float Repeater_Rep_OUT_ADC_Avr(int Num, int Hold_Time){

	float ADC_Tmp[Num], Return_val;

	for(int i=0; i<Num ; i++){
		ADC_Tmp[i] = Repeater_Rep_OUT_ADC();
		HAL_Delay(Hold_Time);
		//Check_Com();
	}
	Return_val = 0;
	for(int i=0; i<Num ; i++){
		Return_val = Return_val + ADC_Tmp[i];
	}

	Return_val = Return_val / Num;

	return Return_val;

}


void Rep_Check_ISO(void){

	// 1.1 전압을 체크한다.
	Rep_SIG24_ADC = Repeater_Rep_SIG24_ADC_Avr(1, 1);

	// 2.1 	전압이 떨어지면 릴레이를 오프한후 50ms 대기후전압을 다시 체크 한다
	if(Rep_SIG24_ADC < Comp_24V ){
		HAL_Delay(50);

		Rep_SIG24_ADC = Repeater_Rep_SIG24_ADC_Avr(10, 2);
		// 2.2 1초 후에도 전압이 복구 되면 정상	으로 확인
		if(Rep_SIG24_ADC >= Comp_24V ){
			Repeater_ISO_OUT_On_M_Set(5);
		}
		// 2.3 1초 후에도 전압이 복구 안되면 단락으로 확인
		else{
			Repeater_ISO_OUT_Off_M_Set(5);
			Rep_ISO_In_Open = ISO_Normal;
			Rep_ISO_In_Short = ISO_Normal;
			Rep_ISO_Out_Open = ISO_Normal;
			Rep_ISO_Out_Short = ISO_Error;
			Rep_ISO_St = ISO_Error;
		}
	}
	Check_Com();
}

uint8_t Rep_Check_Out_Short(void){

	// Check_Out_error 0 : ISO_Normal , Check_Out_error 1 : ISO_Normal , Check_Out_error 2 : ISO_Normal ,


	return Rep_ISO_St;
}

float Rep_Check_Out_Short_Avr(int Num, int Hold_Time){

	float ADC_Tmp_Off[Num], ADC_Tmp_On[Num], Return_val, Return_val_Off, Return_val_On;

	for(int i=0; i<Num ; i++){

		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

		HAL_Delay(Hold_Time);
		ADC_Tmp_Off[i] = Repeater_Rep_OUT_B_ADC();

		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

		HAL_Delay(Hold_Time);

		ADC_Tmp_On[i] = Repeater_Rep_OUT_B_ADC();

		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

	}

	Return_val = 0;
	Return_val_Off = 0;
	Return_val_On = 0;

	for(int i=0; i<Num ; i++){
		Return_val_Off = Return_val_Off + ADC_Tmp_Off[i];
		Return_val_On = Return_val_On + ADC_Tmp_On[i];
	}

	Return_val = Return_val_On - Return_val_Off;
//	Return_val = Return_val / Num;

	return Return_val;

}


float Rep_Check_In_Short_Avr(int Num, int Hold_Time){

	float ADC_Tmp_Off[Num], ADC_Tmp_On[Num], Return_val, Return_val_Off, Return_val_On;

	for(int i=0; i<Num ; i++){

		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

		HAL_Delay(Hold_Time);
		ADC_Tmp_Off[i] = Repeater_Rep_IN_B_ADC();

		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_RESET);

		HAL_Delay(Hold_Time);

		ADC_Tmp_On[i] = Repeater_Rep_IN_B_ADC();

		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

	}

	Return_val = 0;
	Return_val_Off = 0;
	Return_val_On = 0;

	for(int i=0; i<Num ; i++){
		Return_val_Off = Return_val_Off + ADC_Tmp_Off[i];
		Return_val_On = Return_val_On + ADC_Tmp_On[i];
	}

	Return_val = Return_val_On - Return_val_Off;

	return Return_val;
}

void Com_C5(void){

	uint8_t CRC_Temp;
	uint8_t In_Volt_St, Out_Vol_St;
	uint8_t In_Volt_Val, Out_Vol_Val;

	UART_TX_buf[0] = 0x02;
	UART_TX_buf[1] = CCU_Address;
	UART_TX_buf[2] = 0x00;
	UART_TX_buf[3] = 0x00;
	UART_TX_buf[4] = 0x00;
	UART_TX_buf[5] = 0x00;
	UART_TX_buf[6] = 0x03;

	if(REPEATOR_MODE == REPEATOR){
		Rep_TX_LED(LED_ON);
		 //1. 릴레이를 off 시킨다.
		 Repeater_ISO_OUT_Off_M_Set(5);

		 HAL_Delay(10);

		 //2. ADC 실행
		 //Repeater_ISO_ADC();
		 Rep_SIG24_ADC = Repeater_Rep_SIG24_ADC_Avr(10, 2);
		 //Rep_OUT_ADC = Repeater_Rep_OUT_ADC_Avr(2, 2);

		 //Rep_SIG24_ADC = 24;
		 //3. 입 출력 모두 24V 확인시 루프백 동작으로 릴레이 off
		 if((Rep_SIG24_ADC > Comp_24V) &(Rep_OUT_ADC > Comp_24V)){
			 Rep_ISO_In_Open = ISO_Normal;
			 Rep_ISO_In_Short = ISO_Normal;
			 Rep_ISO_Out_Open = ISO_Normal;
			 Rep_ISO_Out_Short = ISO_Normal;
			 Rep_ISO_St = ISO_Error;
			 Repeater_ISO_OUT_Off_Set();
		 }
		 //4. 입력 24v이상, 출력 24v 이하
		 else if((Rep_SIG24_ADC > Comp_24V) &(Rep_OUT_ADC <= Comp_24V)){
			 //4.1 Out Short 체크
			Rep_OUT_B_ADC = Rep_Check_Out_Short_Avr(2, 2);

			//4.2 Out Open 확인
			if(Rep_OUT_B_ADC < Out_Open_Val){
				Rep_ISO_In_Open = ISO_Normal;
				Rep_ISO_In_Short = ISO_Normal;
				Rep_ISO_Out_Open = ISO_Error;
				Rep_ISO_Out_Short = ISO_Normal;
				Rep_ISO_St = ISO_Normal;
				//Repeater_ISO_OUT_On_Set();
			}
			//4.3 Out 정상 확인
			else if((Rep_OUT_B_ADC >= Out_Open_Val) &(Rep_OUT_B_ADC < Out_Short_Val)) {
				Rep_ISO_In_Open = ISO_Normal;
				Rep_ISO_In_Short = ISO_Normal;
				Rep_ISO_Out_Open = ISO_Normal;
				Rep_ISO_Out_Short = ISO_Normal;
				Rep_ISO_St = ISO_Normal;
				//Repeater_ISO_OUT_On_Set();
			}
			//4.3 Out Short 확인
			else if(Rep_OUT_B_ADC >= Out_Short_Val){
				Rep_ISO_In_Open = ISO_Normal;
				Rep_ISO_In_Short = ISO_Normal;
				Rep_ISO_Out_Open = ISO_Normal;
				Rep_ISO_Out_Short = ISO_Error;
				Rep_ISO_St = ISO_Error;
				Repeater_ISO_OUT_Off_Set();
			}

		 }
		 //5. 입력 24v이하, 출력 24v 이상
		 else if((Rep_SIG24_ADC <= Comp_24V) &(Rep_OUT_ADC > Comp_24V)){
			 //5.1 In Short 체크
				Repeater_ISO_OUT_Off_Set();
				Rep_IN_B_ADC = Rep_Check_In_Short_Avr(2, 2);

				//5.2 In Open 확인
				if(Rep_IN_B_ADC < Out_Open_Val){
					Rep_ISO_In_Open = ISO_Error;
					Rep_ISO_In_Short = ISO_Normal;
					Rep_ISO_Out_Open = ISO_Normal;
					Rep_ISO_Out_Short = ISO_Normal;
					Rep_ISO_St = ISO_Normal;
					//Repeater_ISO_OUT_On_Set();
				}
				//5.3 In 정상 확인
				else if((Rep_IN_B_ADC >= In_Open_Val) &(Rep_IN_B_ADC < In_Short_Val)) {
					Rep_ISO_In_Open = ISO_Normal;
					Rep_ISO_In_Short = ISO_Normal;
					Rep_ISO_Out_Open = ISO_Normal;
					Rep_ISO_Out_Short = ISO_Normal;
					Rep_ISO_St = ISO_Normal;
					//Repeater_ISO_OUT_On_Set();
				}
				//5.3 In Short 확인
				else if(Rep_IN_B_ADC >= In_Short_Val){
					Rep_ISO_In_Open = ISO_Normal;
					Rep_ISO_In_Short = ISO_Error;
					Rep_ISO_Out_Open = ISO_Normal;
					Rep_ISO_Out_Short = ISO_Normal;
					Rep_ISO_St = ISO_Error;
					Repeater_ISO_OUT_Off_Set();
				}

		 }

		 if(Rep_SIG24_ADC <= Comp_24V){
			 In_Volt_St= 0;
		 }
		 else{
			 In_Volt_St= 1;
		 }

		 if(Rep_OUT_ADC <= Comp_24V){
			 Out_Vol_St= 0;
		 }
		 else{
			 Out_Vol_St= 1;
		 }

		//UART_TX_buf[2] : 회선과 릴레이 정보
		//		 0b0??? ???? : 입력회로 정상
		//		 0b1??? ???? : 입력회로 단선
		//		 0b?0?? ???? : 입력회로 정상
		//		 0b?1?? ???? : 입력회로 단락
		//		 0b??0? ???? : 출력회로 정상
		//		 0b??1? ???? : 출력회로 단선
		//		 0b???0 ???? : 출력회로 정상
		//		 0b???1 ???? : 출력회로 단락
		//		 0b???? 1??? : 입력 전압 상태 on
		//		 0b???? 0??? : 입력 전압 상태 off
		//		 0b???? ?1?? : 출력 전압 상태 on
		//		 0b???? ?0?? : 입력 전압 상태 off off0b???? ??1? : 출력 릴레 상태 on
		//		 0b???? ??0? : 입력 릴레이 상태 off
		//		 0b???? ???1 : 출력 릴레이 상태 on
		//		 0b???? ???0 : 출력 릴레이 상태 off

		 UART_TX_buf[2] = (((Rep_ISO_In_Open&0x01) << 7)|((Rep_ISO_In_Short&0x01) << 6)|
						((Rep_ISO_Out_Open&0x01) << 5)|((Rep_ISO_Out_Short&0x01) << 4)|
						((In_Volt_St&0x01) << 3)|((Out_Vol_St&0x01) << 2)|
						((0&0x01) << 1)|((Rep_Out_Relay_Mode&0x01) << 0));

		 //UART_TX_buf[3] : 입력 전압
		 if((uint8_t)(Rep_SIG24_ADC*10) > 255){
			 Rep_SIG24_ADC = 25.5;
		 }
		 In_Volt_Val = (uint8_t)(Rep_SIG24_ADC*10);
		 UART_TX_buf[3] = In_Volt_Val;

		 //UART_TX_buf[4] : 출력 전압
		 if((uint8_t)(Rep_OUT_ADC*10) > 255){
			 Rep_OUT_ADC = 25.5;
		 }
		 Out_Vol_Val = (uint8_t)(Rep_OUT_ADC*10);
		 UART_TX_buf[4] = Out_Vol_Val;

		CRC_Temp = UART_TX_buf[1] ^ UART_TX_buf[2] ^ UART_TX_buf[3] ^ UART_TX_buf[4];
		UART_TX_buf[5] = CRC_Temp;

		HAL_UART_Transmit(&hlpuart2, UART_TX_buf, 7, 1000);
		Rep_TX_LED(LED_OFF);


	}
	else if(REPEATOR_MODE == ANALOG){

		//UART_TX_buf[2] : 회선과 릴레이 정보
		//		 0b0??? ???? : 입력회로 정상
		//		 0b1??? ???? : 입력회로 단선
		//		 0b?0?? ???? : 입력회로 정상
		//		 0b?1?? ???? : 입력회로 단락
		//		 0b??0? ???? : 출력회로 정상
		//		 0b??1? ???? : 출력회로 단선
		//		 0b???0 ???? : 출력회로 정상
		//		 0b???1 ???? : 출력회로 단락
		//		 0b???? 1??? : 입력 전압 상태 on
		//		 0b???? 0??? : 입력 전압 상태 off
		//		 0b???? ?1?? : 출력 전압 상태 on
		//		 0b???? ?0?? : 입력 전압 상태 off off0b???? ??1? : 출력 릴레 상태 on
		//		 0b???? ??0? : 입력 릴레이 상태 off
		//		 0b???? ???1 : 출력 릴레이 상태 on
		//		 0b???? ???0 : 출력 릴레이 상태 off

		uint8_t In_ISO_open;
		uint8_t In_ISO_short;
		uint8_t Out_ISO_open;
		uint8_t Out_ISO_short;

		float Init_IN_ADC_Volt_tmp;
		float Init_OUT_ADC_Volt_tmp;

		if(Ana_In_ISO_Mode == ISO_Normal){
			In_ISO_open = 0;
			In_ISO_short = 0;
		}
		else if(Ana_In_ISO_Mode == ISO_Short){
			In_ISO_open = 0;
			In_ISO_short = 1;
		}
		else if(Ana_In_ISO_Mode == ISO_Open){
			In_ISO_open = 1;
			In_ISO_short = 0;
		}

		if(Ana_Out_ISO_Mode == ISO_Normal){
			Out_ISO_open = 0;
			Out_ISO_short = 0;
		}
		else if(Ana_Out_ISO_Mode == ISO_Short){
			Out_ISO_open = 0;
			Out_ISO_short = 1;
		}
		else if(Ana_Out_ISO_Mode == ISO_Open){
			Out_ISO_open = 1;
			Out_ISO_short = 0;
		}

		if(Init_IN_ADC_Volt > 12){
			In_Volt_St = 1;
		}
		else{
			In_Volt_St = 0 ;
		}

		if(Init_OUT_ADC_Volt > 12){
			Out_Vol_St = 1;
		}
		else{
			Out_Vol_St=0;
		}

		UART_TX_buf[2] = (((In_ISO_open&0x01) << 7)|((In_ISO_short&0x01) << 6)|
						((Out_ISO_open&0x01) << 5)|((Out_ISO_short&0x01) << 4)|
						((In_Volt_St&0x01) << 3)|((Out_Vol_St&0x01) << 2)|
						((Init_Anal_ISO_IN_RY_Mode&0x01) << 1)|((Init_Anal_ISO_OUT_RY_Mode&0x01) << 0));

		 //UART_TX_buf[3] : 입력 전압
		 if((uint8_t)(Init_IN_ADC_Volt*10)>255){
			 Init_IN_ADC_Volt_tmp = 255;
		 }
		 else{
			 Init_IN_ADC_Volt_tmp = (uint8_t)(Init_IN_ADC_Volt*10);
		 }
		 UART_TX_buf[3] = Init_IN_ADC_Volt_tmp;

		 //UART_TX_buf[4] : 출력 전압
		 if((uint8_t)(Init_OUT_ADC_Volt*10) > 255){
			 Init_OUT_ADC_Volt_tmp = 255;
		 }
		 else{
			 Init_OUT_ADC_Volt_tmp = (uint8_t)(Init_OUT_ADC_Volt*10);
		 }
		 UART_TX_buf[4] = Init_OUT_ADC_Volt_tmp;

		CRC_Temp = UART_TX_buf[1] ^ UART_TX_buf[2] ^ UART_TX_buf[3] ^ UART_TX_buf[4];
		UART_TX_buf[5] = CRC_Temp;

		HAL_UART_Transmit(&hlpuart2, UART_TX_buf, 7, 1000);
		Rep_TX_LED(LED_OFF);

	}
}

void Com_C6(void){

	uint8_t CRC_Temp;

	CRC_Temp = UART_RX_buf[1] ^ UART_RX_buf[2] ^ UART_RX_buf[3];
	if(CRC_Temp == UART_RX_buf[4]){
		if(REPEATOR_MODE == REPEATOR){
			if( (UART_RX_buf[3]&0x01) == 1 ){
				Repeater_ISO_OUT_On_M_Set(5);
			}
			else if( (UART_RX_buf[3]&0x01) == 0 ){
				Repeater_ISO_OUT_Off_M_Set(5);
			}
		}
		else if(REPEATOR_MODE == ANALOG){
			if( (UART_RX_buf[3]&0x01) == 1 ){
				Analog_ISO_OUT_On_M_Set(5);
			}
			else if( (UART_RX_buf[3]&0x01) == 0 ){
				Analog_ISO_OUT_Off_M_Set(5);
			}

			if( ((UART_RX_buf[3] >> 1 ) &0x01) == 1 ){
				Analog_ISO_IN_On_M_Set(5);
			}
			else if(((UART_RX_buf[3]>>1)&0x01) == 0 ){
				Analog_ISO_IN_Off_M_Set(5);
			}
		}
	}

}

void Com_F0(void){
	NVIC_SystemReset();
}
/*
void Check_Ch1(void){

	if((REPEATOR_MODE == REPEATOR)&(REPEATOR_OS_MODES == Open_Short)){

		HAL_GPIO_WritePin(OPEN_SHORT_CON_GPIO_Port, OPEN_SHORT_CON_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);

		if(HAL_GPIO_ReadPin(OPEN_SHORT_IN_GPIO_Port, OPEN_SHORT_IN_Pin) == GPIO_PIN_RESET){
			Ch1_Out_Short = 1;
			if(Ch1_Tset_Mode == 1){
				Rep_TX_LED(LED_ON);
			}

		}
		else{
			Ch1_Out_Short = 0;
			if(Ch1_Tset_Mode == 1){
				Rep_TX_LED(LED_OFF);
			}
		}
		HAL_GPIO_WritePin(OPEN_SHORT_CON_GPIO_Port, OPEN_SHORT_CON_Pin, GPIO_PIN_SET);
	}

}
*/

#define Ck_Cnt_Num	10
#define Ch1_err_Cnt	6
uint8_t Ch1_err_Save[Ch1_err_Cnt];
uint16_t Ch1_err_Save_Sum;
uint8_t Ch1_On_Data[Ck_Cnt_Num];
uint8_t Ch1_Off_Data[Ck_Cnt_Num];
uint16_t Ch1_On_Save_Sum;
uint16_t Ch1_Off_Save_Sum;


void Check_Ch1(void){

	if((REPEATOR_MODE == REPEATOR)&(REPEATOR_OS_MODES == Open_Short)){
		Ch1_Open_Cnt_Sum = 0;
		Ch1_Open_int_Cnt_Sum=0;

		for(int i = (Ch1_err_Cnt - 1); i > 0  ; i--){
			Ch1_err_Save[i] = Ch1_err_Save[i-1];
		}

		for(int i=0; i<Ck_Cnt_Num; i++){
			Ch1_On_Data[i] = 0;
			Ch1_Off_Data[i] = 0;
		}

		HAL_GPIO_WritePin(OPEN_SHORT_CON_GPIO_Port, OPEN_SHORT_CON_Pin, GPIO_PIN_RESET);
		for(int i=0 ; i< Ck_Cnt_Num; i++){
			if(HAL_GPIO_ReadPin(Open_Short_In_GPIO_Port, Open_Short_In_Pin) == GPIO_PIN_SET){
				Ch1_On_Data[i] = 1;
			}
			else{
				Ch1_On_Data[i] = 0;
			}
		}
		HAL_GPIO_WritePin(OPEN_SHORT_CON_GPIO_Port, OPEN_SHORT_CON_Pin, GPIO_PIN_SET);


		for(int i=0 ; i< Ck_Cnt_Num; i++){
			if(HAL_GPIO_ReadPin(Open_Short_In_GPIO_Port, Open_Short_In_Pin) == GPIO_PIN_SET){
				Ch1_Off_Data[i] = 1;
			}
			else{
				Ch1_Off_Data[i] = 0;
			}
		}

		Ch1_On_Save_Sum = 0;
		Ch1_Off_Save_Sum = 0;
		Ch1_err_Save[0] = OS_ok;


		for(int i=0 ; i< Ck_Cnt_Num; i++){
			Ch1_On_Save_Sum = Ch1_On_Save_Sum + Ch1_On_Data[i];
			Ch1_Off_Save_Sum = Ch1_Off_Save_Sum + Ch1_Off_Data[i];
		}

		/*
		if(Ch1_On_Save_Sum < (Ck_Cnt_Num/4) ){
			Ch1_err_Save[0] = OS_Short;
		}
		else{
			if(Ch1_Off_Save_Sum < (Ck_Cnt_Num/4) ){
				Ch1_err_Save[0] = OS_ok;
			}
			else{
				Ch1_err_Save[0] = OS_Open;
			}

		}
		*/

		if(Ch1_On_Save_Sum == 0 ){
			Ch1_err_Save[0] = OS_Short;
		}
		else{
			if(Ch1_Off_Save_Sum == 0 ){
				Ch1_err_Save[0] = OS_ok;
			}
			else{
				Ch1_err_Save[0] = OS_Open;
			}
		}

		Ch1_err_Save_Sum = 0;
		for(int i = 0; i< Ch1_err_Cnt ; i++){
			if(Ch1_err_Save[i] == 0){
				Ch1_err_Save_Sum += 1;
			}
		}

		if(Ch1_err_Save_Sum >= (Ch1_err_Cnt * 0.5) ){
			Ch1_Out_Open = Open_Short_ok;
		}
		else{
			Ch1_Out_Open = Open_Short_err;
		}

	}
	else{
		Ch1_Out_Open = Open_Short;

	}
}
