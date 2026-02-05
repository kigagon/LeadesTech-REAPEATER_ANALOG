/*
 * function_Analog_ISO.c
 *
 *  Created on: Jun 9, 2025
 *      Author: ltlab2
 */


#include "main.h"
#include "function_Analog_ISO.h"
#include "function_Analog.h"

#include "Function_Repeater.h"
#include "flash.h"

#include <math.h>

float COM_ADC_Volt;
float IN_ADC_Volt;
float OUT_ADC_Volt;

float OUT_A_ADC_Volt;
float OUT_B_ADC_Volt;

float IN_A_ADC_Volt;
float IN_B_ADC_Volt;

float IN_A_On_A_ADC_Volt;
float IN_A_On_B_ADC_Volt;

float IN_B_On_A_ADC_Volt;
float IN_B_On_B_ADC_Volt;

float OUT_A_On_A_ADC_Volt;
float OUT_A_On_B_ADC_Volt;

float OUT_B_On_A_ADC_Volt;
float OUT_B_On_B_ADC_Volt;

float IN_A_On_Dela_ADC_Volt;
float IN_B_On_Dela_ADC_Volt;
float OUT_A_On_Dela_ADC_Volt;
float OUT_B_On_Dela_ADC_Volt;

float Init_IN_ADC_Volt;
float Init_OUT_ADC_Volt;
uint8_t Init_Anal_ISO_IN_RY_Mode;
uint8_t Init_Anal_ISO_OUT_RY_Mode;

void Check_ISO_Start(void){

	if(ISO_MODE == ISO){
		// 릴레이 접점 불량 확인용
		if(Analog_Address == 0)
		{
			while(1){
				Anal_ISO_IN_On_Set();
				HAL_Delay(1000);
				Anal_ISO_IN_Off_Set();
				HAL_Delay(1000);
				Anal_ISO_OUT_On_Set();
				HAL_Delay(1000);
				Anal_ISO_OUT_Off_Set();
				HAL_Delay(1000);
				}
		}

		Anal_ISO_IN_RY_Mode = Rel_On;
		Anal_ISO_OUT_RY_Mode = Rel_On;

		Analog_ISO_OUT_Off_M_Set(5);
		Analog_ISO_IN_Off_M_Set(5);
		//Anal_ISO_IN_Off_Set_Num(On_Off_Num);
		//Anal_ISO_OUT_Off_Set_Num(On_Off_Num);


		Set_Test_3V(In_A_Test_Port, Test_Port_Off);
		Set_Test_3V(In_B_Test_Port, Test_Port_Off);
		Set_Test_3V(Out_A_Test_Port, Test_Port_Off);
		Set_Test_3V(Out_B_Test_Port, Test_Port_Off);


		// 1. 초기값 설정
		Ana_Out_ISO_Pre_Mode = ISO_Normal;
		Ana_Out_ISO_Mode = ISO_Normal;
		Ana_In_ISO_Pre_Mode = ISO_Normal;
		Ana_In_ISO_Mode = ISO_Normal;

		//1. 콘덴서 충전을 기다린다.
		HAL_Delay(500);

		//2.이전 저장된 동작 을 확인 하여 릴레이를 동작한다.
		Flash_Read_uint8(ISO_STATUS_ADDR, &Pre_IN_Short);
		Flash_Read_uint8(ISO_STATUS_ADDR + 8, &Pre_OUT_Short);
		//Flash_Write_uint8(ISO_STATUS_ADDR, 0);
		//Flash_Write_uint8(ISO_STATUS_ADDR + 8, 0);

		if(Pre_IN_Short == 255){
			Pre_IN_Short = 0;
		}

		if(Pre_OUT_Short == 255){
			Pre_OUT_Short = 0;
		}

		//3. 전원 인가 포트 확인 초기 전압 불안정 구간 확인
		for(int i=0; i<50 ; i++){

			IN_ADC_Volt = Analog_Read_IN_ADC_Volt_Avr(5,10);
			OUT_ADC_Volt = Analog_Read_Out_ADC_Volt_Avr(5,10);
			if((IN_ADC_Volt < comp_24V_val)&(OUT_ADC_Volt < comp_24V_val)){
				HAL_Delay(10);
			}
			else{
				break;
			}
		}

		//4.입력 출력 모두 정상일때
		if((Pre_OUT_Short == 0) & (Pre_IN_Short == 0)){


			IN_ADC_Volt = Analog_Read_IN_ADC_Volt_Avr(5,10);
			OUT_ADC_Volt = Analog_Read_Out_ADC_Volt_Avr(5,10);

			Init_IN_ADC_Volt = IN_ADC_Volt;
			Init_OUT_ADC_Volt = OUT_ADC_Volt;

			if((IN_ADC_Volt > comp_24V_val)&(OUT_ADC_Volt > comp_24V_val)){
				// 4.1 양쪽다 전원 인가시 OUT를 연결한다.
				Analog_ISO_OUT_On_M_Set(5);
				Analog_ISO_IN_Off_M_Set(5);
				//Anal_ISO_IN_Off_Set_Num(On_Off_Num);
				//Anal_ISO_OUT_On_Set_Num(On_Off_Num);

			}
			else{
				// 4.2 한쪽만 전원 인가시
				if((IN_ADC_Volt > comp_24V_val)&(OUT_ADC_Volt <= comp_24V_val)){
					Analog_ISO_OUT_Off_M_Set(5);
					Analog_ISO_IN_On_M_Set(5);
					//Anal_ISO_IN_On_Set_Num(On_Off_Num);
					//Anal_ISO_OUT_Off_Set_Num(On_Off_Num);
				}
				else if((IN_ADC_Volt <= comp_24V_val)&(OUT_ADC_Volt > comp_24V_val)){
					Analog_ISO_OUT_On_M_Set(5);
					Analog_ISO_IN_Off_M_Set(5);
					//Anal_ISO_IN_Off_Set_Num(On_Off_Num);
					//Anal_ISO_OUT_On_Set_Num(On_Off_Num);
				}

				// 4.3 주소가 220일때에는 이상태를 유지하고 아니면 다른 릴레이를 on해 단락인지 확인한다.

				if(Analog_Address != 220){
				    //4.4 출력 on, 입력 off
					if((Anal_ISO_OUT_RY_Mode == Rel_On)&(Anal_ISO_IN_RY_Mode == Rel_Off)){
						Analog_ISO_IN_On_M_Set(5);
						HAL_Delay(100);

						COM_ADC_Volt = Analog_Read_Com_ADC_Volt_Avr(5,10);

						// 전원 인가시 전원이 다운되면 입력쪽 단락, 플래쉬 저장
						if(COM_ADC_Volt < comp_24V_val){
							Analog_ISO_IN_Off_M_Set(5);
							Pre_OUT_Short = 0;
							Pre_IN_Short = 1;
							Flash_Erase(ISO_STATUS_ADDR);
							Flash_Write_uint8(ISO_STATUS_ADDR + 8, Pre_OUT_Short);
							Flash_Write_uint8(ISO_STATUS_ADDR, Pre_IN_Short);
							Ana_In_ISO_Mode  = ISO_Short;
							Ana_Out_ISO_Mode  = ISO_Normal;
						}
					}
				    //4.5 출력 off, 입력 on
					if((Anal_ISO_OUT_RY_Mode == Rel_Off)&(Anal_ISO_IN_RY_Mode == Rel_On)){
						Analog_ISO_OUT_On_M_Set(5);
						HAL_Delay(100);
						COM_ADC_Volt = Analog_Read_Com_ADC_Volt_Avr(5,10);

						// 전원 인가시 전원이 다운되면 입력쪽 단락, 플래쉬 저장
						if(COM_ADC_Volt < comp_24V_val){
							Analog_ISO_OUT_Off_M_Set(5);
							Pre_OUT_Short = 1;
							Pre_IN_Short = 0;
							Flash_Erase(ISO_STATUS_ADDR);
							Flash_Write_uint8(ISO_STATUS_ADDR + 8, Pre_OUT_Short);
							Flash_Write_uint8(ISO_STATUS_ADDR, Pre_IN_Short);
							Ana_In_ISO_Mode  = ISO_Normal;
							Ana_Out_ISO_Mode  = ISO_Short;
						}
					}
				}
				else{

				}

			}
		}
		//3. 릴레이가 정상이 아닐때에는 프래쉬에 맞게 릴레이를 제어하고 릴레이 값을 정상으로 해 프래쉬에 저장한다.
		else{

			if((Pre_OUT_Short == 1) & (Pre_IN_Short == 0)){
				Analog_ISO_OUT_Off_M_Set(5);
				Analog_ISO_IN_On_M_Set(5);

				Ana_Out_ISO_Mode  = ISO_Short;
				Ana_In_ISO_Mode = ISO_Normal;
			}
			else if((Pre_OUT_Short == 0) & (Pre_IN_Short == 1)){
				Analog_ISO_OUT_On_M_Set(5);
				Analog_ISO_IN_Off_M_Set(5);

				Ana_In_ISO_Mode  = ISO_Short;
				Ana_Out_ISO_Mode = ISO_Normal;
			}
			Pre_OUT_Short = 0;
			Pre_IN_Short = 0;
			Flash_Erase(ISO_STATUS_ADDR);
			Flash_Write_uint8(ISO_STATUS_ADDR + 8, Pre_OUT_Short);
			Flash_Write_uint8(ISO_STATUS_ADDR, Pre_IN_Short);

		}
	}

	Init_Anal_ISO_IN_RY_Mode = Anal_ISO_IN_RY_Mode;
	Init_Anal_ISO_OUT_RY_Mode = Anal_ISO_OUT_RY_Mode;
}

void Check_ISO(void){

	if(ISO_MODE == ISO){

		if((Ana_Out_ISO_Mode != ISO_Short)&(Ana_In_ISO_Mode != ISO_Short)){
			// 1.1 전압을 체크한다.
			COM_ADC_Volt = Analog_Read_Com_ADC_Volt_Avr(1,1);
			// 2.1 	전압이 떨어지면 릴레이를 오프한후 50ms 대기후전압을 다시 체크 한다
			if(COM_ADC_Volt < comp_24V_val){

				HAL_Delay(50);
				COM_ADC_Volt = Analog_Read_Com_ADC_Volt_Avr(5,10);

				// 2.2 50ms 후에도 전압이 복구 되면 정상	으로 확인
				if(COM_ADC_Volt >= comp_24V_val){
					Analog_ISO_OUT_On_M_Set(5);
					Analog_ISO_IN_On_M_Set(5);
				}
				// 2.3 50ms 후에도 전압이 복구 안되면 단락으로 확인
				else{
					Analog_ISO_OUT_Off_M_Set(5);
					Analog_ISO_IN_Off_M_Set(5);

					//2.4 릴레이 off후  전압 인가 되는 쪽은 정상 전압이 없는 곳을 단락 확인
					IN_ADC_Volt = Analog_Read_IN_ADC_Volt_Avr(5,10);
					OUT_ADC_Volt = Analog_Read_Out_ADC_Volt_Avr(5,10);

					if((IN_ADC_Volt < comp_24V_val)&(OUT_ADC_Volt >= comp_24V_val)){
						Ana_Out_ISO_Mode  = ISO_Normal;
						Ana_In_ISO_Mode = ISO_Short;
					}
					else if((IN_ADC_Volt >= comp_24V_val)&(OUT_ADC_Volt < comp_24V_val)){
						Ana_Out_ISO_Mode  = ISO_Short;
						Ana_In_ISO_Mode = ISO_Normal;
					}

				}

			}
		}

	}
}


void Check_ISO_det(void){

	COM_ADC_Volt = Analog_Read_Com_ADC_Volt_Avr(5,10);
	//3. 전원 인가 포트 확인
	IN_ADC_Volt = Analog_Read_IN_ADC_Volt_Avr(5,10);
	OUT_ADC_Volt = Analog_Read_Out_ADC_Volt_Avr(5,10);


	/*
	//4. COM_ADC_Volt 전원이 안들어 오면  in out 통신 없음
	if(COM_ADC_Volt > comp_24V_val){

		//5. 양쪽에서 전원 인가시 out 릴레이 만 on
		if((IN_ADC_Volt > comp_24V_val)&(OUT_ADC_Volt > comp_24V_val)){

			Analog_ISO_OUT_On_M_Set(5);
			Analog_ISO_IN_Off_M_Set(5);

		}
		//6. Out 에 전원 인가 확인시 In port 단선 단락 확인
		else if((OUT_ADC_Volt > comp_24V_val)&(IN_ADC_Volt <= comp_24V_val)){

			Set_Test_3V(In_A_Test_Port, Test_Port_On);
			Set_Test_3V(In_B_Test_Port, Test_Port_Off);
			HAL_Delay(100);
			IN_A_On_A_ADC_Volt = Read_IN_A_ADC_Volt();
			IN_A_On_B_ADC_Volt = Read_IN_B_ADC_Volt();

			Set_Test_3V(In_A_Test_Port, Test_Port_Off);
			Set_Test_3V(In_B_Test_Port, Test_Port_On);
			HAL_Delay(100);
			IN_B_On_A_ADC_Volt = Read_IN_A_ADC_Volt();
			IN_B_On_B_ADC_Volt = Read_IN_B_ADC_Volt();

			Set_Test_3V(In_A_Test_Port, Test_Port_Off);
			Set_Test_3V(In_B_Test_Port, Test_Port_Off);

			IN_A_On_Dela_ADC_Volt = fabs(IN_A_On_A_ADC_Volt - IN_A_On_B_ADC_Volt);
			IN_B_On_Dela_ADC_Volt = fabs(IN_B_On_A_ADC_Volt - IN_B_On_B_ADC_Volt);

			if((IN_A_On_Dela_ADC_Volt<comp_Dela_val)&(IN_B_On_Dela_ADC_Volt<comp_Dela_val)){
				Ana_In_ISO_Mode = ISO_Short	;
			}
			else if(((IN_A_On_A_ADC_Volt > comp_Short_val)&(IN_A_On_B_ADC_Volt <= comp_Short_val))
					&((IN_B_On_A_ADC_Volt > comp_Short_val)&(IN_B_On_B_ADC_Volt <= comp_Short_val))){
				Ana_In_ISO_Mode = ISO_Normal	;
			}
			else if(((IN_A_On_A_ADC_Volt > comp_Short_val)&(IN_A_On_B_ADC_Volt <= comp_Open_val))
					&((IN_B_On_A_ADC_Volt <= comp_Open_val)&(IN_B_On_B_ADC_Volt > comp_Short_val))){
				Ana_In_ISO_Mode = ISO_Open	;
			}
			else{
				Ana_In_ISO_Mode = ISO_Normal	;
			}

			Ana_Out_ISO_Mode = ISO_Normal	;


			if(Ana_In_ISO_Mode == ISO_Normal){
				Anal_ISO_IN_On_Set_Num(On_Off_Num);
			}
			else{
				Anal_ISO_IN_Off_Set_Num(On_Off_Num);
			}

			if(Ana_Out_ISO_Mode == ISO_Normal){
				Anal_ISO_OUT_On_Set_Num(On_Off_Num);
			}
			else{
				Anal_ISO_OUT_Off_Set_Num(On_Off_Num);
			}

		}
		//6. In 에 전원 인가 확인시  port 단선 단락 확인
		else if((IN_ADC_Volt > comp_24V_val)&(OUT_ADC_Volt <= comp_24V_val)){

			Set_Test_3V(Out_A_Test_Port, Test_Port_On);
			Set_Test_3V(Out_B_Test_Port, Test_Port_Off);
			HAL_Delay(100);
			OUT_A_On_A_ADC_Volt = Read_OUT_A_ADC_Volt();
			OUT_A_On_B_ADC_Volt = Read_OUT_B_ADC_Volt();

			Set_Test_3V(Out_A_Test_Port, Test_Port_Off);
			Set_Test_3V(Out_B_Test_Port, Test_Port_On);
			HAL_Delay(100);
			OUT_B_On_A_ADC_Volt = Read_OUT_A_ADC_Volt();
			OUT_B_On_B_ADC_Volt = Read_OUT_B_ADC_Volt();

			Set_Test_3V(In_A_Test_Port, Test_Port_Off);
			Set_Test_3V(In_B_Test_Port, Test_Port_Off);

			OUT_A_On_Dela_ADC_Volt = fabs(OUT_A_On_A_ADC_Volt - OUT_A_On_B_ADC_Volt);
			OUT_B_On_Dela_ADC_Volt = fabs(OUT_B_On_A_ADC_Volt - OUT_B_On_B_ADC_Volt);

			if((OUT_A_On_Dela_ADC_Volt<comp_Dela_val)&(OUT_B_On_Dela_ADC_Volt<comp_Dela_val)){
				Ana_Out_ISO_Mode = ISO_Short	;
			}
			else if(((OUT_A_On_A_ADC_Volt > comp_Short_val)&(OUT_A_On_B_ADC_Volt <= comp_Short_val))
					&((OUT_B_On_A_ADC_Volt > comp_Short_val)&(OUT_B_On_B_ADC_Volt <= comp_Short_val))){
				Ana_Out_ISO_Mode = ISO_Normal	;
			}
			else if(((OUT_A_On_A_ADC_Volt > comp_Short_val)&(OUT_A_On_B_ADC_Volt <= comp_Open_val))
					&((OUT_B_On_A_ADC_Volt <= comp_Open_val)&(OUT_B_On_B_ADC_Volt > comp_Short_val))){
				Ana_Out_ISO_Mode = ISO_Open	;
			}
			else{
				Ana_Out_ISO_Mode = ISO_Normal	;
			}
			Ana_In_ISO_Mode = ISO_Normal	;

			if(Ana_In_ISO_Mode == ISO_Normal){
				Anal_ISO_IN_On_Set_Num(On_Off_Num);
			}
			else{
				Anal_ISO_IN_Off_Set_Num(On_Off_Num);
			}

			if(Ana_Out_ISO_Mode == ISO_Normal){
				Anal_ISO_OUT_On_Set_Num(On_Off_Num);
			}
			else{
				Anal_ISO_OUT_Off_Set_Num(On_Off_Num);
			}
		}

	}
	else{
		//5. COM_ADC_Volt 전원이 안들어 오면  in out 통신 없음확인시 릴레이 off
		Anal_ISO_IN_Off_Set_Num(On_Off_Num);
		Anal_ISO_OUT_Off_Set_Num(On_Off_Num);

		while(1){
			COM_ADC_Volt = Read_Com_ADC_Volt();
			 if(COM_ADC_Volt > comp_24V_val){
				 NVIC_SystemReset( );
			 }
		 }

	}

	*/
}
float Read_Com_ADC_Volt(void){

	float Adc_Val;

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
	Adc_Val = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);

	return Adc_Val;
}


float Analog_Read_Com_ADC_Volt_Avr(int Num, int Hold_Time){

	float ADC_Tmp[Num], Return_val;

	for(int i=0; i<Num ; i++){
		ADC_Tmp[i] = Read_Com_ADC_Volt();
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

float Read_IN_ADC_Volt(void){

	float Adc_Val;

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

	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	Adc_Val = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);

	return Adc_Val;
}

float Analog_Read_IN_ADC_Volt_Avr(int Num, int Hold_Time){

	float ADC_Tmp[Num], Return_val;

	for(int i=0; i<Num ; i++){
		ADC_Tmp[i] = Read_IN_ADC_Volt();
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

float Read_Out_ADC_Volt(void){

	float Adc_Val;

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

	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	Adc_Val = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);

	return Adc_Val;
}

float Analog_Read_Out_ADC_Volt_Avr(int Num, int Hold_Time){

	float ADC_Tmp[Num], Return_val;

	for(int i=0; i<Num ; i++){
		ADC_Tmp[i] = Read_Out_ADC_Volt();
		HAL_Delay(Hold_Time);
		//Check_Com();
	}
	Return_val = 0;
	for(int i=0; i<Num ; i++){
		Return_val = Return_val + ADC_Tmp[i];
	}

	Return_val = Return_val / Num;

	if(Return_val < 12){
		Return_val = Return_val + 1;
	}
	return Return_val;

}

float Read_OUT_A_ADC_Volt(void){

	float Adc_Val;

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

	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	Adc_Val = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);

	return Adc_Val;
}

float Read_OUT_B_ADC_Volt(void){

	float Adc_Val;

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
	Adc_Val = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);

	return Adc_Val;
}

float Read_IN_A_ADC_Volt(void){

	float Adc_Val;

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

	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	Adc_Val = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);

	return Adc_Val;
}

float Read_IN_B_ADC_Volt(void){

	float Adc_Val;

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

	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	Adc_Val = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);

	return Adc_Val;
}


void Set_Test_3V(int Test_Port, int OnOff){

	if(Test_Port == In_A_Test_Port){
		if(OnOff == Test_Port_Off){
			HAL_GPIO_WritePin(a_IN_A_TEST_r_LED_TXD_GPIO_Port, a_IN_A_TEST_r_LED_TXD_Pin, GPIO_PIN_SET);
		}
		else if(OnOff == Test_Port_On){
			HAL_GPIO_WritePin(a_IN_A_TEST_r_LED_TXD_GPIO_Port, a_IN_A_TEST_r_LED_TXD_Pin, GPIO_PIN_RESET);
		}
	}
	else if(Test_Port == In_B_Test_Port){
		if(OnOff == Test_Port_Off){
			HAL_GPIO_WritePin(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, a_IN_B_TEST_r_DIP_ADD0_Pin, GPIO_PIN_SET);
		}
		else if(OnOff == Test_Port_On){
			HAL_GPIO_WritePin(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, a_IN_B_TEST_r_DIP_ADD0_Pin, GPIO_PIN_RESET);
		}
	}
	else if(Test_Port == Out_A_Test_Port){
		if(OnOff == Test_Port_Off){
			HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_SET);
		}
		else if(OnOff == Test_Port_On){
			HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_RESET);
		}
	}
	else if(Test_Port == Out_B_Test_Port){
		if(OnOff == Test_Port_Off){
			HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);
		}
		else if(OnOff == Test_Port_On){
			HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_RESET);
		}
	}

}

void Anal_ISO_IN_Off_Set_Num(int Num){
	Anal_ISO_IN_Off_Set();
	for(int i=0; i<Num; i++){
		HAL_Delay(Relay_Set_ms);
		Anal_ISO_IN_On_Set();
		HAL_Delay(Relay_Set_ms);
		Anal_ISO_IN_Off_Set();
	}
	HAL_Delay(Relay_Waite_ms);
}

void Anal_ISO_IN_On_Set_Num(int Num){
	Anal_ISO_IN_On_Set();
	for(int i=0; i<Num; i++){
		HAL_Delay(Relay_Set_ms);
		Anal_ISO_IN_Off_Set();
		HAL_Delay(Relay_Set_ms);
		Anal_ISO_IN_On_Set();
	}
	HAL_Delay(Relay_Waite_ms);
}

void Anal_ISO_OUT_Off_Set_Num(int Num){
	Anal_ISO_OUT_Off_Set();
	for(int i=0; i<Num; i++){
		HAL_Delay(Relay_Set_ms);
		Anal_ISO_OUT_On_Set();
		HAL_Delay(Relay_Set_ms);
		Anal_ISO_OUT_Off_Set();
	}
	HAL_Delay(Relay_Waite_ms);
}

void Anal_ISO_OUT_On_Set_Num(int Num){
	Anal_ISO_OUT_On_Set();
	for(int i=0; i<Num; i++){
		HAL_Delay(Relay_Set_ms);
		Anal_ISO_OUT_Off_Set();
		HAL_Delay(Relay_Set_ms);
		Anal_ISO_OUT_On_Set();
	}
	HAL_Delay(Relay_Waite_ms);
}

void Analog_ISO_IN_On_M_Set(int mset){

	if(Anal_ISO_IN_RY_Mode == Rel_On){

	}
	else{
		Anal_ISO_IN_RY_Mode = Rel_On;

		HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(a_IN_A_TEST_r_LED_TXD_GPIO_Port, a_IN_A_TEST_r_LED_TXD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, a_IN_B_TEST_r_DIP_ADD0_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(a_CON_IN_SET_r_LED_ISOERR_GPIO_Port, a_CON_IN_SET_r_LED_ISOERR_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_RESET);

		HAL_Delay(mset);

		HAL_GPIO_WritePin(a_CON_IN_SET_r_LED_ISOERR_GPIO_Port, a_CON_IN_SET_r_LED_ISOERR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_RESET);
	}
}

void Analog_ISO_IN_Off_M_Set(int mset){

	if(Anal_ISO_IN_RY_Mode == Rel_Off){

	}
	else{
		Anal_ISO_IN_RY_Mode = Rel_Off;

		HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(a_IN_A_TEST_r_LED_TXD_GPIO_Port, a_IN_A_TEST_r_LED_TXD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, a_IN_B_TEST_r_DIP_ADD0_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(a_CON_IN_SET_r_LED_ISOERR_GPIO_Port, a_CON_IN_SET_r_LED_ISOERR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_SET);

		HAL_Delay(Relay_Hold_ms);

		HAL_GPIO_WritePin(a_CON_IN_SET_r_LED_ISOERR_GPIO_Port, a_CON_IN_SET_r_LED_ISOERR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_RESET);
	}
}

void Analog_ISO_OUT_On_M_Set(int mset){

	if(Anal_ISO_OUT_RY_Mode == Rel_On){

	}
	else{
		Anal_ISO_OUT_RY_Mode  = Rel_On;

		HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(a_IN_A_TEST_r_LED_TXD_GPIO_Port, a_IN_A_TEST_r_LED_TXD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, a_IN_B_TEST_r_DIP_ADD0_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(a_CON_OUT_SET_r_DISCON1_GPIO_Port, a_CON_OUT_SET_r_DISCON1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_CON_OUT_RSET_r_OCP1_GPIO_Port, a_CON_OUT_RSET_r_OCP1_Pin, GPIO_PIN_RESET);

		HAL_Delay(Relay_Hold_ms);

		HAL_GPIO_WritePin(a_CON_OUT_SET_r_DISCON1_GPIO_Port, a_CON_OUT_SET_r_DISCON1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_CON_OUT_RSET_r_OCP1_GPIO_Port, a_CON_OUT_RSET_r_OCP1_Pin, GPIO_PIN_RESET);
	}
}

void Analog_ISO_OUT_Off_M_Set(int mset){

	if(Anal_ISO_OUT_RY_Mode  == Rel_Off){

	}
	else{
		Anal_ISO_OUT_RY_Mode  = Rel_Off;

		HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(a_IN_A_TEST_r_LED_TXD_GPIO_Port, a_IN_A_TEST_r_LED_TXD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, a_IN_B_TEST_r_DIP_ADD0_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(a_CON_OUT_SET_r_DISCON1_GPIO_Port, a_CON_OUT_SET_r_DISCON1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_CON_OUT_RSET_r_OCP1_GPIO_Port, a_CON_OUT_RSET_r_OCP1_Pin, GPIO_PIN_SET);

		HAL_Delay(Relay_Hold_ms);

		HAL_GPIO_WritePin(a_CON_OUT_SET_r_DISCON1_GPIO_Port, a_CON_OUT_SET_r_DISCON1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_CON_OUT_RSET_r_OCP1_GPIO_Port, a_CON_OUT_RSET_r_OCP1_Pin, GPIO_PIN_RESET);
	}
}
