/*
 * function_Analog.c
 *
 *  Created on: Apr 13, 2025
 *      Author: root
 */



#include "main.h"
#include "function_Analog.h"
#include "function_Analog_Photo.h"
#include "INA219.h"
#include "flash.h"
#include <stdio.h>

/****************ISO Start****************/
#include "function_Analog_ISO.h"
/****************ISO End****************/

//MF52E-503H3950FAL36
uint16_t Temprature_Table[100] = {3754,3739,3723,3706,3687,3666,3648,3629,3608,3586,
								3561,3541,3519,3495,3469,3440,3417,3392,3364,3335,
								3303,3277,3248,3218,3185,3150,3121,3090,3057,3021,
								2983,2952,2918,2882,2845,2804,2771,2736,2698,2659,
								2617,2582,2546,2507,2467,2425,2389,2352,2313,2273,
								2230,2195,2158,2120,2080,2039,2004,1968,1930,1892,
								1852,1818,1783,1748,1711,1673,1641,1608,1575,1540,
								1505,1475,1444,1413,1381,1348,1321,1292,1263,1234,
								1204,1179,1153,1126,1100,1072,1049,1026,1002,978,
								954,933,912,890,869,847,828,809,790,771};

double current_Avr[31] = {0};
double Current_Value[110] = {0};

uint16_t Ana_Temp_Sen;
uint8_t Ana_Temp_Sen_Com, Ana_Temp_Sen_Com_Tmp;
float temperature_calibrationVal;

uint16_t result1, result2;
INA219_t ina219_1;

void Run_Analoge_Mode(void){

	/****************ISO Start****************/
//	ANAL_OUT_Set();
//	ANAL_IN_Set();
	/****************ISO End****************/
	HAL_Delay(500);

    ///////////////Start reading address of board//////////////////
     Analog_Address_tmp[7] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD7_r_IN1_SIG_GPIO_Port, a_DIP_ADD7_r_IN1_SIG_Pin)) & 0x01;
     Analog_Address_tmp[6] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD6_r_CH1_SW_GPIO_Port, a_DIP_ADD6_r_CH1_SW_Pin)) & 0x01;
     Analog_Address_tmp[5] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD5_r_CH2_SW_GPIO_Port, a_DIP_ADD5_r_CH2_SW_Pin)) & 0x01;
     Analog_Address_tmp[4] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD4_r_CH3_SW_GPIO_Port, a_DIP_ADD4_r_CH3_SW_Pin)) & 0x01;
     Analog_Address_tmp[3] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD3_r_CH4_SW_GPIO_Port, a_DIP_ADD3_r_CH4_SW_Pin)) & 0x01;
     Analog_Address_tmp[2] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD2_r_DIP_ADD5_GPIO_Port, a_DIP_ADD2_r_DIP_ADD5_Pin)) & 0x01;
     Analog_Address_tmp[1] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD1_r_DIP_ADD6_GPIO_Port, a_DIP_ADD1_r_DIP_ADD6_Pin)) & 0x01;
     Analog_Address_tmp[0] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD0_r_DIP_ADD7_GPIO_Port, a_DIP_ADD0_r_DIP_ADD7_Pin)) & 0x01;

     Analog_Address = (Analog_Address_tmp[7] << 7)|(Analog_Address_tmp[6] << 6)|(Analog_Address_tmp[5] << 5)|(Analog_Address_tmp[4] << 4)
             |(Analog_Address_tmp[3] << 3)|(Analog_Address_tmp[2] << 2)|(Analog_Address_tmp[1] << 1)|(Analog_Address_tmp[0] << 0);
	///////////////End reading address of board//////////////////

    if((REPEATOR_MODE == ANALOG) && (ANALOG_MODE == TEMPERATURE)){
       if(Analog_Address == 0){
          ADDRESS_MODES = Non_ADDRESS;
       }
       else{
          ADDRESS_MODES = READ_ADDRESS;
       }
    }

	if(ADDRESS_MODES == Non_ADDRESS)
	{
		Flash_Read_uint8(Waterproof_Temperature_ADDR, &Analog_Address);
//		Analog_Address = 8;
	}


	/****************ISO Start****************/
	Check_ISO_Start();
	/****************ISO End****************/


	if(ANALOG_MODE == SMOKE)
	{
//		SetSMOKE();
	}

	else if(ANALOG_MODE == TEMPERATURE)
	{
		Adjust_Y_Intercept_Value = 0;
		ReturnPercent = 0;
		Ana_photo_Sen_12bit = 0;

		if(ADDRESS_MODES == Non_ADDRESS)
		{
			temperature_calibrationVal = 1.25;
		}
		else
		{
			temperature_calibrationVal = 1.2;
		}
	}

//	AdjustmentCmdPrcessing(0xCA);
//////
//	while(1)
//	{
//		Read_Analog_ADC();
//		HAL_Delay(100);
//	}
//	Read_Analog_ADC();

	uint8_t Value[2];
	INA219_t ina219;
	while(1){
//		result1 = Read16(&ina219_1, 0x00);
//		result2 = Read16(&ina219_1, 0x3E);
//		HAL_I2C_Mem_Read(&hi2c1, 0x80, 0X3e, 1, Value, 2, 1000);

		uint8_t IsInitSuccess = 0;
		    while(!IsInitSuccess)
		    {
		    	IsInitSuccess = INA219_Init(&ina219, &hi2c1, INA219_ADDRESS);
		    }
		    (int16_t)Read16(&ina219, INA219_REG_SHUNTVOLTAGE);
	}
	//LPUART2_UART_RE_Init(0, 9600);
	while(1){

 		if(IfAddressMatched == 1)
		{
			Check_Com();
			IfAddressMatched = 0;
		}

		if(ADDRESS_MODES == READ_ADDRESS)
		{
			///////////////Start reading address of board//////////////////
			Analog_Address_tmp[7] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD7_r_IN1_SIG_GPIO_Port, a_DIP_ADD7_r_IN1_SIG_Pin)) & 0x01;
			Analog_Address_tmp[6] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD6_r_CH1_SW_GPIO_Port, a_DIP_ADD6_r_CH1_SW_Pin)) & 0x01;
			Analog_Address_tmp[5] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD5_r_CH2_SW_GPIO_Port, a_DIP_ADD5_r_CH2_SW_Pin)) & 0x01;
			Analog_Address_tmp[4] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD4_r_CH3_SW_GPIO_Port, a_DIP_ADD4_r_CH3_SW_Pin)) & 0x01;
			Analog_Address_tmp[3] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD3_r_CH4_SW_GPIO_Port, a_DIP_ADD3_r_CH4_SW_Pin)) & 0x01;
			Analog_Address_tmp[2] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD2_r_DIP_ADD5_GPIO_Port, a_DIP_ADD2_r_DIP_ADD5_Pin)) & 0x01;
			Analog_Address_tmp[1] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD1_r_DIP_ADD6_GPIO_Port, a_DIP_ADD1_r_DIP_ADD6_Pin)) & 0x01;
			Analog_Address_tmp[0] =  ~(HAL_GPIO_ReadPin(a_DIP_ADD0_r_DIP_ADD7_GPIO_Port, a_DIP_ADD0_r_DIP_ADD7_Pin)) & 0x01;

			Analog_Address = (Analog_Address_tmp[7] << 7)|(Analog_Address_tmp[6] << 6)|(Analog_Address_tmp[5] << 5)|(Analog_Address_tmp[4] << 4)
					  |(Analog_Address_tmp[3] << 3)|(Analog_Address_tmp[2] << 2)|(Analog_Address_tmp[1] << 1)|(Analog_Address_tmp[0] << 0);

			CCU_Address = Analog_Address;
			///////////////End reading address of board//////////////////
		}
		else
		{
			CCU_Address = Analog_Address;
		}

	  __HAL_UART_ENABLE_IT(&hlpuart2, UART_IT_ERR);
	  // "Enable the UART Data Register not empty Interrupt" //
	  __HAL_UART_ENABLE_IT(&hlpuart2, UART_IT_RXNE);

		if(IfAddressMatched == 1)
		{
			Check_Com();
			IfAddressMatched = 0;
		}

		if(ANALOG_MODE == TEMPERATURE)
		{
			Read_Analog_ADC();
		}

		if(IfAddressMatched == 1)
		{
//			ANAL_RED_LED(LED_OFF); // HIGH
			Check_Com();

//			ANAL_RED_LED(LED_ON); // LOW
			IfAddressMatched = 0;
		}

		Check_ISO();

		if(IfAddressMatched == 1)
		{
			Check_Com();
			IfAddressMatched = 0;
		}

		if(Reinit_Analog_uart == 1)
		{
			LPUART2_UART_RE_Init(Uart_DIR_Mode, UART_BAUDRATE);
//			HAL_Delay(10);
			Reinit_Analog_uart = 0;
		}
	}
}

void ANAL_RED_LED(uint8_t OnOff){
	if(OnOff == LED_ON){
		HAL_GPIO_WritePin(a_LED_RED_r_IN3_SIG_GPIO_Port, a_LED_RED_r_IN3_SIG_Pin, GPIO_PIN_RESET);
	}
	else if(OnOff == LED_OFF){
		HAL_GPIO_WritePin(a_LED_RED_r_IN3_SIG_GPIO_Port, a_LED_RED_r_IN3_SIG_Pin, GPIO_PIN_SET);
	}
	else if(OnOff == LED_TOGGLE)
	{
		HAL_GPIO_TogglePin(a_LED_RED_r_IN3_SIG_GPIO_Port, a_LED_RED_r_IN3_SIG_Pin);
	}
}

void ANAL_BLUE_LED(uint8_t OnOff){
	if(OnOff == LED_ON){
		HAL_GPIO_WritePin(a_LED_BLUE_r_IN2_SIG_GPIO_Port, a_LED_BLUE_r_IN2_SIG_Pin, GPIO_PIN_RESET);
	}
	else if(OnOff == LED_OFF){
		HAL_GPIO_WritePin(a_LED_BLUE_r_IN2_SIG_GPIO_Port, a_LED_BLUE_r_IN2_SIG_Pin, GPIO_PIN_SET);
	}
}

void ANAL_IN_Set(void){
	HAL_GPIO_WritePin(a_CON_IN_SET_r_LED_ISOERR_GPIO_Port, a_CON_IN_SET_r_LED_ISOERR_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(a_CON_IN_SET_r_LED_ISOERR_GPIO_Port, a_CON_IN_SET_r_LED_ISOERR_Pin, GPIO_PIN_RESET);
}

void ANAL_IN_ReSet(void){
	HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_RESET);
}

void ANAL_OUT_Set(void){
	HAL_GPIO_WritePin(a_CON_OUT_SET_r_DISCON1_GPIO_Port, a_CON_OUT_SET_r_DISCON1_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(a_CON_OUT_SET_r_DISCON1_GPIO_Port, a_CON_OUT_SET_r_DISCON1_Pin, GPIO_PIN_RESET);
}

void ANAL_OUT_ReSet(void){
	HAL_GPIO_WritePin(a_CON_OUT_RSET_r_OCP1_GPIO_Port, a_CON_OUT_RSET_r_OCP1_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(a_CON_OUT_RSET_r_OCP1_GPIO_Port, a_CON_OUT_RSET_r_OCP1_Pin, GPIO_PIN_RESET);
}

void Check_Uart_Rx(void){
	if(Uart_Error_Cnt > 50){
		  hlpuart2.Instance = LPUART2;
		  hlpuart2.Init.BaudRate = 9600;
		  hlpuart2.Init.WordLength = UART_WORDLENGTH_8B;
		  hlpuart2.Init.StopBits = UART_STOPBITS_1;
		  hlpuart2.Init.Parity = UART_PARITY_NONE;
		  hlpuart2.Init.Mode = UART_MODE_TX_RX;
		  hlpuart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		  hlpuart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		  hlpuart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
		  hlpuart2.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
		  hlpuart2.FifoMode = UART_FIFOMODE_DISABLE;
		  if (HAL_UART_Init(&hlpuart2) != HAL_OK)
		  {
		    Error_Handler();
		  }
		  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
		  {
		    Error_Handler();
		  }
		  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
		  {
		    Error_Handler();
		  }
		  if (HAL_UARTEx_DisableFifoMode(&hlpuart2) != HAL_OK)
		  {
		    Error_Handler();
		  }

		  Uart_Error_Cnt = 0;
	}
}

void Read_Analog_ADC(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	if(ANALOG_MODE == TEMPERATURE){

		sConfig.Channel = ADC_CHANNEL_3;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
		Error_Handler();
		}

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);

		Ana_Temp_Sen = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		Ana_Temp_Sen_Com_Tmp = 0;

		for(int i=1;i<100;i++){
			if(Temprature_Table[i] < Ana_Temp_Sen){
				Ana_Temp_Sen_Com_Tmp = i - 1;
				break;
			}
			Ana_Temp_Sen_Com_Tmp = i - 1;
		}
		Ana_Temp_Sen_Com = Ana_Temp_Sen_Com_Tmp * temperature_calibrationVal;
	}

	else if(ANALOG_MODE == SMOKE)
	{
//		ANAL_RED_LED(LED_ON);  // LOW

		Ana_photo_Sen_12bit = Return_Ana_photo_Sen_12bit(PGA, DAC_VALUE, 30);

		ReturnPercent = MatchAdcValueToPercentValue(AdcToPercent_MappingTable, Ana_photo_Sen_12bit);
//		ANAL_RED_LED(LED_OFF); // HIGH
	}
}

void Anal_ISO_IN_On_Set(void){

	if(Anal_ISO_IN_RY_Mode == Rel_Off){
		Anal_ISO_IN_RY_Mode = Rel_On;

		HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(a_IN_A_TEST_r_LED_TXD_GPIO_Port, a_IN_A_TEST_r_LED_TXD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, a_IN_B_TEST_r_DIP_ADD0_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(a_CON_IN_SET_r_LED_ISOERR_GPIO_Port, a_CON_IN_SET_r_LED_ISOERR_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_RESET);

		HAL_Delay(Relay_Hold_ms);

		HAL_GPIO_WritePin(a_CON_IN_SET_r_LED_ISOERR_GPIO_Port, a_CON_IN_SET_r_LED_ISOERR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_RESET);
	}
}

void Anal_ISO_IN_Off_Set(void){

	if(Anal_ISO_IN_RY_Mode == Rel_On){
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

void Anal_ISO_OUT_On_Set(void){

	if(Anal_ISO_OUT_RY_Mode == Rel_Off){
		Anal_ISO_OUT_RY_Mode = Rel_On;

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

void Anal_ISO_OUT_Off_Set(void){

	if(Anal_ISO_OUT_RY_Mode == Rel_On){
		Anal_ISO_OUT_RY_Mode = Rel_Off;

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


void Anal_ISO_24_ADC(void){
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
	Anal_SIG24_ADC = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);

}

void Anal_ISO_OUT_ADC(void){
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
	Anal_OUT_ADC = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);

}

void Anal_ISO_IN_ADC(void){
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
	Anal_IN_ADC = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);
}

void Anal_ISO_IN_AB_ADC(void){
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
	Anal_IN_A_ADC = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);

	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	Anal_IN_B_ADC = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);
}

void Anal_ISO_OUT_AB_ADC(void){
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
	Anal_OUT_A_ADC = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);

	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	Anal_OUT_B_ADC = HAL_ADC_GetValue(&hadc1) * 0.0062;
	HAL_ADC_Stop(&hadc1);
}



void Anal_Check_ISO(void){

	Ana_In_ISO_Pre_Mode = Ana_In_ISO_Mode;
	Ana_Out_ISO_Pre_Mode = Ana_Out_ISO_Mode;

	Anal_ISO_24_ADC();

	if( Anal_SIG24_ADC < V24_Check_Val){
		if(Anal_ISO_IN_RY_Mode == 1){
			if(Ana_Out_ISO_Pre_Mode == ISO_Short){

			}
			else{
				Anal_ISO_IN_Off_Set();
				Ana_In_ISO_Mode = Ana_Check_In();
			}
		}
		else{
		}

		if(Anal_ISO_OUT_RY_Mode == 1){
			if(Ana_In_ISO_Pre_Mode == ISO_Short){

			}
			else{
				Anal_ISO_OUT_Off_Set();
				Ana_Out_ISO_Mode = Ana_Check_Out();
			}
		}
		else{
		}

		if((Ana_In_ISO_Mode == ISO_V24 ) & (Ana_Out_ISO_Mode != ISO_V24)){
			Anal_ISO_IN_On_Set();
		}
		else if((Ana_In_ISO_Mode != ISO_V24 ) & (Ana_Out_ISO_Mode == ISO_V24)){
			Anal_ISO_OUT_On_Set();
		}
		else if((Ana_In_ISO_Mode != ISO_V24 ) & (Ana_Out_ISO_Mode != ISO_V24)){

			if((Ana_In_ISO_Mode == ISO_Normal)&(Ana_Out_ISO_Mode == ISO_Short)){
				Anal_ISO_IN_On_Set();
			}
			else if((Ana_In_ISO_Mode == ISO_Short)&(Ana_Out_ISO_Mode == ISO_Normal)){
				Anal_ISO_OUT_On_Set();
			}
			else if((Ana_In_ISO_Mode == ISO_Normal)&(Ana_Out_ISO_Mode == ISO_Normal)){
				Anal_ISO_IN_On_Set();
				Anal_ISO_OUT_On_Set();
			}

		}
	}
	else{
		if(Anal_ISO_IN_RY_Mode == 0){
			Ana_In_ISO_Mode = Ana_Check_In();
			if(Ana_In_ISO_Mode == ISO_V24){
				Anal_ISO_IN_On_Set();
			}
			else{
				if(Ana_In_ISO_Mode == ISO_Short){

				}
				else{
					Anal_ISO_IN_On_Set();
				}
			}
		}
		if(Anal_ISO_OUT_RY_Mode == 0){
			Ana_Out_ISO_Mode = Ana_Check_Out();
			if(Ana_Out_ISO_Mode == ISO_V24){
				if(Ana_In_ISO_Mode == ISO_Normal){
					Anal_ISO_OUT_On_Set();
				}
			}
			else{
				if(Ana_Out_ISO_Mode == ISO_Short){
				}
				else{
					if(Anal_ISO_OUT_RY_Mode == 0){
						Anal_ISO_OUT_On_Set();
					}
				}
			}
		}
	}


}


uint8_t Ana_Check_InA(void){

	uint8_t status;

	HAL_GPIO_WritePin(a_IN_A_TEST_r_LED_TXD_GPIO_Port, a_IN_A_TEST_r_LED_TXD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, a_IN_B_TEST_r_DIP_ADD0_Pin, GPIO_PIN_SET);

	HAL_Delay(10);

	Anal_ISO_IN_AB_ADC();

	HAL_GPIO_WritePin(a_IN_A_TEST_r_LED_TXD_GPIO_Port, a_IN_A_TEST_r_LED_TXD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, a_IN_B_TEST_r_DIP_ADD0_Pin, GPIO_PIN_SET);

	if(Anal_IN_A_ADC < V3_3_Check_Val){
		status = ISO_Open;
	}
	else{
		if(Anal_IN_B_ADC < V3_3_Check_Val){
			status = ISO_Normal;
		}
		else{
			status = ISO_Short;
		}
	}

	return status;
}

uint8_t Ana_Check_InB(void){

	uint8_t status;

	HAL_GPIO_WritePin(a_IN_A_TEST_r_LED_TXD_GPIO_Port, a_IN_A_TEST_r_LED_TXD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, a_IN_B_TEST_r_DIP_ADD0_Pin, GPIO_PIN_RESET);

	HAL_Delay(10);

	Anal_ISO_IN_AB_ADC();

	HAL_GPIO_WritePin(a_IN_A_TEST_r_LED_TXD_GPIO_Port, a_IN_A_TEST_r_LED_TXD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, a_IN_B_TEST_r_DIP_ADD0_Pin, GPIO_PIN_SET);

	if(Anal_IN_B_ADC < V3_3_Check_Val){
		status = ISO_Open;
	}
	else{
		if(Anal_IN_A_ADC < V3_3_Check_Val){
			status = ISO_Normal;
		}
		else{
			status = ISO_Short;
		}
	}

	return status;
}

uint8_t Ana_Check_In(void){

	uint8_t status;
	uint8_t status_A, status_B;

	Anal_ISO_IN_ADC();

	if( Anal_IN_ADC < V24_Check_Val){
		status_A = Ana_Check_InA();
		status_B = Ana_Check_InB();
		if((status_A == ISO_Normal) & (status_B == ISO_Normal)){
			status = ISO_Normal;
		}
		else if((status_A == ISO_Open) | (status_B == ISO_Open)){
			status = ISO_Open;
		}
		else if ((status_A == ISO_Short) | (status_B == ISO_Short)){
			status = ISO_Short;
		}
	}
	else{
		status = ISO_V24;
	}

	return status;
}

uint8_t Ana_Check_OutA(void){

	uint8_t status;

	HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

	HAL_Delay(10);

	Anal_ISO_OUT_AB_ADC();

	HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

	if(Anal_OUT_A_ADC < V3_3_Check_Val){
		status = ISO_Open;
	}
	else{
		if(Anal_OUT_B_ADC < V3_3_Check_Val){
			status = ISO_Normal;
		}
		else{
			status = ISO_Short;
		}
	}

	return status;
}

uint8_t Ana_Check_OutB(void){

	uint8_t status;

	HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_RESET);

	HAL_Delay(10);

	Anal_ISO_OUT_AB_ADC();

	HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);

	if(Anal_OUT_B_ADC < V3_3_Check_Val){
		status = ISO_Open;
	}
	else{
		if(Anal_OUT_A_ADC < V3_3_Check_Val){
			status = ISO_Normal;
		}
		else{
			status = ISO_Short;
		}
	}

	return status;
}

uint8_t Ana_Check_Out(void){

	uint8_t status;
	uint8_t status_A, status_B;

	Anal_ISO_OUT_ADC();

	if( Anal_OUT_ADC < V24_Check_Val){
		status_A = Ana_Check_OutA();
		status_B = Ana_Check_OutB();
		if((status_A == ISO_Normal) & (status_B == ISO_Normal)){
			status = ISO_Normal;
		}
		else if((status_A == ISO_Open) | (status_B == ISO_Open)){
			status = ISO_Open;
		}
		else if ((status_A == ISO_Short) | (status_B == ISO_Short)){
			status = ISO_Short;
		}
	}
	else{
		status = ISO_V24;
	}

	return status;
}

