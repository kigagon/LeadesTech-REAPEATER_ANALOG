
#include "INA219.h"
#include "main.h"
#include <math.h>
#include "flash.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "function_Analog.h"
#include "function_Analog_Photo.h"

extern INA219_t ina219;
extern I2C_HandleTypeDef hi2c1;
extern uint8_t ReturnPercent;
uint8_t DAC_10multiplied;
float DAC_VALUE, DAC_VALUE_TEMP = 2.3;

int16_t Prev_vshunt = 0, Prev_shunt_I;
int16_t temp_Prev_vshunt[10] = {0};
double Increasing_LedVolt_vshunt_temp[30] = {0}, Increasing_LedVolt_vshunt[30] = {0};
double Increasing_LedVolt_Current[30] = {0};

#define MAX_ADC_SAMPLES 100
int Smoke_Avr[31];
uint16_t AMP_LED_O_Value[MAX_ADC_SAMPLES] = {0, };
uint16_t AMP_LED_X_Value[MAX_ADC_SAMPLES] = {0, };
uint16_t AMP_total_Value[MAX_ADC_SAMPLES] = {0, };
uint16_t AMP_LED_O_Value_AVG[MAX_ADC_SAMPLES] = {0, };
uint16_t AMP_LED_X_Value_AVG[MAX_ADC_SAMPLES] = {0, };
uint16_t AMP_total_Value_AVG[MAX_ADC_SAMPLES] = {0, };

double RedefinedSlope = 17.36;
int16_t Adjust_Y_Intercept_Value = 0;
uint8_t UpTo22 = 0, Above22 = 1;
uint8_t Is0xCdCmdReceived = 0;
uint8_t FinalPcntVal_Required4SlopeCalculation;
uint8_t PhotoStatus = 0x40;
float volt = 0;

/* Used In First Calibration Function */
#define cal1_addr_size 15 // firstCalibration
uint8_t cal1_adc_delta_crit = 100; // ADC 값의 현재-이전 값 차이 기준(Criteria)int16_t cal1_adc_delta[cal1_addr_size];
int32_t cal1_calc_adc[cal1_addr_size] = {0} /* 첫 번째 보정에서 계산된 ADC 값 */,
		cal1_adc_delta[cal1_addr_size] /* 계산된 ADC 값들 간의 차이 */;
uint16_t cal1_led_x_adc,
		cal1_led_o_adc_2p2_1p3[cal1_addr_size] = {0,}; /* 2.2V ~ 1.3V 범위 관련 LED O ADC 값 */

double ADCMappingTable_Below22Percent[256] =
{// y = 17.36x+200 -> 15%
        200.00, 217.36, 234.72, 252.08, 269.44, 286.80, 304.16, 321.52,
        338.88, 356.24, 373.60, 390.96, 408.32, 425.68, 443.04, 460.40,
        477.76, 495.12, 512.48, 529.84, 547.20, 564.56, 581.92, 599.28,
        616.64, 634.00, 651.36, 668.72, 686.08, 703.44, 720.80, 738.16,
        755.52, 772.88, 790.24, 807.60, 824.96, 842.32, 859.68, 877.04,
        894.40, 911.76, 929.12, 946.48, 963.84, 981.20, 998.56, 1015.92,
        1033.28, 1050.64, 1068.00, 1085.36, 1102.72, 1120.08, 1137.44, 1154.80,
        1172.16, 1189.52, 1206.88, 1224.24, 1241.60, 1258.96, 1276.32, 1293.68,
        1311.04, 1328.40, 1345.76, 1363.12, 1380.48, 1397.84, 1415.20, 1432.56,
        1449.92, 1467.28, 1484.64, 1502.00, 1519.36, 1536.72, 1554.08, 1571.44,
        1588.80, 1606.16, 1623.52, 1640.88, 1658.24, 1675.60, 1692.96, 1710.32,
        1727.68, 1745.04, 1762.40, 1779.76, 1797.12, 1814.48, 1831.84, 1849.20,
        1866.56, 1883.92, 1901.28, 1918.64, 1936.00, 1953.36, 1970.72, 1988.08,
        2005.44, 2022.80, 2040.16, 2057.52, 2074.88, 2092.24, 2109.60, 2126.96,
        2144.32, 2161.68, 2179.04, 2196.40, 2213.76, 2231.12, 2248.48, 2265.84,
        2283.20, 2300.56, 2317.92, 2335.28, 2352.64, 2370.00, 2387.36, 2404.72,
        2422.08, 2439.44, 2456.80, 2474.16, 2491.52, 2508.88, 2526.24, 2543.60,
        2560.96, 2578.32, 2595.68, 2613.04, 2630.40, 2647.76, 2665.12, 2682.48,
        2699.84, 2717.20, 2734.56, 2751.92, 2769.28, 2786.64, 2804.00/*150*/,
		2821.36, 2838.72, 2856.08, 2873.44, 2890.80, 2908.16, 2925.52, 2942.88, 2960.24,
		2977.60, 2994.96, 3012.32, 3029.68, 3047.04, 3064.40, 3081.76, 3099.12, 3116.48,
		3133.84, 3151.20, 3168.56, 3185.92, 3203.28, 3220.64, 3238.00, 3255.36, 3272.72,
		3290.08, 3307.44, 3324.80, 3342.16, 3359.52, 3376.88, 3394.24, 3411.60, 3428.96,
		3446.32, 3463.68, 3481.04, 3498.40, 3515.76, 3533.12, 3550.48, 3567.84, 3585.20,
		3602.56, 3619.92, 3637.28, 3654.64, 3672.00, 3689.36, 3706.72, 3724.08, 3741.44,
		3758.80, 3776.16, 3793.52, 3810.88, 3828.24, 3845.60, 3862.96, 3880.32, 3897.68,
		3915.04, 3932.40, 3949.76, 3967.12, 3984.48, 4001.84, 4019.20, 4036.56, 4053.92,
		4071.28, 4088.64, 4106.00, 4123.36, 4140.72, 4158.08, 4175.44, 4192.80, 4210.16,
        4227.52, 4244.88, 4262.24, 4279.60, 4296.96, 4314.32, 4331.68, 4349.04, 4366.40,
		4383.76, 4401.12, 4418.48, 4435.84, 4453.20, 4470.56, 4487.92, 4505.28, 4522.64,
		4540.00, 4557.36, 4574.72, 4592.08, 4609.44, 4626.80
    };

void AverageCurrentValues(void)
{
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	uint32_t dac_value = (uint32_t)(DAC_VALUE * 4095.0f / 3.3f);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);

    uint8_t IsInitSuccess = INA219_Init(&ina219, &hi2c1, INA219_ADDRESS);
//    vshunt = 0;
    if(IsInitSuccess == 1)
    {
    	int16_t temp_Prev_vshunt[10] = {0,}, sum = 0;

        // 션트 전압 읽기
    	for(uint8_t i=0; i<10; i++)
    	{
    		temp_Prev_vshunt[i] = (int16_t) Read16(&ina219, INA219_REG_SHUNTVOLTAGE);
    		sum += temp_Prev_vshunt[i];
    	}

    	Prev_vshunt = (int16_t)(sum / 10);
		Prev_shunt_I = (int16_t)(Prev_vshunt / 0.1); // 션트 저항이 0.1Ω인 경우
    }
	HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
}

void V_0to3_checkShuntCurrentValues(void)
{
	uint32_t dac_value = 0;

	uint8_t IsInitSuccess = INA219_Init(&ina219, &hi2c1, INA219_ADDRESS);
//   	vshunt = 0;
	while(IsInitSuccess != 1)
	{
		HAL_Delay(1);
		IsInitSuccess = INA219_Init(&ina219, &hi2c1, INA219_ADDRESS);
	}

	double sum = 0;

	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	for(uint8_t j=26; j>12; j--){
		volt = j*0.1;
		dac_value = (uint32_t)(volt * 4095.0f / 3.3f);
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
		HAL_Delay(3);

		sum = 0;
		// 션트 전압 읽기
		for(uint8_t i=0; i<5; i++)
		{
			Increasing_LedVolt_vshunt_temp[i] = INA219_ReadShuntVolage(&ina219);
			sum += Increasing_LedVolt_vshunt_temp[i];
		}
		Increasing_LedVolt_vshunt[j] = sum / 5;
		Increasing_LedVolt_Current[j] = Increasing_LedVolt_vshunt[j] / 0.1;
		HAL_Delay(1);
	}
	volt = 2;
	dac_value = (uint32_t)(volt * 4095.0f / 3.3f);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);

	HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
//	HAL_Delay(50);
}

uint16_t LED_X_ADC_VALUE(ADC_HandleTypeDef* pAdcHandle, uint8_t num)
{
    uint16_t return_adcValue = 0;

	if (HAL_ADCEx_Calibration_Start(pAdcHandle) != HAL_OK) {
		Error_Handler();
	}

	// dark current adc read
	for(uint8_t i=0; i<num; i++)
	{
		// ADC 변환 시작 및 값 읽기
		HAL_ADC_Start(pAdcHandle);
		if (HAL_ADC_PollForConversion(pAdcHandle, 10) != HAL_OK)
		{
			Error_Handler();
		}
		AMP_LED_X_Value[i] = HAL_ADC_GetValue(pAdcHandle);
		HAL_ADC_Stop(pAdcHandle);
	}

	uint32_t SumOfADC = 0;
	for(uint8_t i=0; i<num; i++)
	{
		SumOfADC += AMP_LED_X_Value[i];
	}
	return_adcValue = (uint16_t)(SumOfADC / num);

	return return_adcValue;
}

uint16_t LED_O_ADC_VALUE(ADC_HandleTypeDef* pAdcHandle, float volt, uint8_t num)
{
    uint16_t return_adcValue = 0;

	uint32_t dac_value = (uint32_t)(volt * 4095.0f / 3.0f);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_Delay(5);

	// LED ON ADC READ
	for(uint8_t i=0; i<num; i++)
	{
		// ADC 변환 시작 및 값 읽기
		HAL_ADC_Start(pAdcHandle);
		if (HAL_ADC_PollForConversion(pAdcHandle, 10) != HAL_OK)
		{
		  Error_Handler();
		}
		AMP_LED_O_Value[i] = 0;
		AMP_LED_O_Value[i] = HAL_ADC_GetValue(pAdcHandle);
		HAL_ADC_Stop(pAdcHandle);
	}
	HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);

	uint32_t SumOfADC = 0;
	for(uint8_t i=0; i<num; i++)
	{
		SumOfADC += AMP_LED_O_Value[i];
	}
	return_adcValue = (uint16_t)(SumOfADC / num);

	return return_adcValue;
}

uint16_t ReadLED_O_value_ToSetPGA(uint8_t gain, float volt, uint8_t num)
{
	uint16_t LED_O_PhotoVal_12bit = 0;

	ADC_ChannelConfTypeDef sConfig = {0};

	// OPAMP 설정 및 시작
	hopamp1.Init.Mode = OPAMP_PGA_MODE;                     // OPAMP를 내부 PGA 모드로 설정
	if(gain == 2){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_2; }
	else if(gain == 4){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_4; }
	else if(gain == 8){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_8; }
	else if(gain == 16){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16; }
	else	{ Error_Handler(); }
	hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_CONNECT_NO;  // 반전 입력 없음
	hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;   // 비반전 입력을 VINP에 연결

	if (HAL_OPAMP_DeInit(&hopamp1) != HAL_OK) { Error_Handler(); return 0;}
	if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
	{
	   Error_Handler();
	}
	if (HAL_OPAMP_Start(&hopamp1) != HAL_OK)
	{
	   Error_Handler();
	}

	// OPAMP 사용 후 ADC 변환
	sConfig.Channel = ADC_CHANNEL_7; // OPAMP 출력과 연결된 ADC 채널
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	   Error_Handler();
	}

	LED_O_PhotoVal_12bit = LED_O_ADC_VALUE(&hadc1, volt, num);
	return LED_O_PhotoVal_12bit;
}

void SetPGA(float volt, uint8_t num)
{
	uint16_t LED_O_ADC_Value = 0;
	Flash_Read_uint8(SMOKE_PGA_ADDR, &PGA);
	if(PGA == 0xFF)
	{
		PGA = 4;

		LED_O_ADC_Value = ReadLED_O_value_ToSetPGA(PGA, DAC_VALUE, 60);

		if(LED_O_ADC_Value < 250)
		{
			Flash_Erase(SMOKE_PGA_ADDR);
			Flash_Write_uint8(SMOKE_PGA_ADDR, 8);
		}
		else if((LED_O_ADC_Value > 800))
		{
			Flash_Erase(SMOKE_PGA_ADDR);
			Flash_Write_uint8(SMOKE_PGA_ADDR, 2);
		}
		else
		{
			Flash_Erase(SMOKE_PGA_ADDR);
			Flash_Write_uint8(SMOKE_PGA_ADDR, 4);
		}
		Flash_Read_uint8(SMOKE_PGA_ADDR, &PGA);

		if(PGA == 8)
		{
			LED_O_ADC_Value = ReadLED_O_value_ToSetPGA(PGA, DAC_VALUE, 30);
			if(LED_O_ADC_Value < 250)
			{
				while((DAC_VALUE >= 1.5) && LED_O_ADC_Value < 250)
				{
					DAC_VALUE -= 0.1;
					LED_O_ADC_Value = ReadLED_O_value_ToSetPGA(PGA, DAC_VALUE, 30);
				}

//				LED_O_ADC_Value = ReadLED_O_value_ToSetPGA(PGA, DAC_VALUE, 60);
//				if(LED_O_ADC_Value < 250)
//				{
//					ANAL_RED_LED(LED_ON);
//				}
			}
		}
	}
}

uint16_t Return_Ana_photo_Sen_12bit(uint8_t gain, float volt, uint8_t num)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	// OPAMP 설정 및 시작
	hopamp1.Init.Mode = OPAMP_PGA_MODE;                     // OPAMP를 내부 PGA 모드로 설정
	if(gain == 2){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_2; }
	else if(gain == 4){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_4; }
	else if(gain == 8){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_8; }
	else if(gain == 16){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16; }
	else	{ Error_Handler(); }
	hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_CONNECT_NO;  // 반전 입력 없음
	hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;   // 비반전 입력을 VINP에 연결

	if (HAL_OPAMP_DeInit(&hopamp1) != HAL_OK) { Error_Handler(); return 0;}
	if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
	{
	   Error_Handler();
	}
	if (HAL_OPAMP_Start(&hopamp1) != HAL_OK)
	{
	   Error_Handler();
	}

	// OPAMP 사용 후 ADC 변환
	sConfig.Channel = ADC_CHANNEL_7; // OPAMP 출력과 연결된 ADC 채널
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	   Error_Handler();
	}

	Ana_photo_X_Sen_12bit = LED_X_ADC_VALUE(&hadc1, num);
	Ana_photo_O_Sen_12bit = LED_O_ADC_VALUE(&hadc1, volt, num);

	// OPAMP 사용 후 중지 (선택 사항)
	if (HAL_OPAMP_Stop(&hopamp1) != HAL_OK)
	{
	   Error_Handler();
	   // 계속 진행할지, 오류를 반환할지 결정
	}

	// 최종 ADC 값 계산
	int32_t PhotoSensorVal = 0;
	PhotoSensorVal = Ana_photo_O_Sen_12bit - Ana_photo_X_Sen_12bit + Adjust_Y_Intercept_Value;
	if(PhotoSensorVal < 0)
	{
		PhotoSensorVal = 0;
	}

	// 광전식 상태값 갱신 및 최종 리턴 값
	PhotoStatus = CheckLED_IsShortOrOpen(&hadc1, PhotoSensorVal);
//	PhotoStatus = 0x40;
	if(PhotoStatus == 0x40)
	{
		return (uint16_t)PhotoSensorVal;
	}
	else
	{
		return 0;
	}
}

uint16_t ReturnToPercent_afterReadAdcValue(uint8_t gain, float volt, uint8_t num, uint8_t set)
{
	//	/* 배열 클리어 및 set, num 변수 크기 조정 START */
	if(set > sizeof(AMP_LED_X_Value_AVG)){
		set = sizeof(AMP_LED_X_Value_AVG);
	}
	for(uint8_t i=0; i<set; i++)
	{
		AMP_LED_O_Value_AVG[i] = 0;
		AMP_LED_X_Value_AVG[i] = 0;
		AMP_total_Value_AVG[i] = 0;
	}

	if(num > sizeof(AMP_LED_X_Value)){
		num = sizeof(AMP_LED_X_Value);
	}

	/* ADC, OPAMP CONFIG START */
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
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_19CYCLES_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_19CYCLES_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
	  Error_Handler();
	}

	for(uint8_t j=0; j<set; j++)
	{
		// num만큼의 배열 클리어
		for(uint8_t i=0; i<num; i++)
		{
			AMP_LED_X_Value[i] = 0;
			AMP_LED_O_Value[i] = 0;
			AMP_total_Value[i] = 0;
		}

		// OPAMP 설정 및 시작
		hopamp1.Init.Mode = OPAMP_PGA_MODE;                     // OPAMP를 내부 PGA 모드로 설정
		if(gain == 2){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_2; }
		else if(gain == 4){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_4; }
		else if(gain == 8){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_8; }
		else if(gain == 16){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16; }
		else	{ Error_Handler(); }
		hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_CONNECT_NO;  // 반전 입력 없음
		hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;   // 비반전 입력을 VINP에 연결

		if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
		{
		   Error_Handler();
		}
		if (HAL_OPAMP_Start(&hopamp1) != HAL_OK)
		{
		   Error_Handler();
		}

		// OPAMP 사용 후 ADC 변환
		sConfig.Channel = ADC_CHANNEL_7; // OPAMP 출력과 연결된 ADC 채널
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
		   Error_Handler();
		}
		/* ADC, OPAMP CONFIG END */

	/* Set만큼 ADC START */
		/* LED OFF 상태에서의 ADC */

	   // ADC 보정 (필요 시)
		if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
			Error_Handler();
		}

		// dark current adc read
		for(uint8_t i=0; i<num; i++)
		{
			// ADC 변환 시작 및 값 읽기
			HAL_ADC_Start(&hadc1);
			if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
			{
				Error_Handler();
			}
			AMP_LED_X_Value[i] = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
		}

		uint32_t tempVal_ADC_Calculation = 0;
		// num만큼 읽은 값들의 평균을 set만큼 AVG 배열에 저장
		for(uint8_t i=0; i<num; i++)
		{
			tempVal_ADC_Calculation += AMP_LED_X_Value[i];
		}
		AMP_LED_X_Value_AVG[j] = (uint16_t)(tempVal_ADC_Calculation / num);
		tempVal_ADC_Calculation = 0;

		/* LED ON 상태에서의 ADC */
		// DAC 기능을 사용하여 LED를 ON
		uint32_t dac_value = (uint32_t)(volt * 4095.0f / 3.0f);
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
		HAL_Delay(5);
//		PhotoStatus = CheckLED_IsShortOrOpen();
//
//		if(PhotoStatus != 0x40)
//		{
//			HAL_OPAMP_Stop(&hopamp1);
//			HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
//			/* 아래 코드 실행 x, for(uint8_t j=0; j<set; j++)의 다음 인덱스를 실행*/
//			continue;
//		}

		// LED ON ADC READ
		for(uint8_t i=0; i<num; i++)
		{
			// ADC 변환 시작 및 값 읽기
			HAL_ADC_Start(&hadc1);
			if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
			{
			  Error_Handler();
			}
			AMP_LED_O_Value[i] = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
		}
		HAL_OPAMP_Stop(&hopamp1);
		HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);

		// num만큼 읽은 값들의 평균을 set만큼 AVG 배열에 저장
		for(uint8_t i=0; i<num; i++)
		{
			tempVal_ADC_Calculation += AMP_LED_O_Value[i];
			if(AMP_LED_O_Value[i] >= AMP_LED_X_Value[i])
			{
				AMP_total_Value[i] = AMP_LED_O_Value[i] - AMP_LED_X_Value[i];
			}
			else
			{
				AMP_total_Value[i] = 0;
			}
		}
		AMP_LED_O_Value_AVG[j] = (uint16_t)(tempVal_ADC_Calculation / num);
		if(AMP_LED_O_Value_AVG[j] >= AMP_LED_X_Value_AVG[j])
		{
			AMP_total_Value_AVG[j] = AMP_LED_O_Value_AVG[j] - AMP_LED_X_Value_AVG[j];
		}
		else
		{
			AMP_total_Value_AVG[j] = 0;
		}
	}

	Ana_photo_X_Sen_12bit = 0, Ana_photo_O_Sen_12bit = 0;
	uint32_t tempVal_AVG_LED_O = 0, tempVal_AVG_LED_X = 0;

	for(uint8_t i=0; i<set; i++)
	{
		tempVal_AVG_LED_X += AMP_LED_X_Value_AVG[i];
		tempVal_AVG_LED_O += AMP_LED_O_Value_AVG[i];
	}
	Ana_photo_X_Sen_12bit = (uint16_t)(tempVal_AVG_LED_X / set);
	Ana_photo_O_Sen_12bit = (uint16_t)(tempVal_AVG_LED_O / set);

//	if(Ana_photo_X_Sen_12bit == Ana_photo_O_Sen_12bit)
//	{
//		HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
//		uint32_t dac_value = (uint32_t)(volt * 4095.0f / 3.3f);
//		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
//		HAL_Delay(5);
//		PhotoStatus = CheckLED_IsShortOrOpen();
//
//		HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
//
//		if(PhotoStatus != 0x40)
//		{
//
//		}
//	}

	int32_t PhotoSensorVal = 0;
	PhotoSensorVal = Ana_photo_O_Sen_12bit - Ana_photo_X_Sen_12bit + Adjust_Y_Intercept_Value;
	if(PhotoSensorVal < 0)
	{
		PhotoSensorVal = 0;
	}
	return (uint16_t)PhotoSensorVal;
}

uint8_t MatchAdcValueToPercentValue(const double *arr, uint16_t ADCValue)
{
	uint8_t low = 0;
	uint8_t high = 255;

    if (ADCValue <= arr[0]) {
        return 0; // 최저값 아래일 경우 0 반환
    }
    else if(ADCValue == 0)
    {
    	return 0;
    }
    else if (ADCValue >= arr[255]) {
        return 255; // 최대값 초과일 경우 255 반환
    }

	while (low <= high) {
		uint8_t mid = (low + high) / 2;

		if (arr[mid] == ADCValue) {
			return mid;
		} else if (arr[mid] < ADCValue) {
			low = mid + 1;
		} else {
			high = mid - 1;
		}
	}

	uint8_t closestIndex = high;

    double diffHigh = fabs(arr[high] - ADCValue);
    double diffLow = fabs(arr[low] - ADCValue);

	if (diffLow < diffHigh) {
		closestIndex = low;
	}

	return closestIndex;
}

void RedefineSmokeTableWithADC(uint8_t ReferenceArr, double slope)
{
    if(ReferenceArr == UpTo22)
    {
    	for (int i = 0; i < 256; i++) {
    	    double value = 200 + (slope * i);
    	    if (value > 65535.0)
    	        value = 65535.0;
    	    else if (value < 0.0)
    	        value = 0.0;

    	    ADCMappingTable_Below22Percent[i] = value;  // double 배열에 바로 저장
    	}
    }
//    else if(ReferenceArr == PGA2_Above20)
//    {
//    	for (int i = 0; i < 256; i++) {
//    	    double value = (double)Adjust_Y_Intercept_Value + (slope * (double)i);
//    	    if (value > 65535.0)
//    	        value = 65535.0;
//    	    else if (value < 0.0)
//    	        value = 0.0;
//
//            ADCMappingTable_Above22Percent[i] = value;
//        }
//    }
}

uint8_t firstCalibration(uint8_t pga)
{/*
 * 0. 사용자가 영점 조정 버튼 누름
	1. firstCalibration 함수 실행.
	이 함수는 플래시메모리에 시료 별 DAC 값을 다르게 저장하기 위한 함수임. 이 함수의 코드 진행 순서는 아래와 같음.
	1-1. DAC를 켜지 않은 상태에서 ADC 30번 진행한 것의 평균값 1개 리턴
	1-2. DAC를 켠 상태에서 ADC 30번 진행한 것의 평균값 1개 리턴하는 작업을 DAC 2.2부터 0.1씩 감소하며 1.3까지 총 10번 진행 -> 2.2부터 1.0까지
	1-3. DAC 각 구간에서의 ADC 30회 평균값 - DAC를 안켜고 ADC한 값 각각을 크기가 10인 배열에 저장
	1-4-1. (LED O- LED X)ADC한 값을 순회하며 현재 값 - 이전 값의 차이가 처음으로 100 이상인 구간이 있다면
			해당 구간에서 멈추고 그 때 인덱스값과 페어인 ADC 전압값을 플래시메모리에 저장
	1-4-2. 만약 100 이상인 구간이 없다면 비교값을 50으로 줄여 다시 탐색,
			현재 값과 이전 값의 차이가 50이상인 DAC 전압값이 존재한다면 그 값을 플래시메모리에 저장,
			만약 존재하지 않는다면 RedLED ON
	*/

	/*
	 * #define cal1_addr_size 10 // firstCalibration
		uint8_t cal1_adc_delta_crit = 100; // ADC 값의 현재-이전 값 차이 기준(Criteria)
		uint16_t cal1_led_x_adc,
		cal1_led_o_adc_2p2_1p3[cal1_addr_size] = {0,}, /\* 2.2V ~ 1.3V 범위 관련 LED O ADC 값 *\/
		cal1_calc_adc[cal1_addr_size] = {0};       // 첫 번째 보정에서 계산된 ADC 값
	 */

	uint8_t pga_setting_changeTo8 = 0;

	ADC_ChannelConfTypeDef sConfig = {0};

	// OPAMP 설정 및 시작
	hopamp1.Init.Mode = OPAMP_PGA_MODE;
	// OPAMP를 내부 PGA 모드로 설정
//	if(pga == 2){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_2; }
//	else
	if(pga == 4){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_4; }
	else if(pga == 8){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_8; }
//	else if(pga == 16){hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16; }
	else	{ Error_Handler(); }
	hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_CONNECT_NO;  // 반전 입력 없음
	hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;   // 비반전 입력을 VINP에 연결

	if (HAL_OPAMP_DeInit(&hopamp1) != HAL_OK) { Error_Handler();}
	if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
	{
	   Error_Handler();
	}
	if (HAL_OPAMP_Start(&hopamp1) != HAL_OK)
	{
	   Error_Handler();
	}

	// OPAMP 사용 후 ADC 변환
	sConfig.Channel = ADC_CHANNEL_7; // OPAMP 출력과 연결된 ADC 채널
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	   Error_Handler();
	}

	/* 1-1. DAC를 켜지 않은 상태에서 ADC 30번 진행한 것의 평균값 1개 리턴 */
	cal1_led_x_adc = LED_X_ADC_VALUE(&hadc1, 30);


	/* 1-2. DAC를 켠 상태에서 ADC 30번 진행한 것의 평균값 1개 리턴하는 작업을 DAC 2.2부터 0.1씩 감소하며 1.0까지 총 13번 진행 */
	float DacVolt = 2.2;
	for(uint8_t i=0; i<13; i++)
	{
		cal1_led_o_adc_2p2_1p3[i] = LED_O_ADC_VALUE(&hadc1, DacVolt, 30);
		DacVolt -= 0.1;
	}


	/* 1-3. (DAC 각 구간에서의 ADC 30회 평균값 - DAC를 안켜고 ADC한 값) 각각 계산된 값을 크기가 10인 배열에 저장 */
	for(uint8_t i=0; i<13; i++)
	{
		cal1_calc_adc[i] = cal1_led_o_adc_2p2_1p3[i] - cal1_led_x_adc;
	}


	/* 1-4-1. (LED O- LED X)ADC한 값을 순회하며 현재 값 - 이전 값의 차이가 처음으로 100 이상인 구간이 있다면
			해당 구간에서 멈추고 그 때 인덱스값과 페어인 ADC 전압값을 플래시메모리에 저장

		1-4-2. 만약 100 이상인 구간이 없다면 비교값을 50으로 줄여 다시 탐색,
			현재 값과 이전 값의 차이가 50이상인 DAC 전압값이 존재한다면 그 값을 플래시메모리에 저장,
			만약 존재하지 않는다면 RedLED ON  													*/

	uint8_t i=1, breakFlag = 0;
	cal1_adc_delta[0] = cal1_calc_adc[0] - 0;
	for(i=1; i<13; i++)
	{
		cal1_adc_delta[i] = cal1_calc_adc[i] - cal1_calc_adc[i-1];
	}
	for(i=0; i<13; i++)
	{
		if(cal1_adc_delta[i] >= 100)
		{
			breakFlag = 1;
			pga_setting_changeTo8 = 0;
			break;
		}
	}
	if(breakFlag == 1)
	{
		saveDACvalueInFlash(i, pga);
		breakFlag = 0;
	}
	else
	{
		for(i=0; i<13; i++)
		{
			if(cal1_adc_delta[i] >= 50)
			{
				breakFlag = 1;
				break;
			}
		}
		if(breakFlag == 1)
		{
			saveDACvalueInFlash(i, pga);
			breakFlag = 0;
		}
		else
		{
			pga_setting_changeTo8 = 1;
			if((pga == 8) && (pga_setting_changeTo8 == 1))
			{
				ANAL_RED_LED(LED_ON);
			}
		}
	}

	// OPAMP 사용 후 중지 (선택 사항)
	if (HAL_OPAMP_Stop(&hopamp1) != HAL_OK)
	{
	   Error_Handler();
	   // 계속 진행할지, 오류를 반환할지 결정
	}
	return pga_setting_changeTo8;
}

void ZeroAdjustment(void)
{ // (LED o ADC 값 - LED x ADC 값)이 200이 되도록 y절편 보정값을 계산
	/* 시료 별 편차를 맞추기 위함
	 * 최종 adc 값 계산해서, 그게 200이 되도록 해야 함. => 최종 adc 값 + y절편 보정값 = 200
	 * "y절편 보정값"이라는 변수에 값을 저장하는데,
	 * 200보다 작으면 그만큼 더해줘야 하니까 200-보정값
	 * 200보다 크면 200에 맞춰야 하니까 200-보정값 해서 <- 이 때의 최종 보정 값은 음수가 되는 것이 맞음
	 */

	Adjust_Y_Intercept_Value = 200 - (Ana_photo_O_Sen_12bit - Ana_photo_X_Sen_12bit);

    Flash_Erase(Adjust_Y_Intercept_ADDR);
    Flash_Write_int16(Adjust_Y_Intercept_ADDR, Adjust_Y_Intercept_Value);
    Flash_Read_int16(Adjust_Y_Intercept_ADDR, &Adjust_Y_Intercept_Value);
}

void saveDACvalueInFlash(uint8_t index, uint8_t pga)
{
	ANAL_RED_LED(LED_ON);

	uint8_t DAC_multiplied_10 = 0;
	switch(index)
	{
	case 0:
		DAC_multiplied_10 = 22;
		break;
	case 1:
		DAC_multiplied_10 = 21;
		break;
	case 2:
		DAC_multiplied_10 = 20;
		break;
	case 3:
		DAC_multiplied_10 = 19;
		break;
	case 4:
		DAC_multiplied_10 = 18;
		break;
	case 5:
		DAC_multiplied_10 = 17;
		break;
	case 6:
		DAC_multiplied_10 = 16;
		break;
	case 7:
		DAC_multiplied_10 = 15;
		break;
	case 8:
		DAC_multiplied_10 = 14;
		break;
	case 9:
		DAC_multiplied_10 = 13;
		break;
	case 10:
		DAC_multiplied_10 = 12;
		break;
	case 11:
		DAC_multiplied_10 = 11;
		break;
	case 12:
		DAC_multiplied_10 = 10;
		break;
	}

	Flash_Erase(SMOKE_PGA_ADDR);
	Flash_Write_uint8(SMOKE_PGA_ADDR, pga);
	Flash_Read_uint8(SMOKE_PGA_ADDR, &PGA);

	Flash_Erase(SMOKE_DAC_Multiplied_10_ADDR);
	Flash_Write_uint8(SMOKE_DAC_Multiplied_10_ADDR, DAC_multiplied_10);
	Flash_Read_uint8(SMOKE_DAC_Multiplied_10_ADDR, &DAC_10multiplied);
	DAC_VALUE = DAC_10multiplied * 0.1;

	Ana_photo_X_Sen_12bit = LED_X_ADC_VALUE(&hadc1, 30);
	Ana_photo_O_Sen_12bit = LED_O_ADC_VALUE(&hadc1, DAC_VALUE, 30);

	ZeroAdjustment();

	ANAL_RED_LED(LED_OFF);
}

void AdjustmentCmdPrcessing(uint8_t cmd)
{
	if(cmd == 0xCA)
	{
//		ZeroAdjustment();
		uint8_t pga_setting_changeTo8 = 0;
		pga_setting_changeTo8 = firstCalibration(4);
		if(pga_setting_changeTo8 == 0)
		{
			return;
		}
		else
		{
			pga_setting_changeTo8 = firstCalibration(8);
			return;
		}
	}

	Read_Analog_ADC();

	if((FinalPcntVal_Required4SlopeCalculation <= 250) && (FinalPcntVal_Required4SlopeCalculation >= -150))
	{
		if(cmd == 0xCD) // 5, 10, 15, 20% button
		{
			Is0xCdCmdReceived = 1;
			UserAdjustedPercent = 0;

			if(UART_RX_buf[3] == 0x10)
			{
				UserAdjustedPercent = 100;
			}
			else if(UART_RX_buf[3] == 0x13)
			{
				UserAdjustedPercent = 130;
			}
			else
			{
				UserAdjustedPercent = UART_RX_buf[3];
			}
		}
		else
		{
			if((Is0xCdCmdReceived == 1) && (UserAdjustedPercent != 0))
			{
				if(cmd == 0xCB) // xx.0 +,-
				{
					if (UART_RX_buf[3] == 0x01) // -
					{
						UserAdjustedPercent -= 10;
					}
					else if (UART_RX_buf[3] == 0x02) // +
					{
						UserAdjustedPercent += 10;
					}
				}
				else if(cmd == 0xCC) // 0.x +,-
				{
					if (UART_RX_buf[3] == 0x01) // -
					{
						UserAdjustedPercent -= 1;
					}
					else if (UART_RX_buf[3] == 0x02) // +
					{
						UserAdjustedPercent += 1;
					}
				}
			}
		}

		if(UserAdjustedPercent < 0)
		{ // 농도 보정 값 무효화
			UserAdjustedPercent = 0;
		}


		// 분자
		if(Ana_photo_Sen_12bit >= 200)
		{
			photoADC_Required4SlopeCalculation = Ana_photo_Sen_12bit - 200;
		}
		else
		{
			photoADC_Required4SlopeCalculation = 0;
		}

		Flash_Erase(photoADC_Required4SlopeCalculation_ADDR);
		Flash_Write_uint16(photoADC_Required4SlopeCalculation_ADDR, photoADC_Required4SlopeCalculation);
		Flash_Read_uint16(photoADC_Required4SlopeCalculation_ADDR, &photoADC_Required4SlopeCalculation);


		// 분모
		if(UserAdjustedPercent != 0)
		{
			FinalPcntVal_Required4SlopeCalculation = UserAdjustedPercent;
		}
		else
		{
			FinalPcntVal_Required4SlopeCalculation = 0;
		}

		Flash_Erase(FinalPcntVal_Required4SlopeCalculation_ADDR);
		Flash_Write_uint8(FinalPcntVal_Required4SlopeCalculation_ADDR, FinalPcntVal_Required4SlopeCalculation);
		Flash_Read_uint8(FinalPcntVal_Required4SlopeCalculation_ADDR, &FinalPcntVal_Required4SlopeCalculation);

		// 기울기 계산
		if((photoADC_Required4SlopeCalculation == 0))
		{
			RedefinedSlope = 17.36;
		}
		else if(FinalPcntVal_Required4SlopeCalculation == 0)
		{
			RedefinedSlope = 17.36;
		}
		else
		{
			RedefinedSlope = (double)photoADC_Required4SlopeCalculation / (double)FinalPcntVal_Required4SlopeCalculation;
		}
		RedefineSmokeTableWithADC(UpTo22, RedefinedSlope);
	}
}
