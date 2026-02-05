/*
 * INA219.c
 *
 *  Created on: Dec 30, 2020
 *       Author: Piotr Smolen <komuch@gmail.com>
 */

#include "main.h"
#include "INA219.h"
#include <stdio.h>
#include <stdlib.h>
#include "function_Analog.h"
#include "function_Analog_Photo.h"

extern I2C_HandleTypeDef hi2c1;
extern int16_t Prev_shunt_A;
INA219_t ina219;

uint16_t diff = 0;
uint16_t ina219_calibrationValue;
int16_t ina219_currentDivider_mA;
int16_t ina219_powerMultiplier_mW;
uint8_t IsInitSuccess;

int16_t raw;
float shunt_mV = 0, shunt_A = 0;
uint16_t Bus_V = 0;

int32_t compareVal;

uint16_t Read16(INA219_t *ina219, uint8_t Register)
{
	uint8_t Value[2];

	HAL_I2C_Mem_Read(ina219->ina219_i2c, (INA219_ADDRESS<<1), Register, 1, Value, 2, 1000);

	return ((Value[0] << 8) | Value[1]);
}


void Write16(INA219_t *ina219, uint8_t Register, uint16_t Value)
{
	uint8_t addr[2];
	addr[0] = (Value >> 8) & 0xff;  // upper byte
	addr[1] = (Value >> 0) & 0xff; // lower byte
	HAL_I2C_Mem_Write(ina219->ina219_i2c, (INA219_ADDRESS<<1), Register, 1, (uint8_t*)addr, 2, 1000);
}

uint16_t INA219_ReadBusVoltage(INA219_t *ina219)
{
	uint16_t result = Read16(ina219, INA219_REG_BUSVOLTAGE);

	return ((result >> 3  ) * 4);

}

float INA219_ReadShuntVolage(INA219_t *ina219)
{
	int16_t result = (int16_t)Read16(ina219, INA219_REG_SHUNTVOLTAGE);

	return (result * 0.01f);
}

int16_t INA219_ReadCurrent(INA219_t *ina219)
{
	int16_t result = INA219_ReadCurrent_raw(ina219);

	return (result / ina219_currentDivider_mA );
}

void INA219_Reset(INA219_t *ina219)
{
	Write16(ina219, INA219_REG_CONFIG, INA219_CONFIG_RESET);
	HAL_Delay(1);
}

void INA219_setCalibration(INA219_t *ina219, uint16_t CalibrationData)
{
	Write16(ina219, INA219_REG_CALIBRATION, CalibrationData);
}

uint16_t INA219_getConfig(INA219_t *ina219)
{
	uint16_t result = Read16(ina219, INA219_REG_CONFIG);
	return result;
}

void INA219_setConfig(INA219_t *ina219, uint16_t Config)
{
	Write16(ina219, INA219_REG_CONFIG, Config);
}

void INA219_setCalibration_32V_2A(INA219_t *ina219)
{
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
	             INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
	             INA219_CONFIG_SADCRES_12BIT_1S_532US |
	             INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	ina219_calibrationValue = 4096;
	ina219_currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
	ina219_powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)

	INA219_setCalibration(ina219, ina219_calibrationValue);
	INA219_setConfig(ina219, config);
}

void INA219_setCalibration_32V_1A(INA219_t *ina219)
{
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
//						INA219_CONFIG_GAIN_1_40MV |
						 INA219_CONFIG_GAIN_2_80MV |
//						INA219_CONFIG_GAIN_4_160MV |
//	                    INA219_CONFIG_GAIN_8_320MV |
						INA219_CONFIG_BADCRES_12BIT |
	                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
	                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	ina219_calibrationValue = 10240;
	ina219_currentDivider_mA = 25;    // Current LSB = 40uA per bit (1000/40 = 25)
	ina219_powerMultiplier_mW = 0.8f; // Power LSB = 800uW per bit

	INA219_setCalibration(ina219, ina219_calibrationValue);
	INA219_setConfig(ina219, config);
}

void INA219_setCalibration_16V_400mA(INA219_t *ina219)
{
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
	                    INA219_CONFIG_GAIN_1_40MV |
						INA219_CONFIG_BADCRES_12BIT |
	                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
	                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	ina219_calibrationValue = 8192;
	ina219_currentDivider_mA = 20;    // Current LSB = 50uA per bit (1000/50 = 20)
	ina219_powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit

	INA219_setCalibration(ina219, ina219_calibrationValue);
	INA219_setConfig(ina219, config);
}

void INA219_setPowerMode(INA219_t *ina219, uint8_t Mode)
{
	uint16_t config = INA219_getConfig(ina219);

	switch (Mode) {
		case INA219_CONFIG_MODE_POWERDOWN:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_POWERDOWN & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(ina219, config);
			break;

		case INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(ina219, config);
			break;

		case INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(ina219, config);
			break;

		case INA219_CONFIG_MODE_ADCOFF:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_ADCOFF & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(ina219, config);
			break;
	}
}

uint8_t INA219_Init(INA219_t *ina219, I2C_HandleTypeDef *i2c, uint8_t Address)
{
	ina219->ina219_i2c = i2c;
	ina219->Address = Address;

	ina219_currentDivider_mA = 0;
	ina219_powerMultiplier_mW = 0;

	uint8_t ina219_isReady = HAL_I2C_IsDeviceReady(i2c, (Address << 1), 3, 2);

	if(ina219_isReady == HAL_OK)
	{

		INA219_Reset(ina219);
//		INA219_setCalibration_32V_2A(ina219);
//		INA219_setCalibration_32V_1A(ina219); // 1A 이하에서 정밀하게 측정
		INA219_setCalibration_16V_400mA(ina219);
//		uint16_t config = INA219_getConfig(ina219);
//		uint16_t calibration = Read16(ina219, INA219_REG_CALIBRATION);
		return 1;
	}

	else
	{
		return 0;
	}
}

int16_t INA219_ReadCurrent_raw(INA219_t *ina219)
{
	int16_t result = Read16(ina219, INA219_REG_CURRENT);

	return (result);
}

uint8_t CheckLED_IsShortOrOpen(ADC_HandleTypeDef* pAdcHandle, int32_t finalSensorValue)
{
	/* LED 단선 및 단락을 체크하는 함수.
	 * 0. DAC 값이 잡힌 시료들(1차 캘리브레이션을 통과한 시료들)에만 적용된다는 전제 하에 진행함.
	 * 1. ADC를 하다가 값이 50 이하로 떨어졌을 때 문제가 발생했다고 판단함.
	 * 2. 원래 켜던 DAC 값(매개변수로 전달받은 volt 값)으로 LED를 켜고 전류를 쟀을 때,
	 *    이미 단선 또는 단락이 확정이므로 전류값이 음수이면 단선, 양수이면 단락으로 판정
	 */

    uint8_t IsInitSuccess = 0;
    while(!IsInitSuccess)
    {
    	IsInitSuccess = INA219_Init(&ina219, &hi2c1, INA219_ADDRESS);
    }

	uint32_t dac_value = (uint32_t)(PGA * 4095.0f / 3.0f);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_Delay(5);

    raw = (int16_t)Read16(&ina219, INA219_REG_SHUNTVOLTAGE);

	HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);

	if(raw > 600)
	{
		return 0x42; // short circuit
	}
	else if(raw < 5)
	{
		return 0x41; // open circuit
	}
	else
	{
		return 0x40;
	}

/*	if(finalSensorValue > 100)
	{
		return 0x40;
	}
	else
	{
	    uint8_t IsInitSuccess = 0;
	    while(!IsInitSuccess)
	    {
	    	IsInitSuccess = INA219_Init(&ina219, &hi2c1, INA219_ADDRESS);
	    }

		uint32_t dac_value = (uint32_t)(PGA * 4095.0f / 3.0f);
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
		HAL_Delay(5);

	    raw = (int16_t)Read16(&ina219, INA219_REG_SHUNTVOLTAGE);

		HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);

		if(raw < 400)
		{
			return 0x41; // open circuit
		}
		else
		{
			return 0x42;
		}


        // 션트 전압 읽기
    	shunt_mV = INA219_ReadShuntVolage(&ina219);
    	Bus_V = INA219_ReadBusVoltage(&ina219);

	    shunt_A = 0;
        // 전류 계산 (셔트 전압 / 션트 저항 값)
        shunt_A = (float)(shunt_mV / 0.1f) * 100; // 션트 저항이 0.1Ω인 경우

        if(shunt_A <= 0) { return 0x41; // open circuit }
        else{ return 0x42; }

	}*/
}

/*uint8_t CheckLED_IsShortOrOpen(ADC_HandleTypeDef* pAdcHandle, float volt)
{
	uint32_t TempAdcValue = 0;
    uint16_t return_LED_X_adcValue = 0, return_LED_O_adcValue = 0;

	if (HAL_ADCEx_Calibration_Start(pAdcHandle) != HAL_OK) {
		Error_Handler();
	}

	// dark current adc read
	for(uint8_t i=0; i<10; i++)
	{
		// ADC 변환 시작 및 값 읽기
		HAL_ADC_Start(pAdcHandle);
		if (HAL_ADC_PollForConversion(pAdcHandle, 10) != HAL_OK)
		{
			Error_Handler();
		}
		TempAdcValue += HAL_ADC_GetValue(pAdcHandle);
		HAL_ADC_Stop(pAdcHandle);
	}
	return_LED_X_adcValue = (uint16_t)(TempAdcValue / 10);

	uint32_t dac_value = (uint32_t)(volt * 4095.0f / 3.0f);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_Delay(5);

	TempAdcValue = 0;
	// LED ON ADC READ
	for(uint8_t i=0; i<10; i++)
	{
		// ADC 변환 시작 및 값 읽기
		HAL_ADC_Start(pAdcHandle);
		if (HAL_ADC_PollForConversion(pAdcHandle, 10) != HAL_OK)
		{
		  Error_Handler();
		}
		TempAdcValue = HAL_ADC_GetValue(pAdcHandle);
		HAL_ADC_Stop(pAdcHandle);
	}
	HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
	return_LED_O_adcValue = (uint16_t)(TempAdcValue / 10);

	compareVal = return_LED_O_adcValue - return_LED_X_adcValue;

	if(compareVal > 20) {return 0x40;}
	else
	{
	    uint8_t IsInitSuccess = INA219_Init(&ina219, &hi2c1, INA219_ADDRESS);

	    shunt_A = 0;
	    if(IsInitSuccess == 1)
	    {
	        // 션트 전압 읽기
	    	shunt_mV = (int16_t) Read16(&ina219, INA219_REG_SHUNTVOLTAGE);

	        // 전류 계산 (셔트 전압 / 션트 저항 값)
	        shunt_A = (int16_t)(shunt_mV / 0.1); // 션트 저항이 0.1Ω인 경우

	        if(shunt_A < 0)
	        {
	        	return 0x41; // open circuit
	        }
	        else if(shunt_A > 30)
	        {
	        	return 0x42;
	        }
	    }
	    else
	    {
	    	return 0;
	    }
	}
	return 0;
}*/
