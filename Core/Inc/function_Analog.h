/*
 * function_Analog.h
 *
 *  Created on: Apr 13, 2025
 *      Author: root
 */

#ifndef INC_FUNCTION_ANALOG_H_
#define INC_FUNCTION_ANALOG_H_

#endif /* INC_FUNCTION_ANALOG_H_ */

extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;
extern OPAMP_HandleTypeDef hopamp1;
//extern INA219_t ina219;
extern I2C_HandleTypeDef hi2c1;

extern uint16_t Ana_Temp_Sen;
extern uint8_t Ana_Temp_Sen_Com, Ana_Temp_Sen_Com_Tmp;
extern float temperature_calibrationVal;

void Run_Analoge_Mode(void);
extern void ANAL_RED_LED(uint8_t OnOff);
void ANAL_BLUE_LED(uint8_t OnOff);
void ANAL_IN_Set(void);
void ANAL_IN_ReSet(void);
void ANAL_OUT_Set(void);
void ANAL_OUT_ReSet(void);
void Check_Uart_Rx(void);
extern void Read_Analog_ADC(void);

void Anal_ISO_IN_On_Set(void);
void Anal_ISO_IN_Off_Set(void);
void Anal_ISO_OUT_On_Set(void);
void Anal_ISO_OUT_Off_Set(void);

void Anal_ISO_24_ADC(void);
void Anal_Check_ISO(void);
void Anal_ISO_OUT_ADC(void);
void Anal_ISO_IN_ADC(void);

void Anal_ISO_IN_AB_ADC(void);
void Anal_ISO_OUT_AB_ADC(void);

uint8_t Ana_Check_In(void);
uint8_t Ana_Check_Out(void);
uint8_t Ana_Check_InA(void);
uint8_t Ana_Check_InB(void);
uint8_t Ana_Check_OutA(void);
uint8_t Ana_Check_OutB(void);
