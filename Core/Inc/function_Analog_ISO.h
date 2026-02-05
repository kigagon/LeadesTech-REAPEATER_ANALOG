/*
 * function_Analog_ISO.h
 *
 *  Created on: Jun 9, 2025
 *      Author: ltlab2
 */

#ifndef INC_FUNCTION_ANALOG_ISO_H_
#define INC_FUNCTION_ANALOG_ISO_H_



#endif /* INC_FUNCTION_ANALOG_ISO_H_ */

#define comp_24V_val	10
#define comp_Short_val	3.7
#define comp_Open_val	1.5

#define comp_Dela_val	0.3

#define In_A_Test_Port	0
#define In_B_Test_Port	1
#define Out_A_Test_Port	2
#define Out_B_Test_Port	3

#define Test_Port_Off	0
#define Test_Port_On	1

#define On_Off_Num 1

#define Relay_Waite_ms 5
#define Relay_Set_ms 5

#define Relay_Hold_ms 5

extern float COM_ADC_Volt;
extern float IN_ADC_Volt;
extern float OUT_ADC_Volt;

extern float OUT_A_ADC_Volt;
extern float OUT_B_ADC_Volt;

extern float IN_A_ADC_Volt;
extern float IN_B_ADC_Volt;

extern float IN_A_On_A_ADC_Volt;
extern float IN_A_On_B_ADC_Volt;

extern float IN_B_On_A_ADC_Volt;
extern float IN_B_On_B_ADC_Volt;

extern float OUT_A_On_A_ADC_Volt;
extern float OUT_A_On_B_ADC_Volt;

extern float OUT_B_On_A_ADC_Volt;
extern float OUT_B_On_B_ADC_Volt;

extern float IN_A_On_Dela_ADC_Volt;
extern float IN_B_On_Dela_ADC_Volt;
extern float OUT_A_On_Dela_ADC_Volt;
extern float OUT_B_On_Dela_ADC_Volt;

extern float Init_IN_ADC_Volt;
extern float Init_OUT_ADC_Volt;
extern uint8_t Init_Anal_ISO_IN_RY_Mode;
extern uint8_t Init_Anal_ISO_OUT_RY_Mode;

void Check_ISO_Start(void);
void Check_ISO(void);
void Check_ISO_det(void);
float Read_Com_ADC_Volt(void);
float Analog_Read_Com_ADC_Volt_Avr(int Num, int Hold_Time);
float Read_IN_ADC_Volt(void);
float Analog_Read_IN_ADC_Volt_Avr(int Num, int Hold_Time);
float Read_Out_ADC_Volt(void);
float Analog_Read_Out_ADC_Volt_Avr(int Num, int Hold_Time);
float Read_OUT_A_ADC_Volt(void);
float Read_OUT_B_ADC_Volt(void);
float Read_IN_A_ADC_Volt(void);
float Read_IN_B_ADC_Volt(void);

void Set_Test_3V(int Test_Port, int OnOff);

void Anal_ISO_IN_Off_Set_Num(int Num);
void Anal_ISO_IN_On_Set_Num(int Num);
void Anal_ISO_OUT_Off_Set_Num(int Num);
void Anal_ISO_OUT_On_Set_Num(int Num);

void Analog_ISO_IN_On_M_Set(int mset);
void Analog_ISO_IN_Off_M_Set(int mset);
void Analog_ISO_OUT_On_M_Set(int mset);
void Analog_ISO_OUT_Off_M_Set(int mset);
