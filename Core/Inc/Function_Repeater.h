/*
 * Function_Repeater.h
 *
 *  Created on: Apr 13, 2025
 *      Author: root
 */

#ifndef INC_FUNCTION_REPEATER_H_
#define INC_FUNCTION_REPEATER_H_



#endif /* INC_FUNCTION_REPEATER_H_ */

#define Output_Off	0
#define Output_On	1

#define Output_Port1	0
#define Output_Port2	1
#define Output_Port3	2
#define Output_Port4	3

#define err_ck_Cnt	10

#define Out_Short_Val	12
#define In_Short_Val	12

#define Out_Open_Val	3
#define In_Open_Val		3

#define Comp_24V_short	12

#define	Charge_Mode_First_Ready		0
#define	Charge_Mode_First_Off		1
#define	Charge_Mode_First_On		2

#define Init_Set 	0
#define Out_Set 	1

#define Comp_24V	8

#define Charge_Surveillance	300 // 60*5 , Timer2 : 200ms

extern uint8_t Rep_port_Charge_First_Mode[4];
extern uint8_t Rep_Out_Mode[4];
extern uint8_t Rep_ISO_In_Open;
extern float Rep_IN_B_ADC;

extern uint8_t Pre_IN_Short, Pre_OUT_Short;

extern int Ch1_Open_Cnt_Sum;
extern int Ch1_Open_int_Cnt_Sum;

extern uint8_t Ch1_Out_Open;
extern uint8_t Ch1_Out_Off_Cnt;

extern uint16_t Ch1_On_Save_Sum;
extern uint16_t Ch1_Off_Save_Sum;

void Run_Repeater_Mode(void);
void Rep_TX_LED(uint8_t OnOff);
void Rep_ISO_LED(uint8_t OnOff);
void Rep_Set_Out(int Port , int Status);
void Rep_Set_In(int Port , int Status);
void Set_Out_Port(void);
void Check_Charge_Mode(void);
void Check_Fuse_Open(void);
void Rep_Read_Port(void);

void Repeater_ISO_OUT_On_Set(void);
void Repeater_ISO_OUT_Off_Set(void);

void Rep_Check_ISO(void);
void Rep_Out_Check_ISO(void);

void Com_C5(void);
void Com_C6(void);
void Com_F0(void);

float Repeater_Rep_IN_B_ADC(void);
float Repeater_Rep_OUT_B_ADC(void);

uint8_t Rep_Check_Out_Short(void);

float Rep_Check_Out_Short_Avr(int Num, int Hold_Time);


float Repeater_Rep_SIG24_ADC(void);
float Repeater_Rep_SIG24_ADC_Avr(int Num, int Hold_Time);

float Repeater_Rep_OUT_ADC(void);
float Repeater_Rep_OUT_ADC_Avr(int Num, int Hold_Time);

void Repeater_ISO_OUT_Off_M_Set(int mset);
void Repeater_ISO_OUT_On_M_Set(int mset);

void Check_Ch1(void);
