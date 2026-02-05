/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Reset_GPIO(uint8_t REPEATOR_MODE);
uint32_t Read_ADC_Channel(uint32_t channel);
void Check_Com(void);
void Com_C0(void);
void Com_C1(void);
void Com_C2(void);
void Com_C4(void);
void Com_B0(void);
void Com_B1(void);
void Repeater_Out_Init(void);
void LPUART2_UART_RE_Init(uint8_t Side, int BAUDRATE);
extern int16_t Prev_shunt_mA;
extern float shunt_mA;
extern uint16_t diff;
//#define DAC_VALUE 2 // 1.95 // 2.05 // 2.3
extern float DAC_VALUE;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define a_IN_A_TEST_r_LED_TXD_Pin GPIO_PIN_13
#define a_IN_A_TEST_r_LED_TXD_GPIO_Port GPIOC
#define a_CON_IN_SET_r_LED_ISOERR_Pin GPIO_PIN_14
#define a_CON_IN_SET_r_LED_ISOERR_GPIO_Port GPIOC
#define a_CON_IN_RSET_r_OUT_A_TEST_Pin GPIO_PIN_15
#define a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port GPIOC
#define a_OUT_A_TEST_r_CON_OUT_SET_Pin GPIO_PIN_0
#define a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port GPIOF
#define a_OUT_B_TEST_r_IN_A_TEST_Pin GPIO_PIN_1
#define a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port GPIOF
#define a_IN_A_ADC_r_CON_OUT_RSET_Pin GPIO_PIN_0
#define a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port GPIOC
#define a_COM_ADC_r_SIG24_ADC_Pin GPIO_PIN_1
#define a_COM_ADC_r_SIG24_ADC_GPIO_Port GPIOC
#define a_OUT_B_ADC_r_OUT_A_ADC_Pin GPIO_PIN_2
#define a_OUT_B_ADC_r_OUT_A_ADC_GPIO_Port GPIOC
#define a_TEMP_SEN_r_OUT_B_ADC_Pin GPIO_PIN_3
#define a_TEMP_SEN_r_OUT_B_ADC_GPIO_Port GPIOC
#define a_SMOKE_ADC_OP_r_OUT_ADC_Pin GPIO_PIN_0
#define a_SMOKE_ADC_OP_r_OUT_ADC_GPIO_Port GPIOA
#define a_OPAMP_IN_M_r_DC_24V_Pin GPIO_PIN_1
#define a_OPAMP_IN_M_r_DC_24V_GPIO_Port GPIOA
#define a_OUT_A_ADC_r_OUT4_Pin GPIO_PIN_2
#define a_OUT_A_ADC_r_OUT4_GPIO_Port GPIOA
#define a_op1_vout_r_OUT3_Pin GPIO_PIN_3
#define a_op1_vout_r_OUT3_GPIO_Port GPIOA
#define a_IR_LED_DAC_r_OUT2_Pin GPIO_PIN_4
#define a_IR_LED_DAC_r_OUT2_GPIO_Port GPIOA
#define a_OUT_ADC_r_OUT1_Pin GPIO_PIN_5
#define a_OUT_ADC_r_OUT1_GPIO_Port GPIOA
#define a_IN_B_ADC_r_OCP4_Pin GPIO_PIN_6
#define a_IN_B_ADC_r_OCP4_GPIO_Port GPIOA
#define a_IN_ADC_r_OCP3_Pin GPIO_PIN_7
#define a_IN_ADC_r_OCP3_GPIO_Port GPIOA
#define r_OCP2_Pin GPIO_PIN_4
#define r_OCP2_GPIO_Port GPIOC
#define a_CON_OUT_RSET_r_OCP1_Pin GPIO_PIN_5
#define a_CON_OUT_RSET_r_OCP1_GPIO_Port GPIOC
#define OPEN_SHORT_CON_Pin GPIO_PIN_0
#define OPEN_SHORT_CON_GPIO_Port GPIOB
#define Open_Short_In_Pin GPIO_PIN_1
#define Open_Short_In_GPIO_Port GPIOB
#define Open_Short_In_EXTI_IRQn EXTI0_1_IRQn
#define r_IN4_SIG_Pin GPIO_PIN_2
#define r_IN4_SIG_GPIO_Port GPIOB
#define a_LED_RED_r_IN3_SIG_Pin GPIO_PIN_12
#define a_LED_RED_r_IN3_SIG_GPIO_Port GPIOB
#define a_LED_BLUE_r_IN2_SIG_Pin GPIO_PIN_13
#define a_LED_BLUE_r_IN2_SIG_GPIO_Port GPIOB
#define MODE_ANALOG_Pin GPIO_PIN_14
#define MODE_ANALOG_GPIO_Port GPIOB
#define a_DIP_ADD7_r_IN1_SIG_Pin GPIO_PIN_15
#define a_DIP_ADD7_r_IN1_SIG_GPIO_Port GPIOB
#define a_DIP_ADD6_r_CH1_SW_Pin GPIO_PIN_6
#define a_DIP_ADD6_r_CH1_SW_GPIO_Port GPIOC
#define a_DIP_ADD5_r_CH2_SW_Pin GPIO_PIN_7
#define a_DIP_ADD5_r_CH2_SW_GPIO_Port GPIOC
#define a_DIP_ADD4_r_CH3_SW_Pin GPIO_PIN_8
#define a_DIP_ADD4_r_CH3_SW_GPIO_Port GPIOC
#define a_DIP_ADD3_r_CH4_SW_Pin GPIO_PIN_9
#define a_DIP_ADD3_r_CH4_SW_GPIO_Port GPIOC
#define a_DIP_ADD2_r_DIP_ADD5_Pin GPIO_PIN_8
#define a_DIP_ADD2_r_DIP_ADD5_GPIO_Port GPIOA
#define a_DIP_ADD1_r_DIP_ADD6_Pin GPIO_PIN_9
#define a_DIP_ADD1_r_DIP_ADD6_GPIO_Port GPIOA
#define a_DIP_ADD0_r_DIP_ADD7_Pin GPIO_PIN_10
#define a_DIP_ADD0_r_DIP_ADD7_GPIO_Port GPIOA
#define MODE_REPEATOR_Pin GPIO_PIN_11
#define MODE_REPEATOR_GPIO_Port GPIOA
#define MODE_ISO_Pin GPIO_PIN_12
#define MODE_ISO_GPIO_Port GPIOA
#define a_SPI_CS_r_DIP_ADD4_Pin GPIO_PIN_15
#define a_SPI_CS_r_DIP_ADD4_GPIO_Port GPIOA
#define a_SCK_r_DIP_ADD3_Pin GPIO_PIN_10
#define a_SCK_r_DIP_ADD3_GPIO_Port GPIOC
#define a_MISO_DIP_ADD2_Pin GPIO_PIN_11
#define a_MISO_DIP_ADD2_GPIO_Port GPIOC
#define a_MOSI_r_DIP_ADD1_Pin GPIO_PIN_12
#define a_MOSI_r_DIP_ADD1_GPIO_Port GPIOC
#define a_IN_B_TEST_r_DIP_ADD0_Pin GPIO_PIN_2
#define a_IN_B_TEST_r_DIP_ADD0_GPIO_Port GPIOD
#define a_CON_OUT_SET_r_DISCON1_Pin GPIO_PIN_4
#define a_CON_OUT_SET_r_DISCON1_GPIO_Port GPIOB
#define a_N_MODE_r_DISCON2_Pin GPIO_PIN_5
#define a_N_MODE_r_DISCON2_GPIO_Port GPIOB
#define a_SCL_r_DISCON3_Pin GPIO_PIN_6
#define a_SCL_r_DISCON3_GPIO_Port GPIOB
#define a_SDA_r_DISCON4_Pin GPIO_PIN_7
#define a_SDA_r_DISCON4_GPIO_Port GPIOB
#define BOOT_R_N_MODE_Pin GPIO_PIN_3
#define BOOT_R_N_MODE_GPIO_Port GPIOF

/* USER CODE BEGIN Private defines */

extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;
extern OPAMP_HandleTypeDef hopamp1;

#define Input_Address		0
#define All_Add_Address		253
#define Isolation_Address	254
#define Test_Address		250

#define LED_ON 		1
#define LED_OFF 	0

#define REPEATOR 	1
#define ANALOG 		0
extern uint8_t REPEATOR_MODE;

#define OOK 			1
#define DEFFERENTIAL	0
extern uint8_t COM_MODE;

#define Non_ISO		0
#define ISO			1
extern uint8_t ISO_MODE;

#define Non_ADDRESS		0
#define READ_ADDRESS	1
extern uint8_t ADDRESS_MODES;

#define Normal		1
#define Open_Short	0
#define Open_Short_err	0
#define Open_Short_ok	1
#define OS_ok		0
#define OS_Open		1
#define OS_Short	2
extern uint8_t REPEATOR_OS_MODES;

#define SMOKE		1
#define TEMPERATURE	0
extern uint8_t ANALOG_MODE;

#define Port_High	1
#define Port_Low	0
extern uint8_t Test_MODE;

//array to store addresses
extern uint8_t CCU_Address_tmp[8] ;
extern uint8_t CCU_Address ;

//array to store addresses
extern uint8_t Analog_Address_tmp[8];
extern uint8_t Analog_Address;

#define UART_buf_len 32

extern UART_HandleTypeDef hlpuart2;
extern UART_HandleTypeDef huart3;

extern uint8_t UART_TX_buf[UART_buf_len] ;
extern uint8_t UART_RX_buf[UART_buf_len] ;
extern uint8_t UART_RX_buf_tmp[UART_buf_len] ;
extern uint8_t UART_State;					// Define reception start state
extern int UART_buf_count;					// Save the length of received data after starting reception
extern int UART_buf_count_tmp;				// Variable to store the length of the received data before initializing it when reception is completed
extern uint8_t UART_Receive_complete;			// Variable indicating that reception has been completed
extern uint8_t UART_DMA_CNT;					// Time check variable after DMA start
extern int UART_DMA_ERR_CNT;				//DMA error count

extern uint8_t IfAddressMatched;

extern uint8_t Rep_Input_value[4];
extern uint8_t Rep_Pre_Input_value[4];
extern uint8_t Rep_Output_value[4];
extern uint8_t Rep_port_open[4];
extern uint8_t Rep_port_Fuse_Open[4];
extern uint8_t Rep_V24_value;

extern int Uart_Error_Cnt, Uart_RX_Ch_Mode;

#define	Rep_port_Fuse_Open_Cnt_Max 	2
extern uint8_t Rep_port_Fuse_Open_Cnt[4];

#define	Fuse_Short	0
#define	Fuse_Open	1
extern uint8_t Rep_port_Fuse_Open_Mode[4];

#define Input_Off	0
#define Input_On	1

#define Input_Port1	0
#define Input_Port2	1
#define Input_Port3	2
#define Input_Port4	3

#define Output_Off	0
#define Output_On	1

#define Output_Port1	0
#define Output_Port2	1
#define Output_Port3	2
#define Output_Port4	3

#define	Charge_Wait_Cnt 	1
#define	Charge_Cnt_Max 		7

#define	Charge_Set_On				1
#define	Charge_Set_Off				0

#define Fire_Off		0
#define Fire_On			1

#define	Charge_Mode_On					1
#define	Charge_Mode_Off 				0

extern uint8_t Rep_port_Charge_Setting[4];
extern uint16_t Rep_port_Charge_Cnt[4];
extern uint8_t Rep_port_Charge_Mode[4];
extern uint8_t Rep_port_Charge_Time[4];
extern uint8_t Rep_All_port_Charge_Mode;


#define ISO_Normal	0	// defult
#define ISO_Error	1	// error
#define ISO_Short	2	// short
#define ISO_Open	3	// open ->short ground
#define ISO_V24		4	// 24V


extern uint8_t Rep_Out_ISO_Pre_Mode;
extern uint8_t Rep_Out_ISO_Mode;
extern uint8_t Rep_Out_ISO_Error_Mode;

extern uint8_t UART_DIR_RX_Error, Uart_DIR_Mode;


#define Relay_Off	0	//
#define Relay_On	1	//

extern uint8_t Rep_Out_Relay_Mode;

extern uint8_t Rep_ISO_St;
extern uint8_t Rep_ISO_In_St;
extern uint8_t Rep_ISO_Out_St;
extern uint8_t Rep_ISO_In_Short;
extern uint8_t Rep_ISO_Out_Open;
extern uint8_t Rep_ISO_Out_Short;

extern uint8_t Ana_Out_ISO_Pre_Mode;
extern uint8_t Ana_Out_ISO_Mode;
extern uint8_t Ana_In_ISO_Pre_Mode;
extern uint8_t Ana_In_ISO_Mode;

extern int Ana_Temp_Sen;
extern uint8_t Ana_Temp_Sen_Com, Ana_Temp_Sen_Com_Tmp;
extern float temperature_calibrationVal;
extern uint8_t Ana_Led_Com_mode, Ana_Led_Fire_Mode;

extern int Ana_Smoke_Sen, Ana_Smoke_OpAmp;
extern uint16_t Ana_photo_O_Sen_12bit, Ana_photo_X_Sen_12bit, Ana_photo_Sen_12bit;
extern uint16_t photoADC_Required4SlopeCalculation;
extern int16_t UserAdjustedPercent;
extern uint8_t returnPercent;
extern float DAC_Volt;
extern uint8_t CurrentlyDisplayedPercent, DisplayedPercent_Required4SlopeCalculation;
extern uint8_t PhotoStatus;

#define V24_Check_Val	10
#define V3_3_Check_Val	2.5

extern float Rep_SIG24_ADC;
extern float Rep_OUT_A_ADC;
extern float Rep_OUT_B_ADC;
extern float Rep_OUT_ADC;

#define Rep_SIG24_ADC_tmp_num 10
extern float Rep_SIG24_ADC_sum;
extern float Rep_SIG24_ADC_tmp[Rep_SIG24_ADC_tmp_num];

extern float Anal_SIG24_ADC;
extern float Anal_OUT_A_ADC;
extern float Anal_OUT_B_ADC;
extern float Anal_OUT_ADC;
extern float Anal_IN_A_ADC;
extern float Anal_IN_B_ADC;
extern float Anal_IN_ADC;

#define Rel_Off	0
#define Rel_On	1

extern uint8_t Anal_ISO_IN_RY_Mode;
extern uint8_t Anal_ISO_OUT_RY_Mode;

extern	uint8_t Anal_Loop_Mode;
extern uint8_t Rep_Loop_Mode;
#define Loop_In		0
#define Loop_Out	1
#define Loop_Out_In_Short	2
#define Loop_Out_In_Open	3
#define Loop_Out_Short_In_Open	4

extern uint8_t Set_Uart_Dir_Flash_Wr;
extern int UART_BAUDRATE;
extern uint32_t UART_DIR_ADDR;
extern int Reinit_Analog_uart;
extern int Analog_uart_Suc;

extern uint8_t Loop_Enable ;

extern uint8_t Ana_Com_Led_mode , Fire_Mode;




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
