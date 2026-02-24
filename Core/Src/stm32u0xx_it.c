/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32u0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32u0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash.h"
#include "Function_Repeater.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_lpuart2_tx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef hlpuart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SVC instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32U0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32u0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and line 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Open_Short_In_Pin);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */
//  Ch1_Open_Cnt_Sum++;
  Ch1_Open_int_Cnt_Sum++;
  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_lpuart2_tx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles TIM1 Capture Compare Interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  for(int i=0; i<4; i++){
	  if(Rep_port_Fuse_Open_Mode[i] == Fuse_Open){
		  Rep_port_Fuse_Open_Cnt[i]++;
	  }
  }

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 Global Interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 Global Interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  //200ms
  //HAL_GPIO_TogglePin(a_CON_IN_SET_r_LED_ISOERR_GPIO_Port, a_CON_IN_SET_r_LED_ISOERR_Pin);
  for(int i=0; i<4; i++){
	  //Rep_port_Charge_Time[i] = 30 - 2;
	  if(Rep_port_Charge_First_Mode[i] == Charge_Mode_First_On){
		  Rep_port_Charge_Cnt[i]++;
	  }
	  else{
		  Rep_port_Charge_Cnt[i] = 0;
	  }

	  if(Rep_port_Charge_First_Mode[i] == Charge_Mode_First_On){
		  if( (Rep_port_Charge_Cnt[i] >= (Rep_port_Charge_Time[i] * 5))
				  &  (Rep_port_Charge_Cnt[i] < ((Rep_port_Charge_Time[i] * 5) + Charge_Cnt_Max)) ){
				Rep_Set_In(i,Input_Off);
			}
		  else if((Rep_port_Charge_Cnt[i] >= ((Rep_port_Charge_Time[i] * 5) + Charge_Cnt_Max))
				  & (Rep_port_Charge_Cnt[i] < Charge_Surveillance)){
				//Rep_port_Charge_First_Mode[i] = Charge_Mode_First_Off; //Timer Count Off
				Rep_Set_In(i,Input_On);
			}
		  else if(Rep_port_Charge_Cnt[i] >= ((Rep_port_Charge_Time[i] * 5) + Charge_Cnt_Max) + Charge_Surveillance){
			  Rep_port_Charge_First_Mode[i] = Charge_Mode_First_Off; //Timer Count Off
		  }
	  }
  }

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM15 + LPTIM3 global interrupt (combined with EXTI 33).
  */
void TIM15_LPTIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM15_LPTIM3_IRQn 0 */

  /* USER CODE END TIM15_LPTIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim15);
  /* USER CODE BEGIN TIM15_LPTIM3_IRQn 1 */

  /* USER CODE END TIM15_LPTIM3_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt (combined with EXTI 26) + LPUART2 global interrupt (combined with EXTI lines 35).
  */
void USART2_LPUART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_LPUART2_IRQn 0 */

  /* USER CODE END USART2_LPUART2_IRQn 0 */
  HAL_UART_IRQHandler(&hlpuart2);
  /* USER CODE BEGIN USART2_LPUART2_IRQn 1 */

  int k;

  UART_RX_buf_tmp[UART_buf_count]= LPUART2->RDR;
  UART_buf_count++;

  if(UART_buf_count > 5)
  {
	  if((UART_RX_buf_tmp[UART_buf_count -1 ] == 0x03) &(UART_RX_buf_tmp[UART_buf_count -1 - 5 ] == 0x02) )
	  {
			for(k=0;k<6;k++)
			{
				UART_RX_buf[k] = UART_RX_buf_tmp[UART_buf_count - 6 + k];
			}
			UART_State = 0;
			UART_buf_count = 0;
			Uart_Error_Cnt = 0;
			COM_MODE = DEFFERENTIAL;
			UART_Receive_complete = 1;
			Analog_uart_Suc = 1;

			if((UART_RX_buf[1] == Analog_Address)|(UART_RX_buf[1] == 253))
			{ //UART_RX_buf[1] == 253 : Broadcast Address
				IfAddressMatched = 1;
			}
			else
			{
				IfAddressMatched = 0;
				if(UART_RX_buf[2] == 0xC9)
				{
					IfAddressMatched = 1;
				}
			}
			for(k=0;k<UART_buf_len;k++){
				UART_RX_buf_tmp[k] = 0;
			}
	  }
  }

  if(UART_buf_count >= UART_buf_len)
  {
	UART_State = 0;
	UART_buf_count = 0;
	UART_Receive_complete = 0;
  }

  if(REPEATOR_MODE == ANALOG)
  {
	  if(Uart_Error_Cnt > 30)
	  {
		UART_DIR_RX_Error = 0;
		if(Uart_DIR_Mode == 0)
		{
			Uart_DIR_Mode = 1;
		}
		else if(Uart_DIR_Mode == 1)
		{
			Uart_DIR_Mode = 0;
		}
		UART_BAUDRATE = 9600;
		Reinit_Analog_uart = 1;
		//LPUART2_UART_RE_Init(Uart_DIR_Mode, UART_BAUDRATE);
		Uart_Error_Cnt = 0;
	  }
	  Uart_Error_Cnt++;
  }
  /* USER CODE END USART2_LPUART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 (combined with EXTI 24) + LPUART1 global interrupt (combined with EXTI lines 28).
  */
void USART3_LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_LPUART1_IRQn 0 */

  /* USER CODE END USART3_LPUART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_LPUART1_IRQn 1 */



  /* USER CODE END USART3_LPUART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
