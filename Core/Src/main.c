/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  * 광전식, 정온식 센서 코드 다 바뀐 거의 최종코드임.
  * 광전식은 AdjustmentCmdPrcessing 쪽. 사용자가 감도조정할 때 기울기 계산하는 수식이 바꼈고,
  * 정온식은 형식온도표에서 작동시간기준표, 작동시험온도와 관련된 테스트 진행했는데 곱해주는 보정변수 값이 좀 낮았는지 시간 내에 도달하지 못해서,
  * temperature_calibrationVal의 값을 방수형 1.2 -> 1.25로 변경 및 일반형은 1.1 -> 1.2로 변경하였음
  * 방수형 정온식인 경우를 처리하지 못했어서 송신패킷만들때 문제가 있었음, 수정하였음.
  *
  * 250527_ANALOG_SmokeAndTemp_Test 프로그램 복붙해서 수정하는 코드임
  * ADC 수정 완료했음
  * 정온식 방수형 코드 추가했음 (25.05.29)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include "Function_Repeater.h"
#include "function_Analog.h"
#include "function_Analog_Photo.h"
#include "INA219.h"
#include "flash.h"

#include "Compile_Data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_lpuart2_tx;
DMA_HandleTypeDef hdma_usart3_tx;

OPAMP_HandleTypeDef hopamp1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */

uint8_t REPEATOR_MODE;
uint8_t COM_MODE;
uint8_t ISO_MODE;

uint8_t ADDRESS_MODES;
uint8_t REPEATOR_OS_MODES;
uint8_t ANALOG_MODE;
uint8_t Test_MODE;

//array to store addresses
uint8_t CCU_Address_tmp[8] ;
uint8_t CCU_Address ;

//array to store addresses
uint8_t Analog_Address_tmp[8];
uint8_t Analog_Address;

uint8_t Rep_Input_value[4];
uint8_t Rep_Pre_Input_value[4];
uint8_t Rep_Output_value[4];
uint8_t Rep_port_open[4];
uint8_t Rep_port_Fuse_Open[4];
uint8_t Rep_V24_value;

uint8_t Rep_port_Fuse_Open_Cnt[4];
uint8_t Rep_port_Fuse_Open_Mode[4];

uint8_t Rep_port_Charge_Setting[4];
uint16_t Rep_port_Charge_Cnt[4];
uint8_t Rep_port_Charge_Mode[4];
uint8_t Rep_port_Charge_Time[4];
//uint8_t Rep_All_port_Charge_Mode;

uint8_t Rep_Out_Relay_Mode;

uint8_t Rep_Out_ISO_Pre_Mode;
uint8_t Rep_Out_ISO_Mode;
uint8_t Rep_Out_ISO_Error_Mode;

uint8_t Rep_ISO_St;
uint8_t Rep_ISO_In_St;
uint8_t Rep_ISO_Out_St;
uint8_t Rep_ISO_In_Short;
uint8_t Rep_ISO_Out_Open;
uint8_t Rep_ISO_Out_Short;

uint8_t Ana_Out_ISO_Pre_Mode;
uint8_t Ana_Out_ISO_Mode;
uint8_t Ana_In_ISO_Pre_Mode;
uint8_t Ana_In_ISO_Mode;

uint8_t Ana_Led_Com_mode , Ana_Led_Fire_Mode;
uint8_t Ana_Com_Led_mode , Fire_Mode;

int Ana_Smoke_Sen, Ana_Smoke_OpAmp;
uint8_t scaledValue = 0;

float DAC_Volt;

//Define UI uart receiving array

uint8_t UART_TX_buf[UART_buf_len] ;
uint8_t UART_RX_buf[UART_buf_len] ;
uint8_t UART_RX_buf_tmp[UART_buf_len] ;
uint8_t UART_State;					// Define reception start state
int UART_buf_count;					// Save the length of received data after starting reception
int UART_buf_count_tmp;				// Variable to store the length of the received data before initializing it when reception is completed
uint8_t UART_Receive_complete;			// Variable indicating that reception has been completed
uint8_t UART_DMA_CNT;					// Time check variable after DMA start
int UART_DMA_ERR_CNT;				//DMA error count

uint8_t IfAddressMatched;

uint8_t UART_DIR_RX_Error, Uart_DIR_Mode = 0;

int Uart_Error_Cnt, Uart_RX_Ch_Mode;

float Rep_SIG24_ADC;
float Rep_OUT_A_ADC;
float Rep_OUT_B_ADC;
float Rep_OUT_ADC;

float Rep_SIG24_ADC_sum;
float Rep_SIG24_ADC_tmp[Rep_SIG24_ADC_tmp_num];

float Anal_SIG24_ADC;
float Anal_OUT_A_ADC;
float Anal_OUT_B_ADC;
float Anal_OUT_ADC;
float Anal_IN_A_ADC;
float Anal_IN_B_ADC;
float Anal_IN_ADC;

uint8_t Anal_ISO_IN_RY_Mode;
uint8_t Anal_ISO_OUT_RY_Mode;

uint8_t Anal_Loop_Mode;
uint8_t Rep_Loop_Mode;

uint8_t Set_Uart_Dir_Flash_Wr;

int UART_BAUDRATE = 9600;
int Reinit_Analog_uart = 0;
int Analog_uart_Suc = 0;

uint32_t UART_DIR_ADDR = ADDR_FLASH_PAGE_58;

uint8_t Loop_Enable = 0 ;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_LPUART2_UART_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_OPAMP1_Init();
  MX_USART3_UART_Init();
  MX_LPUART2_UART_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */

  /*************Read Mode*******************/
  if(HAL_GPIO_ReadPin(MODE_REPEATOR_GPIO_Port, MODE_REPEATOR_Pin) == GPIO_PIN_RESET){
	  REPEATOR_MODE = ANALOG;
  }
  else{
	  REPEATOR_MODE = REPEATOR;
  }

  if(HAL_GPIO_ReadPin(MODE_ISO_GPIO_Port, MODE_ISO_Pin) == GPIO_PIN_RESET){
	  ISO_MODE = Non_ISO;
  }
  else{
	  ISO_MODE = ISO;
  }

  if(HAL_GPIO_ReadPin(MODE_ANALOG_GPIO_Port, MODE_ANALOG_Pin) == GPIO_PIN_RESET){
	  if(REPEATOR_MODE == ANALOG){
		  ANALOG_MODE = TEMPERATURE;
	  }
	  else if(REPEATOR_MODE == REPEATOR){
		  REPEATOR_OS_MODES = Open_Short;
	  }
  }
  else{
	  if(REPEATOR_MODE == ANALOG){
		  ANALOG_MODE = SMOKE;
	  }
	  else if(REPEATOR_MODE == REPEATOR){
		  REPEATOR_OS_MODES = Normal;
	  }
  }

//  Flash_Read_uint8(UART_DIR_ADDR, &Uart_DIR_Mode);
//  if(Uart_DIR_Mode == 0xff){
//	Flash_Erase(UART_DIR_ADDR);
//	Flash_Write_uint8(UART_DIR_ADDR, 0);
//	Uart_DIR_Mode = 0;
//  }
//  LPUART2_UART_RE_Init(0, 9600);

  Reset_GPIO(REPEATOR_MODE);

  // Timer1: 1 second cycle
  // Timer2: 0.5 second cycle
  // Timer3: 0.2 second cycle
  // Timer15: 0.1 second cycle

  HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim15,TIM_CHANNEL_1);

  /* huart2 RX Interrupt  Enable */
  /* Process Unlocked */
  __HAL_UNLOCK(&hlpuart2);
  /* Enable the UART Parity Error Interrupt */
  __HAL_UART_ENABLE_IT(&hlpuart2, UART_IT_PE);
  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(&hlpuart2, UART_IT_ERR);
  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&hlpuart2, UART_IT_RXNE);

  HAL_GPIO_WritePin(a_SPI_CS_r_DIP_ADD4_GPIO_Port, a_SPI_CS_r_DIP_ADD4_Pin, GPIO_PIN_SET);

  Compile_Date();

  ////////////////////////////

  if(REPEATOR_MODE == REPEATOR){
	  Run_Repeater_Mode();
  }
  else if(REPEATOR_MODE == ANALOG){
	  Ana_Led_Com_mode = LED_ON;
//	  Ana_Led_Fire_Mode = Fire_Off;
//	  while(!INA219_Init(&ina219, &hi2c1, INA219_ADDRESS))
//	  {
//
//	  }

	  //INA219_setPowerMode(&ina219, INA219_CONFIG_MODE_ADCOFF);
//
//	  vbus = INA219_ReadBusVoltage(&ina219);
//	  vshunt = INA219_ReadShuntVolage(&ina219);
//	  current = INA219_ReadCurrent(&ina219);

	  Run_Analoge_Mode();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
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
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000509;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART2_UART_Init(void)
{

  /* USER CODE BEGIN LPUART2_Init 0 */

  /* USER CODE END LPUART2_Init 0 */

  /* USER CODE BEGIN LPUART2_Init 1 */

  /* USER CODE END LPUART2_Init 1 */
  hlpuart2.Instance = LPUART2;
  hlpuart2.Init.BaudRate = 9600;
  hlpuart2.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart2.Init.StopBits = UART_STOPBITS_1;
  hlpuart2.Init.Parity = UART_PARITY_NONE;
  hlpuart2.Init.Mode = UART_MODE_TX_RX;
  hlpuart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* USER CODE BEGIN LPUART2_Init 2 */

  /* USER CODE END LPUART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
  huart3.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerSupplyRange = OPAMP_POWERSUPPLY_LOW;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALPOWER_NORMALSPEED;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_2;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 199;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, a_IN_A_TEST_r_LED_TXD_Pin|a_CON_IN_SET_r_LED_ISOERR_Pin|a_CON_IN_RSET_r_OUT_A_TEST_Pin|r_OCP2_Pin
                          |a_CON_OUT_RSET_r_OCP1_Pin|a_DIP_ADD6_r_CH1_SW_Pin|a_DIP_ADD5_r_CH2_SW_Pin|a_DIP_ADD4_r_CH3_SW_Pin
                          |a_DIP_ADD3_r_CH4_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, a_OUT_A_TEST_r_CON_OUT_SET_Pin|a_OUT_B_TEST_r_IN_A_TEST_Pin|BOOT_R_N_MODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OPEN_SHORT_CON_GPIO_Port, OPEN_SHORT_CON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, r_IN4_SIG_Pin|a_DIP_ADD7_r_IN1_SIG_Pin|a_CON_OUT_SET_r_DISCON1_Pin|a_N_MODE_r_DISCON2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, a_DIP_ADD2_r_DIP_ADD5_Pin|a_DIP_ADD1_r_DIP_ADD6_Pin|a_SPI_CS_r_DIP_ADD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, a_IN_B_TEST_r_DIP_ADD0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : a_IN_A_TEST_r_LED_TXD_Pin a_CON_IN_SET_r_LED_ISOERR_Pin a_CON_IN_RSET_r_OUT_A_TEST_Pin r_OCP2_Pin
                           a_CON_OUT_RSET_r_OCP1_Pin a_DIP_ADD6_r_CH1_SW_Pin a_DIP_ADD5_r_CH2_SW_Pin a_DIP_ADD4_r_CH3_SW_Pin
                           a_DIP_ADD3_r_CH4_SW_Pin */
  GPIO_InitStruct.Pin = a_IN_A_TEST_r_LED_TXD_Pin|a_CON_IN_SET_r_LED_ISOERR_Pin|a_CON_IN_RSET_r_OUT_A_TEST_Pin|r_OCP2_Pin
                          |a_CON_OUT_RSET_r_OCP1_Pin|a_DIP_ADD6_r_CH1_SW_Pin|a_DIP_ADD5_r_CH2_SW_Pin|a_DIP_ADD4_r_CH3_SW_Pin
                          |a_DIP_ADD3_r_CH4_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : a_OUT_A_TEST_r_CON_OUT_SET_Pin a_OUT_B_TEST_r_IN_A_TEST_Pin BOOT_R_N_MODE_Pin */
  GPIO_InitStruct.Pin = a_OUT_A_TEST_r_CON_OUT_SET_Pin|a_OUT_B_TEST_r_IN_A_TEST_Pin|BOOT_R_N_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : OPEN_SHORT_CON_Pin r_IN4_SIG_Pin a_DIP_ADD7_r_IN1_SIG_Pin a_CON_OUT_SET_r_DISCON1_Pin
                           a_N_MODE_r_DISCON2_Pin */
  GPIO_InitStruct.Pin = OPEN_SHORT_CON_Pin|r_IN4_SIG_Pin|a_DIP_ADD7_r_IN1_SIG_Pin|a_CON_OUT_SET_r_DISCON1_Pin
                          |a_N_MODE_r_DISCON2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Open_Short_In_Pin */
  GPIO_InitStruct.Pin = Open_Short_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Open_Short_In_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : a_LED_RED_r_IN3_SIG_Pin a_LED_BLUE_r_IN2_SIG_Pin */
  GPIO_InitStruct.Pin = a_LED_RED_r_IN3_SIG_Pin|a_LED_BLUE_r_IN2_SIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MODE_ANALOG_Pin */
  GPIO_InitStruct.Pin = MODE_ANALOG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MODE_ANALOG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : a_DIP_ADD2_r_DIP_ADD5_Pin a_DIP_ADD1_r_DIP_ADD6_Pin a_SPI_CS_r_DIP_ADD4_Pin */
  GPIO_InitStruct.Pin = a_DIP_ADD2_r_DIP_ADD5_Pin|a_DIP_ADD1_r_DIP_ADD6_Pin|a_SPI_CS_r_DIP_ADD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : a_DIP_ADD0_r_DIP_ADD7_Pin MODE_REPEATOR_Pin MODE_ISO_Pin */
  GPIO_InitStruct.Pin = a_DIP_ADD0_r_DIP_ADD7_Pin|MODE_REPEATOR_Pin|MODE_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : a_IN_B_TEST_r_DIP_ADD0_Pin */
  GPIO_InitStruct.Pin = a_IN_B_TEST_r_DIP_ADD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void Reset_GPIO(uint8_t REPEATOR_MODE){

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/////////////////////////////////////////////////////////////////////
	//// a_OUT_A_TEST_r_CON_OUT_SET_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_OUT_A_TEST_r_CON_OUT_SET_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		  GPIO_InitStruct.Pin = a_OUT_A_TEST_r_CON_OUT_SET_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		  HAL_GPIO_Init(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, &GPIO_InitStruct);
		  HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_SET);
	  }

	//// a_IN_A_ADC_r_CON_OUT_RSET_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_IN_A_ADC_r_CON_OUT_RSET_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		  GPIO_InitStruct.Pin = a_IN_A_ADC_r_CON_OUT_RSET_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, &GPIO_InitStruct);
	  }


	if(REPEATOR_MODE == REPEATOR){

		HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, a_IN_A_ADC_r_CON_OUT_RSET_Pin, GPIO_PIN_SET);

		HAL_Delay(1);

		HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, a_IN_A_ADC_r_CON_OUT_RSET_Pin, GPIO_PIN_RESET);
	}
	/////////////////////////////////////////////////////////////////////


	//// a_IN_A_TEST_r_LED_TXD_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_IN_A_TEST_r_LED_TXD_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_IN_A_TEST_r_LED_TXD_GPIO_Port, &GPIO_InitStruct);
		Rep_TX_LED(LED_OFF);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		  GPIO_InitStruct.Pin = a_IN_A_TEST_r_LED_TXD_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		  HAL_GPIO_Init(a_IN_A_TEST_r_LED_TXD_GPIO_Port, &GPIO_InitStruct);
		  HAL_GPIO_WritePin(a_IN_A_TEST_r_LED_TXD_GPIO_Port, a_IN_A_TEST_r_LED_TXD_Pin, GPIO_PIN_SET);
	  }

	//// a_CON_IN_SET_r_LED_ISOERR_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_CON_IN_SET_r_LED_ISOERR_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_CON_IN_SET_r_LED_ISOERR_GPIO_Port, &GPIO_InitStruct);
		Rep_ISO_LED(LED_OFF);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		  GPIO_InitStruct.Pin = a_CON_IN_SET_r_LED_ISOERR_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		  HAL_GPIO_Init(a_CON_IN_SET_r_LED_ISOERR_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_CON_IN_RSET_r_OUT_A_TEST_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_CON_IN_RSET_r_OUT_A_TEST_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_RESET);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		  GPIO_InitStruct.Pin = a_CON_IN_RSET_r_OUT_A_TEST_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		  HAL_GPIO_Init(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_OUT_A_TEST_r_CON_OUT_SET_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_OUT_A_TEST_r_CON_OUT_SET_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		  GPIO_InitStruct.Pin = a_OUT_A_TEST_r_CON_OUT_SET_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		  HAL_GPIO_Init(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, &GPIO_InitStruct);
		  HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_SET);
	  }

	//// a_OUT_B_TEST_r_IN_A_TEST_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_OUT_B_TEST_r_IN_A_TEST_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_RESET);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		  GPIO_InitStruct.Pin = a_OUT_B_TEST_r_IN_A_TEST_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		  HAL_GPIO_Init(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, &GPIO_InitStruct);
		  HAL_GPIO_WritePin(a_OUT_B_TEST_r_IN_A_TEST_GPIO_Port, a_OUT_B_TEST_r_IN_A_TEST_Pin, GPIO_PIN_SET);
	  }

	//// a_IN_A_ADC_r_CON_OUT_RSET_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_IN_A_ADC_r_CON_OUT_RSET_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		  GPIO_InitStruct.Pin = a_IN_A_ADC_r_CON_OUT_RSET_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_COM_ADC_r_SIG24_ADC_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_COM_ADC_r_SIG24_ADC_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_COM_ADC_r_SIG24_ADC_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_COM_ADC_r_SIG24_ADC_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_COM_ADC_r_SIG24_ADC_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_OUT_B_ADC_r_IN_B_ADC_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_OUT_B_ADC_r_OUT_A_ADC_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_OUT_B_ADC_r_OUT_A_ADC_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_OUT_B_ADC_r_OUT_A_ADC_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_OUT_B_ADC_r_OUT_A_ADC_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_TEMP_SEN_r_OUT_B_ADC_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_TEMP_SEN_r_OUT_B_ADC_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_TEMP_SEN_r_OUT_B_ADC_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_TEMP_SEN_r_OUT_B_ADC_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_TEMP_SEN_r_OUT_B_ADC_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_SMOKE_ADC_OP_r_OUT_ADC_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_SMOKE_ADC_OP_r_OUT_ADC_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_SMOKE_ADC_OP_r_OUT_ADC_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_SMOKE_ADC_OP_r_OUT_ADC_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_SMOKE_ADC_OP_r_OUT_ADC_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_OPAMP_IN_M_r_DC_24V_Pin /////
	if(REPEATOR_MODE == REPEATOR){
			GPIO_InitStruct.Pin = a_OPAMP_IN_M_r_DC_24V_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(a_OPAMP_IN_M_r_DC_24V_GPIO_Port, &GPIO_InitStruct);
		  }
	else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_OPAMP_IN_M_r_DC_24V_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_OPAMP_IN_M_r_DC_24V_GPIO_Port, &GPIO_InitStruct);
	  }


//	//// a_SMOKE_ADC_r_OUT4_Pin /////
//	if(REPEATOR_MODE == REPEATOR){
//		GPIO_InitStruct.Pin = a_OUT_A_ADC_r_OUT4_Pin;
//		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//		GPIO_InitStruct.Pull = GPIO_PULLUP;
//		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//		HAL_GPIO_Init(a_OUT_A_ADC_r_OUT4_GPIO_Port, &GPIO_InitStruct);
//		HAL_GPIO_WritePin(a_OUT_A_ADC_r_OUT4_GPIO_Port,a_OUT_A_ADC_r_OUT4_Pin,GPIO_PIN_SET);
//	  }
//	  else if(REPEATOR_MODE == ANALOG){
//		GPIO_InitStruct.Pin = a_OUT_A_ADC_r_OUT4_Pin;
//		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//		GPIO_InitStruct.Pull = GPIO_NOPULL;
//		HAL_GPIO_Init(a_OUT_A_ADC_r_OUT4_GPIO_Port, &GPIO_InitStruct);
//	  }
//
//	Rep_Set_Out(Output_Port4,Output_Off );
//
//	//// a_op1_vout_r_OUT3_Pin /////
//	if(REPEATOR_MODE == REPEATOR){
//		GPIO_InitStruct.Pin = a_op1_vout_r_OUT3_Pin;
//		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//		GPIO_InitStruct.Pull = GPIO_PULLUP;
//		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//		HAL_GPIO_Init(a_op1_vout_r_OUT3_GPIO_Port, &GPIO_InitStruct);
//		HAL_GPIO_WritePin(a_op1_vout_r_OUT3_GPIO_Port,a_op1_vout_r_OUT3_Pin,GPIO_PIN_SET);
//	  }
//	  else if(REPEATOR_MODE == ANALOG){
//		GPIO_InitStruct.Pin = a_op1_vout_r_OUT3_Pin;
//		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//		GPIO_InitStruct.Pull = GPIO_NOPULL;
//		HAL_GPIO_Init(a_op1_vout_r_OUT3_GPIO_Port, &GPIO_InitStruct);
//	  }
//
//	Rep_Set_Out(Output_Port3,Output_Off );
//
//	//// a_IR_LED_DAC_r_OUT2_Pin /////
//	if(REPEATOR_MODE == REPEATOR){
//		GPIO_InitStruct.Pin = a_IR_LED_DAC_r_OUT2_Pin;
//		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//		GPIO_InitStruct.Pull = GPIO_PULLUP;
//		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//		HAL_GPIO_Init(a_IR_LED_DAC_r_OUT2_GPIO_Port, &GPIO_InitStruct);
//		HAL_GPIO_WritePin(a_IR_LED_DAC_r_OUT2_GPIO_Port,a_IR_LED_DAC_r_OUT2_Pin,GPIO_PIN_SET);
//	  }
//	  else if(REPEATOR_MODE == ANALOG){
//		GPIO_InitStruct.Pin = a_IR_LED_DAC_r_OUT2_Pin;
//		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//		GPIO_InitStruct.Pull = GPIO_NOPULL;
//		HAL_GPIO_Init(a_IR_LED_DAC_r_OUT2_GPIO_Port, &GPIO_InitStruct);
//	  }
//
//	Rep_Set_Out(Output_Port2,Output_Off );
//
//	//// a_OUT_ADC_r_OUT1_Pin /////
//	if(REPEATOR_MODE == REPEATOR){
//		GPIO_InitStruct.Pin = a_OUT_ADC_r_OUT1_Pin;
//		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//		GPIO_InitStruct.Pull = GPIO_PULLUP;
//		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//		HAL_GPIO_Init(a_OUT_ADC_r_OUT1_GPIO_Port, &GPIO_InitStruct);
//		HAL_GPIO_WritePin(a_OUT_ADC_r_OUT1_GPIO_Port,a_OUT_ADC_r_OUT1_Pin,GPIO_PIN_SET);
//	  }
//	  else if(REPEATOR_MODE == ANALOG){
//		GPIO_InitStruct.Pin = a_OUT_ADC_r_OUT1_Pin;
//		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//		GPIO_InitStruct.Pull = GPIO_NOPULL;
//		HAL_GPIO_Init(a_OUT_ADC_r_OUT1_GPIO_Port, &GPIO_InitStruct);
//	  }
//
//	Rep_Set_Out(Output_Port1,Output_Off );

	//// a_OUT_A_ADC_r_OCP4_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_IN_B_ADC_r_OCP4_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(a_IN_B_ADC_r_OCP4_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_IN_B_ADC_r_OCP4_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_IN_B_ADC_r_OCP4_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_IN_ADC_r_OCP3_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_IN_ADC_r_OCP3_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(a_IN_ADC_r_OCP3_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_IN_ADC_r_OCP3_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_IN_ADC_r_OCP3_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_IN_B_ADC_r_OCP2_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = r_OCP2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(r_OCP2_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = r_OCP2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(r_OCP2_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_CON_OUT_RSET_r_OCP1_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_CON_OUT_RSET_r_OCP1_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(a_CON_OUT_RSET_r_OCP1_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_CON_OUT_RSET_r_OCP1_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_CON_OUT_RSET_r_OCP1_GPIO_Port, &GPIO_InitStruct);
	  }

	//// r_IN4_SIG_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = r_IN4_SIG_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(r_IN4_SIG_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = r_IN4_SIG_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(r_IN4_SIG_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_LED_BLUE_r_IN3_SIG_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_LED_RED_r_IN3_SIG_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(a_LED_RED_r_IN3_SIG_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_LED_RED_r_IN3_SIG_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_LED_RED_r_IN3_SIG_GPIO_Port, &GPIO_InitStruct);
		ANAL_RED_LED(LED_OFF);
	  }



	//// a_LED_RED_r_IN2_SIG_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_LED_BLUE_r_IN2_SIG_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(a_LED_BLUE_r_IN2_SIG_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_LED_BLUE_r_IN2_SIG_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_LED_BLUE_r_IN2_SIG_GPIO_Port, &GPIO_InitStruct);
		ANAL_BLUE_LED(LED_OFF);
	  }


	//// a_DIP_ADD7_r_IN1_SIG_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_DIP_ADD7_r_IN1_SIG_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(a_DIP_ADD7_r_IN1_SIG_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_DIP_ADD7_r_IN1_SIG_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(a_DIP_ADD7_r_IN1_SIG_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_DIP_ADD6_r_CH1_SW_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_DIP_ADD6_r_CH1_SW_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_DIP_ADD6_r_CH1_SW_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_DIP_ADD6_r_CH1_SW_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(a_DIP_ADD6_r_CH1_SW_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_DIP_ADD5_r_CH2_SW_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_DIP_ADD5_r_CH2_SW_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_DIP_ADD5_r_CH2_SW_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_DIP_ADD5_r_CH2_SW_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(a_DIP_ADD5_r_CH2_SW_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_DIP_ADD4_r_CH3_SW_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_DIP_ADD4_r_CH3_SW_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_DIP_ADD4_r_CH3_SW_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_DIP_ADD4_r_CH3_SW_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(a_DIP_ADD4_r_CH3_SW_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_DIP_ADD3_r_CH4_SW_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_DIP_ADD3_r_CH4_SW_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_DIP_ADD3_r_CH4_SW_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_DIP_ADD3_r_CH4_SW_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(a_DIP_ADD3_r_CH4_SW_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_DIP_ADD2_r_DIP_ADD5_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_DIP_ADD2_r_DIP_ADD5_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_DIP_ADD2_r_DIP_ADD5_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_DIP_ADD2_r_DIP_ADD5_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(a_DIP_ADD2_r_DIP_ADD5_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_DIP_ADD1_r_DIP_ADD6_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_DIP_ADD1_r_DIP_ADD6_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_DIP_ADD1_r_DIP_ADD6_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_DIP_ADD1_r_DIP_ADD6_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(a_DIP_ADD1_r_DIP_ADD6_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_DIP_ADD0_r_DIP_ADD7_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_DIP_ADD0_r_DIP_ADD7_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_DIP_ADD0_r_DIP_ADD7_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_DIP_ADD0_r_DIP_ADD7_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(a_DIP_ADD0_r_DIP_ADD7_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_SPI_CS_r_DIP_ADD4_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_SPI_CS_r_DIP_ADD4_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_SPI_CS_r_DIP_ADD4_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_SPI_CS_r_DIP_ADD4_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_SPI_CS_r_DIP_ADD4_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_SCK_r_DIP_ADD3_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_SCK_r_DIP_ADD3_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_SCK_r_DIP_ADD3_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_SCK_r_DIP_ADD3_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
		HAL_GPIO_Init(a_SCK_r_DIP_ADD3_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_MISO_DIP_ADD2_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_MISO_DIP_ADD2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_MISO_DIP_ADD2_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_MISO_DIP_ADD2_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
		HAL_GPIO_Init(a_MISO_DIP_ADD2_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_MOSI_r_DIP_ADD1_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_MOSI_r_DIP_ADD1_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_MOSI_r_DIP_ADD1_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_MOSI_r_DIP_ADD1_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
		HAL_GPIO_Init(a_MOSI_r_DIP_ADD1_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_IN_B_TEST_r_DIP_ADD0_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_IN_B_TEST_r_DIP_ADD0_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_IN_B_TEST_r_DIP_ADD0_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, &GPIO_InitStruct);

		HAL_GPIO_WritePin(a_IN_B_TEST_r_DIP_ADD0_GPIO_Port, a_IN_B_TEST_r_DIP_ADD0_Pin, GPIO_PIN_SET);
	  }



	//// a_CON_OUT_SET_r_DISCON1_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_CON_OUT_SET_r_DISCON1_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_CON_OUT_SET_r_DISCON1_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_CON_OUT_SET_r_DISCON1_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_CON_OUT_SET_r_DISCON1_GPIO_Port, &GPIO_InitStruct);
	  }


	//// a_N_MODE_r_DISCON2_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_N_MODE_r_DISCON2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_N_MODE_r_DISCON2_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_N_MODE_r_DISCON2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_N_MODE_r_DISCON2_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_SCL_r_DISCON3_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_SCL_r_DISCON3_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_SCL_r_DISCON3_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_SCL_r_DISCON3_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(a_SCL_r_DISCON3_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_SDA_r_DISCON4_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = a_SDA_r_DISCON4_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(a_SDA_r_DISCON4_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(REPEATOR_MODE == ANALOG){
		GPIO_InitStruct.Pin = a_SDA_r_DISCON4_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(a_SDA_r_DISCON4_GPIO_Port, &GPIO_InitStruct);
	  }

	//// a_IR_LED_DAC_r_OUT2_Pin /////
	if(REPEATOR_MODE == REPEATOR){
		GPIO_InitStruct.Pin = BOOT_R_N_MODE_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(BOOT_R_N_MODE_GPIO_Port, &GPIO_InitStruct);
	  }

}

uint32_t Read_ADC_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
	{
		/* Calibration Error */
		Error_Handler();
	}
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return adcValue;
}

void Check_Com(void){

	  __HAL_UART_ENABLE_IT(&hlpuart2, UART_IT_ERR);
	  /* Enable the UART Data Register not empty Interrupt */
	  __HAL_UART_ENABLE_IT(&hlpuart2, UART_IT_RXNE);


	uint8_t CRC_Temp;

	if(UART_Receive_complete == 1){	//motor mode


		if(UART_RX_buf[2] == 0xC9) // Setting waterproof Temperature's address
		{
			if(ADDRESS_MODES == Non_ADDRESS)
			{
				Analog_Address = UART_RX_buf[1];
				Flash_Erase(Waterproof_Temperature_ADDR);
				Flash_Write_uint8(Waterproof_Temperature_ADDR, Analog_Address);
				Flash_Read_uint8(Waterproof_Temperature_ADDR, &Analog_Address);
				Com_C0();
			}
		}
		else if((CCU_Address == UART_RX_buf[1]) | (CCU_Address == 254)) {
			CRC_Temp = UART_RX_buf[1] ^ UART_RX_buf[2] ^ UART_RX_buf[3];
			if(CRC_Temp == UART_RX_buf[4]){
				// Request for information
				if(UART_RX_buf[2] == 0xC0){
					Com_C0();
				}
				//output Setting
				else if(UART_RX_buf[2] == 0xC1){	//output Set
					Com_C1();
				}
				//Charge Set On
				else if(UART_RX_buf[2] == 0xC2){
					Com_C2();
				}
				//Charge Set Off
				else if(UART_RX_buf[2] == 0xC4){
					Com_C4();
				}
				else if(UART_RX_buf[2] == 0xC5){
					Com_C5();
				}
				else if(UART_RX_buf[2] == 0xC6){
					Com_C6();
				}
				else if(UART_RX_buf[2] == 0xF0){
					Com_F0();
				}
				else if (UART_RX_buf[2] == 0xCA) // Zero setting
				{
					AdjustmentCmdPrcessing(0xCA);
				}
				else if (UART_RX_buf[2] == 0xCB) // xx.0 +,-
				{
					AdjustmentCmdPrcessing(0xCB);
				}
				else if (UART_RX_buf[2] == 0xCC) // 0.x +,-
				{
					AdjustmentCmdPrcessing(0xCC);
				}
				else if(UART_RX_buf[2] == 0xCD)
				{
					AdjustmentCmdPrcessing(0xCD);
				}
				else if(UART_RX_buf[2] == 0xE1){
					//Com_C0_Old();
				}
				else if(UART_RX_buf[2] == 0xEA){
					//Com_C0_Old();
				}
				else if(UART_RX_buf[2] == 0xEE){
					//Com_C0_Old();
				}
				else if(UART_RX_buf[2] == 0xB1){
					Com_B1();
				}

				if(REPEATOR_OS_MODES == Open_Short){

					if(Rep_Output_value[0] == 0 ){
						if (Ch1_Out_Off_Cnt > 2){
							Check_Ch1();
						}
						else{
							Ch1_Out_Off_Cnt++;
							Ch1_Out_Open = Normal;
						}

					}
					else {
						Ch1_Out_Open = Normal;
						Ch1_Out_Off_Cnt = 0;

					}
				}
				else{
					Ch1_Out_Open = Open_Short;
				}
			}
		}
		if(UART_RX_buf[1] == 253){
			CRC_Temp = UART_RX_buf[1] ^ UART_RX_buf[2] ^ UART_RX_buf[3];
			if(CRC_Temp == UART_RX_buf[4]){
				if(UART_RX_buf[2] == 0xB0){
					Com_B0();
				}
			}

		}
		UART_Receive_complete = 0;
		IfAddressMatched = 0;


	}
}
// Request for information
void Com_C0(void){

	uint8_t CRC_Temp;

	UART_TX_buf[0] = 0x02;
	UART_TX_buf[1] = CCU_Address;
	UART_TX_buf[2] = 0x00;
	UART_TX_buf[3] = 0x00;
	UART_TX_buf[4] = 0x00;
	UART_TX_buf[5] = 0x00;
	UART_TX_buf[6] = 0x03;

	if(CCU_Address == 254){
		UART_TX_buf[1] = UART_RX_buf[1];
	}

	if(REPEATOR_MODE == REPEATOR){

		Rep_TX_LED(LED_ON);

		if(CCU_Address == 254){
			if(1 == UART_RX_buf[1]){
				UART_TX_buf[2] = (0x00<<7)|(0x00<<6)|
								(0x00<<5)|(Rep_port_Fuse_Open_Mode[0]<<4)|
								(0x00<<3)|(0x00<<2)|
								(0x00<<1)|(Rep_Output_value[0]<<0);
				UART_TX_buf[3] = ((Rep_port_Charge_Mode[3] & 0x01)<<7)|((Rep_port_Charge_Mode[2] & 0x01)<<6)|
								((Rep_port_Charge_Mode[1] & 0x01)<<5)|((Rep_port_Charge_Mode[0] & 0x01)<<4)|
								(0x00<<3)|(0x00<<2)|
								(0x00<<1)|(Rep_Input_value[0]<<0);

				UART_TX_buf[4] = ((Rep_V24_value&0x01)<<7)|((Ch1_Out_Open&0x01)<<6)|
								((REPEATOR_OS_MODES&0x01)<<5)|((ISO_MODE&0x01)<<4)|
								(0x00<<3)|(0x00<<2)|
								(0x00<<1)|(Rep_port_open[0]<<0);

			}
			else if(85 == UART_RX_buf[1]){

				UART_TX_buf[2] = (0x00<<7)|(0x00<<6)|
								(Rep_port_Fuse_Open_Mode[1]<<5)|(0<<4)|
								(0x00<<3)|(0x00<<2)|
								((Rep_Output_value[1]<<1)|0<<0);

				UART_TX_buf[3] = ((Rep_port_Charge_Mode[3] & 0x01)<<7)|((Rep_port_Charge_Mode[2] & 0x01)<<6)|
								((Rep_port_Charge_Mode[1] & 0x01)<<5)|((Rep_port_Charge_Mode[0] & 0x01)<<4)|
								(0x00<<3)|(0x00<<2)|
								(Rep_Input_value[1]<<1)|(0x00<<0);

				UART_TX_buf[4] = ((Rep_V24_value&0x01)<<7)|((Ch1_Out_Open&0x01)<<6)|
								((REPEATOR_OS_MODES&0x01)<<5)|((ISO_MODE&0x01)<<4)|
								(0x00<<3)|(0x00<<2)|
								(Rep_port_open[1]<<1)|(0x00<<0);

			}
			else if(170 == UART_RX_buf[1]){
				UART_TX_buf[2] = (0x00<<7)|(Rep_port_Fuse_Open_Mode[2]<<6)|
								(0x00<<5)|(0x00<<4)|
								(0x00<<3)|(Rep_Output_value[2]<<2)|
								((0x00<<1)|0x00<<0);

				UART_TX_buf[3] = ((Rep_port_Charge_Mode[3] & 0x01)<<7)|((Rep_port_Charge_Mode[2] & 0x01)<<6)|
								((Rep_port_Charge_Mode[1] & 0x01)<<5)|((Rep_port_Charge_Mode[0] & 0x01)<<4)|
								(0x00<<3)|(Rep_Input_value[2]<<2)|
								(0x00<<1)|(0x00<<0);

				UART_TX_buf[4] = ((Rep_V24_value&0x01)<<7)|((Ch1_Out_Open&0x01)<<6)|
								((REPEATOR_OS_MODES&0x01)<<5)|((ISO_MODE&0x01)<<4)|
								(0x00<<3)|(Rep_port_open[2]<<2)|
								(0x00<<1)|(0x00<<0);
			}
			else if(250 == UART_RX_buf[1]){
				UART_TX_buf[2] = (Rep_port_Fuse_Open_Mode[3]<<7)|(0x00<<6)|
								(0x00<<5)|(0x00<<4)|
								(Rep_Output_value[3]<<3)|(0x00<<2)|
								((0x00<<1)|0x00<<0);

				UART_TX_buf[3] = ((Rep_port_Charge_Mode[3] & 0x01)<<7)|((Rep_port_Charge_Mode[2] & 0x01)<<6)|
								((Rep_port_Charge_Mode[1] & 0x01)<<5)|((Rep_port_Charge_Mode[0] & 0x01)<<4)|
								(Rep_Input_value[3]<<3)|(0x00<<2)|
								(0x00<<1)|(0x00<<0);

				UART_TX_buf[4] = ((Rep_V24_value&0x01)<<7)|((Ch1_Out_Open&0x01)<<6)|
								((REPEATOR_OS_MODES&0x01)<<5)|((ISO_MODE&0x01)<<4)|
								(Rep_port_open[3]<<3)|(0x00<<2)|
								(0x00<<1)|(0x00<<0);
			}
		}
		else{

			UART_TX_buf[2] = (Rep_port_Fuse_Open_Mode[3]<<7)|(Rep_port_Fuse_Open_Mode[2]<<6)|
							(Rep_port_Fuse_Open_Mode[1]<<5)|(Rep_port_Fuse_Open_Mode[0]<<4)|
							(Rep_Output_value[3]<<3)|(Rep_Output_value[2]<<2)|
							(Rep_Output_value[1]<<1)|(Rep_Output_value[0]<<0);

			UART_TX_buf[3] = ((Rep_port_Charge_Mode[3] & 0x01)<<7)|((Rep_port_Charge_Mode[2] & 0x01)<<6)|
							((Rep_port_Charge_Mode[1] & 0x01)<<5)|((Rep_port_Charge_Mode[0] & 0x01)<<4)|
							(Rep_Input_value[3]<<3)|(Rep_Input_value[2]<<2)|
							(Rep_Input_value[1]<<1)|(Rep_Input_value[0]<<0);


			UART_TX_buf[4] = ((Rep_V24_value&0x01)<<7)|((Ch1_Out_Open&0x01)<<6)|
							((REPEATOR_OS_MODES&0x01)<<5)|((ISO_MODE&0x01)<<4)|
							(Rep_port_open[3]<<3)|(Rep_port_open[2]<<2)|
							(Rep_port_open[1]<<1)|(Rep_port_open[0]<<0);


//			UART_TX_buf[2] = (uint8_t)Ch1_On_Save_Sum;
//			UART_TX_buf[3] = (uint8_t)Ch1_Off_Save_Sum;
		}



		/*
		if(Rep_V24_value == 1){
			if(ISO_MODE == 0){
				UART_TX_buf[4] = 0xA0 |
					(Rep_port_open[3]<<3)|(Rep_port_open[2]<<2)|
					(Rep_port_open[1]<<1)|(Rep_port_open[0]<<0);
			}
			else if(ISO_MODE == 1){
				UART_TX_buf[4] = 0xA0 |
						(Rep_port_open[3]<<3)|(Rep_port_open[2]<<2)|
						(Rep_port_open[1]<<1)|(Rep_port_open[0]<<0);
				if(CCU_Address == 254){
					if(1 == UART_RX_buf[1]){
						UART_TX_buf[4] = 0xA0 |
										(0x00<<3)|(0x00<<2)|
										(0x00<<1)|(Rep_port_open[0]<<0);

					}
					else if(85 == UART_RX_buf[1]){
						UART_TX_buf[4] = 0xA0 |
										(0x00<<3)|(0x00<<2)|
										(Rep_port_open[1]<<1)|(0x00<<0);

					}
					else if(170 == UART_RX_buf[1]){
						UART_TX_buf[4] = 0xA0 |
											(0x00<<3)|(Rep_port_open[2]<<2)|
											(0x00<<1)|(0x00<<0);
					}
					else if(250 == UART_RX_buf[1]){
						UART_TX_buf[4] = 0xA0 |
											(Rep_port_open[3]<<3)|(0x00<<2)|
											(0x00<<1)|(0x00<<0);

					}
					else{
						UART_TX_buf[4] = 0xA0 |
											(0x00<<3)|(0x00<<2)|
											(0x00<<1)|(0x00<<0);
					}
				}
			}
		}
		else if(Rep_V24_value == 0){
			UART_TX_buf[4] = 0x20 |
						(Rep_port_open[3]<<3)|(Rep_port_open[2]<<2)|
						(Rep_port_open[1]<<1)|(Rep_port_open[0]<<0);
			if(CCU_Address == 254){
				if(1 == UART_RX_buf[1]){
					UART_TX_buf[4] = 0xA0 |
									(0x00<<3)|(0x00<<2)|
									(0x00<<1)|(Rep_port_open[0]<<0);

				}
				else if(85 == UART_RX_buf[1]){
					UART_TX_buf[4] = 0xA0 |
									(0x00<<3)|(0x00<<2)|
									(Rep_port_open[1]<<1)|(0x00<<0);

				}
				else if(170 == UART_RX_buf[1]){
					UART_TX_buf[4] = 0xA0 |
										(0x00<<3)|(Rep_port_open[2]<<2)|
										(0x00<<1)|(0x00<<0);
				}
				else if(250 == UART_RX_buf[1]){
					UART_TX_buf[4] = 0xA0 |
										(Rep_port_open[3]<<3)|(0x00<<2)|
										(0x00<<1)|(0x00<<0);

				}
				else{
					UART_TX_buf[4] = 0xA0 |
										(0x00<<3)|(0x00<<2)|
										(0x00<<1)|(0x00<<0);
				}
			}
		}
		*/

		CRC_Temp = UART_TX_buf[1]^UART_TX_buf[2]^UART_TX_buf[3]^UART_TX_buf[4];
		UART_TX_buf[5] = CRC_Temp;

		HAL_UART_Transmit(&hlpuart2, UART_TX_buf, 7, 1000);
		HAL_Delay(1);
		HAL_GPIO_WritePin(BOOT_R_N_MODE_GPIO_Port, BOOT_R_N_MODE_Pin,  GPIO_PIN_RESET);
		UART_Receive_complete = 0;
		for(int i=0; i<UART_buf_len; i++){
			UART_RX_buf[i]=0;
		}
		Rep_TX_LED(LED_OFF);

	}
	else if(REPEATOR_MODE == ANALOG)
	{
		if (Ana_Led_Com_mode == LED_ON) {
			ANAL_BLUE_LED(LED_ON);
		} else if (Ana_Led_Com_mode == LED_OFF) {
			ANAL_BLUE_LED(LED_OFF);
		}

		/****************ISO Start****************/
		uint8_t In_ISO_open;
		uint8_t In_ISO_short;
		uint8_t Out_ISO_open;
		uint8_t Out_ISO_short;

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


		UART_TX_buf[2] =  (In_ISO_short & 0x01) << 7 /* ISO MODE*/
				| (In_ISO_open & 0x01) << 6 /* ISO MODE*/
				| (Out_ISO_short & 0x01) << 5 /* ISO MODE*/
				| (Out_ISO_open & 0x01) << 4 /* ISO MODE*/
				| (ISO_MODE & 0x01) << 3 /* ISO MODE*/
				| (Ana_Led_Com_mode & 0x01) << 2
				| (Ana_Led_Fire_Mode & 0x01) << 0;

		/****************ISO End****************/

		if (ANALOG_MODE == TEMPERATURE) {
			UART_TX_buf[3] = Ana_Temp_Sen_Com;
			UART_TX_buf[4] = 0x44;
		}
		else if (ANALOG_MODE == SMOKE) {
//			scaledValue = (uint8_t)((Ana_photo_Sen_12bit * 255) / 4095);
//			UART_TX_buf[2] = scaledValue;
//			UART_TX_buf[2] = (uint8_t)(diff >> 8);
//			UART_TX_buf[3] = (uint8_t)(diff & 0xFF);
			UART_TX_buf[3] = ReturnPercent;
			UART_TX_buf[4] = PhotoStatus; // 0x40 : photo sensor, 0x41 : photo Led Open, 0x42 : Photo Led Short
//			UART_TX_buf[4] = 0x40;
//			UART_TX_buf[2] = DAC_10multiplied;
//			UART_TX_buf[3] = (uint8_t)(Ana_photo_Sen_12bit >> 8);
//			UART_TX_buf[4] = (uint8_t)(Ana_photo_Sen_12bit & 0xFF);
		}
		CRC_Temp = UART_TX_buf[1] ^ UART_TX_buf[2] ^ UART_TX_buf[3] ^ UART_TX_buf[4];
		UART_TX_buf[5] = CRC_Temp;

		HAL_UART_Transmit(&hlpuart2, UART_TX_buf, 7, 1000);
		HAL_Delay(10);
		UART_Receive_complete = 0;
		for (int i = 0; i < UART_buf_len; i++) {
			UART_RX_buf[i] = 0;
		}
		ANAL_BLUE_LED(LED_OFF);

		Read_Analog_ADC(); // 62ms
	}
}

//output Setting
void Com_C1(void){

	if(REPEATOR_MODE == REPEATOR){
		if( ((UART_RX_buf[3]>>3) & 0x01) == 0x01){
			Rep_Output_value[3] = 1;
		}
		else{
			Rep_Output_value[3] = 0;
		}

		if( ((UART_RX_buf[3]>>2) & 0x01) == 0x01){
			Rep_Output_value[2] = 1;
		}
		else{
			Rep_Output_value[2] = 0;
		}

		if( ((UART_RX_buf[3]>>1) & 0x01) == 0x01){
			Rep_Output_value[1] = 1;
		}
		else{
			Rep_Output_value[1] = 0;
		}

		if( ((UART_RX_buf[3]>>0) & 0x01) == 0x01){
			Rep_Output_value[0] = 1;
		}
		else{
			Rep_Output_value[0] = 0;
		}

		if(CCU_Address == 254){

			Rep_Output_value[0] = 0;
			Rep_Output_value[1] = 0;
			Rep_Output_value[2] = 0;
			Rep_Output_value[3] = 0;

			if(1 == UART_RX_buf[1]){
				if( ((UART_RX_buf[3]>>0) & 0x01) == 0x01){
							Rep_Output_value[0] = 1;
						}
						else{
							Rep_Output_value[0] = 0;
						}
			}
			else if(85 == UART_RX_buf[1]){
				if( ((UART_RX_buf[3]>>1) & 0x01) == 0x01){
					Rep_Output_value[1] = 1;
				}
				else{
					Rep_Output_value[1] = 0;
				}
			}
			else if(170 == UART_RX_buf[1]){
				if( ((UART_RX_buf[3]>>2) & 0x01) == 0x01){
							Rep_Output_value[2] = 1;
						}
						else{
							Rep_Output_value[2] = 0;
						}
			}
			else if(250 == UART_RX_buf[1]){
				if( ((UART_RX_buf[3]>>3) & 0x01) == 0x01){
					Rep_Output_value[3] = 1;
				}
				else{
					Rep_Output_value[3] = 0;
				}
			}
			else{
				Rep_Output_value[0] = 0;
				Rep_Output_value[1] = 0;
				Rep_Output_value[2] = 0;
				Rep_Output_value[3] = 0;
			}
		}

	}
	else if(REPEATOR_MODE == ANALOG){

	}
}



//Charge Set On
void Com_C2(void){

	uint8_t Charge_Time[4];

	if(REPEATOR_MODE == REPEATOR){
		if( ((UART_RX_buf[3]>>3) & 0x01) == 0x01){
			Rep_port_Charge_Setting[3] = 1;
		}
		else{
			Rep_port_Charge_Setting[3] = 0;
		}

		if( ((UART_RX_buf[3]>>2) & 0x01) == 0x01){
			Rep_port_Charge_Setting[2] = 1;
		}
		else{
			Rep_port_Charge_Setting[2] = 0;
		}

		if( ((UART_RX_buf[3]>>1) & 0x01) == 0x01){
			Rep_port_Charge_Setting[1] = 1;
		}
		else{
			Rep_port_Charge_Setting[1] = 0;
		}

		if( ((UART_RX_buf[3]>>0) & 0x01) == 0x01){
			Rep_port_Charge_Setting[0] = 1;
		}
		else{
			Rep_port_Charge_Setting[0] = 0;
		}
/*
		Charge_Time = ((UART_RX_buf[3]>>4) & 0x0f)*10 - 2;
		Rep_port_Charge_Time[0] = (UART_RX_buf[3]>>4) & 0x0f;
		Rep_port_Charge_Time[1] = (UART_RX_buf[3]>>4) & 0x0f;
		Rep_port_Charge_Time[2] = (UART_RX_buf[3]>>4) & 0x0f;
		Rep_port_Charge_Time[3] = (UART_RX_buf[3]>>4) & 0x0f;
*/
		for(int i=0; i<4 ; i++){
			if(UART_RX_buf[3] == 0){
				Charge_Time[i] = 0;
			}
			else{
				Charge_Time[i] = ((UART_RX_buf[3]>>4) & 0x0f)*10 - 2;
			}
			Rep_port_Charge_Time[i] = Charge_Time[i];
		}
//		if( ((UART_RX_buf[3]>>4) & 0x0f) > 0){
//			Rep_All_port_Charge_Mode = 1;
//		}
//		else {
//			Rep_All_port_Charge_Mode = 0;
//		}

		for(int i = 0 ; i<4 ; i++){
			Rep_port_Charge_Mode[i] = Rep_port_Charge_Setting[i];
		}
	}
	else if(REPEATOR_MODE == ANALOG){

	}
}


//Charge Set Off
void Com_C4(void){

	uint8_t Charge_Time[4];

	if(REPEATOR_MODE == REPEATOR){
		if( ((UART_RX_buf[3]>>3) & 0x01) == 0x01){
			Rep_port_Charge_Setting[3] = 1;
		}
		else{
			Rep_port_Charge_Setting[3] = 0;
		}

		if( ((UART_RX_buf[3]>>2) & 0x01) == 0x01){
			Rep_port_Charge_Setting[2] = 1;
		}
		else{
			Rep_port_Charge_Setting[2] = 0;
		}

		if( ((UART_RX_buf[3]>>1) & 0x01) == 0x01){
			Rep_port_Charge_Setting[1] = 1;
		}
		else{
			Rep_port_Charge_Setting[1] = 0;
		}

		if( ((UART_RX_buf[3]>>0) & 0x01) == 0x01){
			Rep_port_Charge_Setting[0] = 1;
		}
		else{
			Rep_port_Charge_Setting[0] = 0;
		}
/*
		Charge_Time = ((UART_RX_buf[3]>>4) & 0x0f)*10 - 2;
		Rep_port_Charge_Time[0] = (UART_RX_buf[3]>>4) & 0x0f;
		Rep_port_Charge_Time[1] = (UART_RX_buf[3]>>4) & 0x0f;
		Rep_port_Charge_Time[2] = (UART_RX_buf[3]>>4) & 0x0f;
		Rep_port_Charge_Time[3] = (UART_RX_buf[3]>>4) & 0x0f;
*/
		/*
		for(int i=0; i<4 ; i++){
			Charge_Time[i] = ((UART_RX_buf[3]>>4) & 0x0f)*10 - 2;
			Rep_port_Charge_Time[i] = Charge_Time[i];
		}
		*/

		for(int i=0; i<4 ; i++){
			Charge_Time[i] =0;
			Rep_port_Charge_Time[i] = Charge_Time[i];
		}


//		if( ((UART_RX_buf[3]>>4) & 0x0f) > 0){
//			Rep_All_port_Charge_Mode = 1;
//		}
//		else {
//			Rep_All_port_Charge_Mode = 0;
//		}

		for(int i = 0 ; i<4 ; i++){
			Rep_port_Charge_Mode[i] = Rep_port_Charge_Setting[i];
		}
	}
	else if(REPEATOR_MODE == ANALOG){

	}
}

void Com_B0(void){


	UART_BAUDRATE = 1200 * 2 * UART_RX_buf[3];

	LPUART2_UART_RE_Init(Uart_DIR_Mode, UART_BAUDRATE);

}

void Com_B1(void){

	uint8_t CRC_Temp;

	UART_TX_buf[0] = 0x02;
	UART_TX_buf[1] = CCU_Address;
	UART_TX_buf[2] = 0x00;
	UART_TX_buf[3] = 0x00;
	UART_TX_buf[4] = 0x00;
	UART_TX_buf[5] = 0x00;
	UART_TX_buf[6] = 0x03;

	Rep_TX_LED(LED_ON);

	if(UART_RX_buf[3] == 0){
		UART_TX_buf[2] = F_Version_Year;
		UART_TX_buf[3] = F_Version_Month;
		UART_TX_buf[4] = F_Version_Day;
	}
	else if(UART_RX_buf[3] == 1){
		UART_TX_buf[2] = F_Version_Hour;
		UART_TX_buf[3] = F_Version_Min;
		UART_TX_buf[4] = F_Version_Sec;
	}

	CRC_Temp = UART_TX_buf[1]^UART_TX_buf[2]^UART_TX_buf[3]^UART_TX_buf[4];
	UART_TX_buf[5] = CRC_Temp;

	HAL_UART_Transmit(&hlpuart2, UART_TX_buf, 7, 1000);
	HAL_Delay(1);
	UART_Receive_complete = 0;
	for(int i=0; i<UART_buf_len; i++){
		UART_RX_buf[i]=0;
	}
	Rep_TX_LED(LED_OFF);
}

void Repeater_Out_Init(void){

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = a_OUT_ADC_r_OUT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(a_OUT_ADC_r_OUT1_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = BOOT_R_N_MODE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BOOT_R_N_MODE_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = a_op1_vout_r_OUT3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(a_op1_vout_r_OUT3_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = a_OUT_A_ADC_r_OUT4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(a_OUT_A_ADC_r_OUT4_GPIO_Port, &GPIO_InitStruct);

	Rep_Set_Out(Output_Port1,Output_Off);
	Rep_Set_Out(Output_Port2,Output_Off);
	Rep_Set_Out(Output_Port3,Output_Off);
	Rep_Set_Out(Output_Port4,Output_Off);

}

void LPUART2_UART_RE_Init(uint8_t Side, int BAUDRATE)
{

	if (HAL_UART_DeInit(&hlpuart2) != HAL_OK)
	  {
		Error_Handler();
	  }


	HAL_UART_MspDeInit(&hlpuart2);

	HAL_UART_MspInit(&hlpuart2);

  hlpuart2.Instance = LPUART2;
  hlpuart2.Init.BaudRate = BAUDRATE;
  hlpuart2.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart2.Init.StopBits = UART_STOPBITS_1;
  hlpuart2.Init.Parity = UART_PARITY_NONE;
  hlpuart2.Init.Mode = UART_MODE_TX_RX;
  hlpuart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  if(Side == 0){
	  hlpuart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  hlpuart2.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_DISABLE;
  }
  else if(Side == 1){
	  hlpuart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
	  hlpuart2.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  }
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
}


//////////////////////////////////////////250521//////////////////////////////////////////

void Set_ISO_Init(void){

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	  if(ISO_MODE == ISO){

			//// a_OUT_A_TEST_r_CON_OUT_SET_Pin /////
			if(REPEATOR_MODE == REPEATOR){
				GPIO_InitStruct.Pin = a_OUT_A_TEST_r_CON_OUT_SET_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				HAL_GPIO_Init(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, &GPIO_InitStruct);
			  }
			  else if(REPEATOR_MODE == ANALOG){
				  GPIO_InitStruct.Pin = a_OUT_A_TEST_r_CON_OUT_SET_Pin;
				  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				  GPIO_InitStruct.Pull = GPIO_NOPULL;
				  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				  HAL_GPIO_Init(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, &GPIO_InitStruct);
				  HAL_GPIO_WritePin(a_OUT_A_TEST_r_CON_OUT_SET_GPIO_Port, a_OUT_A_TEST_r_CON_OUT_SET_Pin, GPIO_PIN_SET);
			  }

			//// a_IN_A_ADC_r_CON_OUT_RSET_Pin /////
			if(REPEATOR_MODE == REPEATOR){
				GPIO_InitStruct.Pin = a_IN_A_ADC_r_CON_OUT_RSET_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				HAL_GPIO_Init(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, &GPIO_InitStruct);
			  }
			  else if(REPEATOR_MODE == ANALOG){
				  GPIO_InitStruct.Pin = a_IN_A_ADC_r_CON_OUT_RSET_Pin;
				  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
				  GPIO_InitStruct.Pull = GPIO_NOPULL;
				  HAL_GPIO_Init(a_IN_A_ADC_r_CON_OUT_RSET_GPIO_Port, &GPIO_InitStruct);
			  }

			//// a_CON_IN_SET_r_LED_ISOERR_Pin /////
			if(REPEATOR_MODE == REPEATOR){
				GPIO_InitStruct.Pin = a_CON_IN_SET_r_LED_ISOERR_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				HAL_GPIO_Init(a_CON_IN_SET_r_LED_ISOERR_GPIO_Port, &GPIO_InitStruct);
				Rep_ISO_LED(LED_OFF);
			  }
			  else if(REPEATOR_MODE == ANALOG){
				  GPIO_InitStruct.Pin = a_CON_IN_SET_r_LED_ISOERR_Pin;
				  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				  GPIO_InitStruct.Pull = GPIO_NOPULL;
				  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				  HAL_GPIO_Init(a_CON_IN_SET_r_LED_ISOERR_GPIO_Port, &GPIO_InitStruct);
			  }

			//// a_CON_IN_RSET_r_OUT_A_TEST_Pin /////
			if(REPEATOR_MODE == REPEATOR){
				GPIO_InitStruct.Pin = a_CON_IN_RSET_r_OUT_A_TEST_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				HAL_GPIO_Init(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, &GPIO_InitStruct);
				HAL_GPIO_WritePin(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, a_CON_IN_RSET_r_OUT_A_TEST_Pin, GPIO_PIN_RESET);
			  }
			  else if(REPEATOR_MODE == ANALOG){
				  GPIO_InitStruct.Pin = a_CON_IN_RSET_r_OUT_A_TEST_Pin;
				  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				  GPIO_InitStruct.Pull = GPIO_NOPULL;
				  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				  HAL_GPIO_Init(a_CON_IN_RSET_r_OUT_A_TEST_GPIO_Port, &GPIO_InitStruct);
			  }

			//// a_CON_OUT_SET_r_DISCON1_Pin /////
			if(REPEATOR_MODE == REPEATOR){
				GPIO_InitStruct.Pin = a_CON_OUT_SET_r_DISCON1_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_PULLUP;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				HAL_GPIO_Init(a_CON_OUT_SET_r_DISCON1_GPIO_Port, &GPIO_InitStruct);
			  }
			  else if(REPEATOR_MODE == ANALOG){
				GPIO_InitStruct.Pin = a_CON_OUT_SET_r_DISCON1_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				HAL_GPIO_Init(a_CON_OUT_SET_r_DISCON1_GPIO_Port, &GPIO_InitStruct);
			  }

			//// a_CON_OUT_RSET_r_OCP1_Pin /////
			if(REPEATOR_MODE == REPEATOR){
				GPIO_InitStruct.Pin = a_CON_OUT_RSET_r_OCP1_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_PULLDOWN;
				HAL_GPIO_Init(a_CON_OUT_RSET_r_OCP1_GPIO_Port, &GPIO_InitStruct);
			  }
			  else if(REPEATOR_MODE == ANALOG){
				GPIO_InitStruct.Pin = a_CON_OUT_RSET_r_OCP1_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				HAL_GPIO_Init(a_CON_OUT_RSET_r_OCP1_GPIO_Port, &GPIO_InitStruct);
			  }

			if(REPEATOR_MODE == REPEATOR){
				Rep_Out_Relay_Mode = Relay_On;
				Repeater_ISO_OUT_Off_Set();
			}
			else if(REPEATOR_MODE == ANALOG){
				Anal_ISO_IN_RY_Mode = Rel_On;
				Anal_ISO_OUT_RY_Mode = Rel_On;
				Anal_ISO_IN_Off_Set();
				Anal_ISO_OUT_Off_Set();
			}

	  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
