
#include "main.h"
#include <stdint.h>
#include <string.h> // use memcpy function
//#include <math.h> // use fmod function
#include "flash.h"
#include "function_Analog.h"
#include "stm32u0xx.h"

#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_63   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     (ADDR_FLASH_PAGE_63 + FLASH_PAGE_SIZE - 1)   /* End @ of user Flash area */

uint32_t Waterproof_Temperature_ADDR = ADDR_FLASH_PAGE_62;
uint32_t Adjust_Y_Intercept_ADDR = ADDR_FLASH_PAGE_61;
uint32_t LED_X_ADC_Value_ADDR = ADDR_FLASH_PAGE_60;
uint32_t FinalPcntVal_Required4SlopeCalculation_ADDR = ADDR_FLASH_PAGE_59;
uint32_t photoADC_Required4SlopeCalculation_ADDR = ADDR_FLASH_PAGE_58;

uint32_t SMOKE_DAC_Multiplied_100_ADDR = ADDR_FLASH_PAGE_57;
uint32_t SMOKE_PGA_ADDR = ADDR_FLASH_PAGE_56;

uint32_t ISO_STATUS_ADDR = ADDR_FLASH_PAGE_55;

uint32_t FirstPage = 0, NbOfPages = 0;
uint32_t Address = 0, PageError = FLASH_CR_MER1;
__IO uint32_t MemoryProgramStatus = 0;
__IO uint32_t data32 = 0;

static FLASH_EraseInitTypeDef EraseInitStruct;

uint32_t GetPage(uint32_t Addr)
{
  return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}

void Flash_EraseAt(uint32_t FlashAddress, uint32_t length)
{
	ANAL_RED_LED(LED_ON);
	HAL_Delay(300);

	uint8_t IsErased = 0;

	while(IsErased != 1)
	{
		Flash_Erase(FlashAddress);
		IsErased = Flash_IsErased(FlashAddress, length);
		HAL_Delay(100);
	}
	if(IsErased == 1)
	{
		ANAL_RED_LED(LED_OFF);
		HAL_Delay(100);
	}
}

void Flash_Erase(uint32_t address)
{
	HAL_FLASH_Unlock();

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	FirstPage = GetPage(address);

	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page        = FirstPage;
	EraseInitStruct.NbPages     = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{
		while (1)
		{
		  Error_Handler();
		}
	}
	HAL_FLASH_Lock();
}

uint8_t Flash_IsErased(uint32_t startAddr, uint32_t length)
{
    uint8_t *p = (uint8_t*)startAddr;

    for (uint32_t i = 0; i < length; i++)
    {
        if (p[i] != 0xFF)
        {
            return 0;   // 하나라도 0xFF 아니면 지워진 게 아님
        }
    }

    return 1;            // 전체가 0xFF
}

void Flash_Write_uint8(uint32_t address, uint8_t value)
{
	uint64_t UINT_DIGIT_VALUE = (uint64_t)value;

	HAL_FLASH_Unlock();

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, UINT_DIGIT_VALUE) != HAL_OK)
	{
	  while (1)
	  {
		Error_Handler();
	  }
	}

	HAL_FLASH_Lock();
}

void Flash_Write_uint16(uint32_t address, uint16_t value)
{
	uint64_t UINT_DIGIT_VALUE = (uint64_t)value;

	HAL_FLASH_Unlock();

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, UINT_DIGIT_VALUE) != HAL_OK)
	{
	  while (1)
	  {
		Error_Handler();
	  }
	}

	HAL_FLASH_Lock();
}

void Flash_Write_int16(uint32_t address, int16_t value)
{
	uint64_t UINT_DIGIT_VALUE = (uint64_t)value;

	HAL_FLASH_Unlock();

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, UINT_DIGIT_VALUE) != HAL_OK)
	{
	  while (1)
	  {
		Error_Handler();
	  }
	}

	HAL_FLASH_Lock();
}

// 지정된 주소에서 uint64_t 값을 읽는 함수
void Flash_Read_int16(uint32_t address, int16_t *value)
{
	uint64_t UINT_DIGIT_VALUE =  *((__IO uint64_t *)address);
	*value = (int16_t)UINT_DIGIT_VALUE;
}

void Flash_Read_uint16(uint32_t address, uint16_t *value)
{
	uint64_t UINT_DIGIT_VALUE =  *((__IO uint64_t *)address);
	*value = (uint16_t)UINT_DIGIT_VALUE;
}

void Flash_Read_uint8(uint32_t address, uint8_t *value)
{
	uint64_t UINT_DIGIT_VALUE =  *((__IO uint64_t *)address);
	*value = (uint8_t)UINT_DIGIT_VALUE;
}
