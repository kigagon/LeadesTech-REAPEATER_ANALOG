
#ifndef FLASH_H_
#define FLASH_H_

//extern uint32_t shunt_Current_ADDR;
extern uint32_t Waterproof_Temperature_ADDR;
extern uint32_t Adjust_Y_Intercept_ADDR;
extern uint32_t LED_X_ADC_Value_ADDR;
extern uint32_t FinalPcntVal_Required4SlopeCalculation_ADDR;
extern uint32_t photoADC_Required4SlopeCalculation_ADDR;
extern uint32_t SMOKE_DAC_Multiplied_100_ADDR;
extern uint32_t SMOKE_PGA_ADDR;

extern uint32_t ISO_STATUS_ADDR;

extern void Flash_EraseAt(uint32_t FlashAddress, uint32_t length);
void Flash_Erase(uint32_t address);
uint8_t Flash_IsErased(uint32_t startAddr, uint32_t length);

void Flash_Write_uint8(uint32_t address, uint8_t value);
void Flash_Write_uint16(uint32_t address, uint16_t value);
void Flash_Write_int16(uint32_t address, int16_t value);

void Flash_Read_int16(uint32_t address, int16_t *value);
void Flash_Read_uint16(uint32_t address, uint16_t *value);
void Flash_Read_uint8(uint32_t address, uint8_t *value);

/* Flash memory page base address definitions for STM32U073 */
#define ADDR_FLASH_PAGE_0    ((uint32_t)0x08000000) /* Base address of Page 0, 2 Kbytes */
#define ADDR_FLASH_PAGE_1    ((uint32_t)0x08000800) /* Base address of Page 1, 2 Kbytes */
#define ADDR_FLASH_PAGE_2    ((uint32_t)0x08001000) /* Base address of Page 2, 2 Kbytes */
#define ADDR_FLASH_PAGE_3    ((uint32_t)0x08001800) /* Base address of Page 3, 2 Kbytes */
#define ADDR_FLASH_PAGE_4    ((uint32_t)0x08002000) /* Base address of Page 4, 2 Kbytes */
#define ADDR_FLASH_PAGE_5    ((uint32_t)0x08002800) /* Base address of Page 5, 2 Kbytes */
#define ADDR_FLASH_PAGE_6    ((uint32_t)0x08003000) /* Base address of Page 6, 2 Kbytes */
#define ADDR_FLASH_PAGE_7    ((uint32_t)0x08003800) /* Base address of Page 7, 2 Kbytes */
#define ADDR_FLASH_PAGE_8    ((uint32_t)0x08004000) /* Base address of Page 8, 2 Kbytes */
#define ADDR_FLASH_PAGE_9    ((uint32_t)0x08004800) /* Base address of Page 9, 2 Kbytes */
#define ADDR_FLASH_PAGE_10   ((uint32_t)0x08005000) /* Base address of Page 10, 2 Kbytes */
#define ADDR_FLASH_PAGE_11   ((uint32_t)0x08005800) /* Base address of Page 11, 2 Kbytes */
#define ADDR_FLASH_PAGE_12   ((uint32_t)0x08006000) /* Base address of Page 12, 2 Kbytes */
#define ADDR_FLASH_PAGE_13   ((uint32_t)0x08006800) /* Base address of Page 13, 2 Kbytes */
#define ADDR_FLASH_PAGE_14   ((uint32_t)0x08007000) /* Base address of Page 14, 2 Kbytes */
#define ADDR_FLASH_PAGE_15   ((uint32_t)0x08007800) /* Base address of Page 15, 2 Kbytes */
#define ADDR_FLASH_PAGE_16   ((uint32_t)0x08008000) /* Base address of Page 16, 2 Kbytes */
#define ADDR_FLASH_PAGE_17   ((uint32_t)0x08008800) /* Base address of Page 17, 2 Kbytes */
#define ADDR_FLASH_PAGE_18   ((uint32_t)0x08009000) /* Base address of Page 18, 2 Kbytes */
#define ADDR_FLASH_PAGE_19   ((uint32_t)0x08009800) /* Base address of Page 19, 2 Kbytes */
#define ADDR_FLASH_PAGE_20   ((uint32_t)0x0800A000) /* Base address of Page 20, 2 Kbytes */
#define ADDR_FLASH_PAGE_21   ((uint32_t)0x0800A800) /* Base address of Page 21, 2 Kbytes */
#define ADDR_FLASH_PAGE_22   ((uint32_t)0x0800B000) /* Base address of Page 22, 2 Kbytes */
#define ADDR_FLASH_PAGE_23   ((uint32_t)0x0800B800) /* Base address of Page 23, 2 Kbytes */
#define ADDR_FLASH_PAGE_24   ((uint32_t)0x0800C000) /* Base address of Page 24, 2 Kbytes */
#define ADDR_FLASH_PAGE_25   ((uint32_t)0x0800C800) /* Base address of Page 25, 2 Kbytes */
#define ADDR_FLASH_PAGE_26   ((uint32_t)0x0800D000) /* Base address of Page 26, 2 Kbytes */
#define ADDR_FLASH_PAGE_27   ((uint32_t)0x0800D800) /* Base address of Page 27, 2 Kbytes */
#define ADDR_FLASH_PAGE_28   ((uint32_t)0x0800E000) /* Base address of Page 28, 2 Kbytes */
#define ADDR_FLASH_PAGE_29   ((uint32_t)0x0800E800) /* Base address of Page 29, 2 Kbytes */
#define ADDR_FLASH_PAGE_30   ((uint32_t)0x0800F000) /* Base address of Page 30, 2 Kbytes */
#define ADDR_FLASH_PAGE_31   ((uint32_t)0x0800F800) /* Base address of Page 31, 2 Kbytes */
#define ADDR_FLASH_PAGE_32   ((uint32_t)0x08010000) /* Base address of Page 32, 2 Kbytes */
#define ADDR_FLASH_PAGE_33   ((uint32_t)0x08010800) /* Base address of Page 33, 2 Kbytes */
#define ADDR_FLASH_PAGE_34   ((uint32_t)0x08011000) /* Base address of Page 34, 2 Kbytes */
#define ADDR_FLASH_PAGE_35   ((uint32_t)0x08011800) /* Base address of Page 35, 2 Kbytes */
#define ADDR_FLASH_PAGE_36   ((uint32_t)0x08012000) /* Base address of Page 36, 2 Kbytes */
#define ADDR_FLASH_PAGE_37   ((uint32_t)0x08012800) /* Base address of Page 37, 2 Kbytes */
#define ADDR_FLASH_PAGE_38   ((uint32_t)0x08013000) /* Base address of Page 38, 2 Kbytes */
#define ADDR_FLASH_PAGE_39   ((uint32_t)0x08013800) /* Base address of Page 39, 2 Kbytes */
#define ADDR_FLASH_PAGE_40   ((uint32_t)0x08014000) /* Base address of Page 40, 2 Kbytes */
#define ADDR_FLASH_PAGE_41   ((uint32_t)0x08014800) /* Base address of Page 41, 2 Kbytes */
#define ADDR_FLASH_PAGE_42   ((uint32_t)0x08015000) /* Base address of Page 42, 2 Kbytes */
#define ADDR_FLASH_PAGE_43   ((uint32_t)0x08015800) /* Base address of Page 43, 2 Kbytes */
#define ADDR_FLASH_PAGE_44   ((uint32_t)0x08016000) /* Base address of Page 44, 2 Kbytes */
#define ADDR_FLASH_PAGE_45   ((uint32_t)0x08016800) /* Base address of Page 45, 2 Kbytes */
#define ADDR_FLASH_PAGE_46   ((uint32_t)0x08017000) /* Base address of Page 46, 2 Kbytes */
#define ADDR_FLASH_PAGE_47   ((uint32_t)0x08017800) /* Base address of Page 47, 2 Kbytes */
#define ADDR_FLASH_PAGE_48   ((uint32_t)0x08018000) /* Base address of Page 48, 2 Kbytes */
#define ADDR_FLASH_PAGE_49   ((uint32_t)0x08018800) /* Base address of Page 49, 2 Kbytes */
#define ADDR_FLASH_PAGE_50   ((uint32_t)0x08019000) /* Base address of Page 50, 2 Kbytes */
#define ADDR_FLASH_PAGE_51   ((uint32_t)0x08019800) /* Base address of Page 51, 2 Kbytes */
#define ADDR_FLASH_PAGE_52   ((uint32_t)0x0801A000) /* Base address of Page 52, 2 Kbytes */
#define ADDR_FLASH_PAGE_53   ((uint32_t)0x0801A800) /* Base address of Page 53, 2 Kbytes */
#define ADDR_FLASH_PAGE_54   ((uint32_t)0x0801B000) /* Base address of Page 54, 2 Kbytes */
#define ADDR_FLASH_PAGE_55   ((uint32_t)0x0801B800) /* Base address of Page 55, 2 Kbytes */
#define ADDR_FLASH_PAGE_56   ((uint32_t)0x0801C000) /* Base address of Page 56, 2 Kbytes */
#define ADDR_FLASH_PAGE_57   ((uint32_t)0x0801C800) /* Base address of Page 57, 2 Kbytes */
#define ADDR_FLASH_PAGE_58   ((uint32_t)0x0801D000) /* Base address of Page 58, 2 Kbytes */
#define ADDR_FLASH_PAGE_59   ((uint32_t)0x0801D800) /* Base address of Page 59, 2 Kbytes */
#define ADDR_FLASH_PAGE_60   ((uint32_t)0x0801E000) /* Base address of Page 60, 2 Kbytes */
#define ADDR_FLASH_PAGE_61   ((uint32_t)0x0801E800) /* Base address of Page 61, 2 Kbytes */
#define ADDR_FLASH_PAGE_62   ((uint32_t)0x0801F000) /* Base address of Page 62, 2 Kbytes */
#define ADDR_FLASH_PAGE_63   ((uint32_t)0x0801F800) /* Base address of Page 63, 2 Kbytes */



#endif
