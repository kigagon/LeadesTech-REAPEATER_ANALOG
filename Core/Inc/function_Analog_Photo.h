#ifndef FUNCTION_ANALOG_PHOTO_H_
#define FUNCTION_ANALOG_PHOTO_H_

extern float DAC_VALUE, DAC_VALUE_TEMP;
extern uint8_t PGA;

extern uint16_t Ana_photo_O_Sen_12bit, Ana_photo_X_Sen_12bit, Flash_Ana_photo_X_Sen_12bit, Ana_photo_Sen_12bit;
extern double RedefinedSlope;
extern int16_t Adjust_Y_Intercept_Value;
extern uint8_t FirstUpperLimitOfPercent, FinalUpperLimitOfPercent;

extern int16_t UserAdjustedPercent;
extern uint8_t FinalPcntVal_Required4SlopeCalculation;
extern uint16_t photoADC_Required4SlopeCalculation;

extern uint8_t PhotoStatus;
extern uint8_t ReturnPercent;

extern double AdcToPercent_MappingTable[256];

#define MAX_ADC_SAMPLES 50


// First Setting
extern void SetSMOKE(void);

// Adc
uint16_t LED_X_ADC_VALUE(ADC_HandleTypeDef* pAdcHandle, uint8_t num_samples);
uint16_t LED_O_ADC_VALUE(ADC_HandleTypeDef* pAdcHandle, float volt, uint8_t num_samples);
uint16_t Return_Ana_photo_Sen_12bit(uint8_t gain, float volt, uint8_t num);

// Make MappingTable And Return PercentValue
void RedefineSmokeTableWithADC(double slope);
uint8_t MappingTableSetting_UsingOneSlope(double slope);
uint8_t MappingTableSetting_UsingTwoSlope(double slope);
uint8_t MatchAdcValueToPercentValue(const double *arr, uint16_t ADCValue);

// Adjustment work
uint8_t firstCalibration(uint8_t pga);
void saveDACvalueInFlash(uint8_t index, uint8_t pga);
void AdjustmentCmdPrcessing(uint8_t cmd);


#endif
