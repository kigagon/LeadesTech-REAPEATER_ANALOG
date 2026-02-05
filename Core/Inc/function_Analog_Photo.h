#ifndef FUNCTION_ANALOGE_PHOTO_H_
#define FUNCTION_ANALOGE_PHOTO_H_

extern int16_t Prev_vshunt, Prev_shunt_I;
extern uint8_t FinalPcntVal_Required4SlopeCalculation;
extern double RedefinedSlope;
extern int16_t Adjust_Y_Intercept_Value;
extern uint8_t UpTo22, Above22;
extern double ADCMappingTable_Below22Percent[256];
extern uint8_t PhotoStatus;
extern uint8_t PGA;
extern uint8_t DAC_10multiplied;
extern float DAC_VALUE, DAC_VALUE_TEMP;
extern int16_t raw;

extern void AverageCurrentValues(void);
extern void V_0to3_checkShuntCurrentValues(void);
//extern int8_t CheckLED_IsShortOrOpen(void);
extern uint8_t CheckLED_IsShortOrOpen(ADC_HandleTypeDef* pAdcHandle, int32_t finalSensorValue);
uint16_t LED_X_ADC_VALUE(ADC_HandleTypeDef* pAdcHandle, uint8_t num_samples);
uint16_t LED_O_ADC_VALUE(ADC_HandleTypeDef* pAdcHandle, float volt, uint8_t num_samples);
uint16_t ReadLED_O_value_ToSetPGA(uint8_t gain, float volt, uint8_t num);
extern void SetPGA(float volt, uint8_t num);
uint16_t Return_Ana_photo_Sen_12bit(uint8_t gain, float volt, uint8_t num);


uint16_t ReturnToPercent_afterReadAdcValue(uint8_t gain, float volt, uint8_t num, uint8_t set);
uint8_t MatchAdcValueToPercentValue(const double *arr, uint16_t ADCValue);

void RedefineSmokeTableWithADC(uint8_t ReferenceArr, double slope);
void ZeroAdjustment(void);
uint8_t firstCalibration(uint8_t pga);
void saveDACvalueInFlash(uint8_t index, uint8_t pga);
extern void AdjustmentCmdPrcessing(uint8_t cmd);


#endif
