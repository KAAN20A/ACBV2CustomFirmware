#ifndef SECURITY_H
#define SECURITY_H

#ifdef __cplusplus
extern "C" {
#endif


#include"main.h"

#define ADC_REGULAR_RANK_1 1U

#define ADC_SAMPLETIME_239CYCLES_5 0x07U

#define ADC_CHANNEL_TEMPSENSOR 16U
#define ADC_CHANNEL_VREFINT 17U

float calculateBoardTemperature(void);
uint32_t ReadADC_Channel(uint32_t channel);
float calculateBusVoltage(void);
float calculateInternalTemperature(void);


#ifdef __cplusplus
}
#endif

#endif
