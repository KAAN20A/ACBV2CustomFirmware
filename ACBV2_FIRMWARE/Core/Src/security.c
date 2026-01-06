#include "security.h"

HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {
    focparameter.board_temperature = calculateBoardTemperature();
    focparameter.bus_voltage = calculateBusVoltage();
    focparameter.internal_temperature = calculateInternalTemperature();
  }
}

float calculateBoardTemperature() {
  uint32_t adc_value = 0;
  float voltage = 0.0f;

  uint32_t adc_value = ReadADC_Channel(ADC_CHANNEL_1);

  float temprature = ((float)adc_value / 4095.0f) * 3.3f;

  return voltage;
}

uint32_t ReadADC_Channel(uint32_t channel) {
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10);

  uint32_t value = HAL_ADC_GetValue(&hadc1);

  HAL_ADC_Stop(&hadc1);

  return value;
}
float calculateBusVoltage(void) {
  uint32_t adc_value = ReadADC_Channel(ADC_CHANNEL_2);

  float voltage = ((float)adc_value / 4095.0f) * 3.3f;

  return voltage;
}
float calculateInternalTemperature(void) {
  uint32_t raw_temp = ReadADC_Channel(ADC_CHANNEL_TEMPSENSOR);
  uint32_t raw_vref = ReadADC_Channel(ADC_CHANNEL_VREFINT);

  float temp_voltage = ((float)raw_temp / 4095.0f) * 3.3f;
  float vref_voltage = ((float)raw_vref / 4095.0f) * 3.3f;

  return temp_voltage;
}
