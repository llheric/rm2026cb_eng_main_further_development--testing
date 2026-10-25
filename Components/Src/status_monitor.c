#include "status_monitor.h"
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;


volatile float voltage_vrefint_proportion = 8.0586080586080586080586080586081e-4f;
struct Core_Status core;

static float calc_battery_percentage(float voltage);

static uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
	static ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ch;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;//ADC_SAMPLETIME_3CYCLES;

	if (HAL_ADC_ConfigChannel(ADCx, &sConfig) != HAL_OK)
	{
		return 0;
	}

	HAL_ADC_Start(ADCx);

	HAL_ADC_PollForConversion(ADCx, 10);
	return (uint16_t)HAL_ADC_GetValue(ADCx);

}
void Status_Init(void)
{
	uint8_t i = 0;
	uint32_t total_adc = 0;
	for(i = 0; i < 200; i++)
	{
		total_adc += adcx_get_chx_value(&hadc1, ADC_CHANNEL_VREFINT);
	}

	voltage_vrefint_proportion = 200 * 1.2f / total_adc;
}
void get_temprate(void)
{
	uint16_t adcx = 0;

	adcx = adcx_get_chx_value(&hadc1, ADC_CHANNEL_TEMPSENSOR);
	core.core_temp = (float)adcx * voltage_vrefint_proportion;
	core.core_temp = (core.core_temp - 0.76f) * 400.0f + 25.0f;

}


void get_battery_voltage(void)
{
	uint16_t adcx = 0;

	adcx = adcx_get_chx_value(&hadc3, ADC_CHANNEL_8);
	//(22K ¦¸ + 200K ¦¸)  / 22K ¦¸ = 10.090909090909090909090909090909
	core.batt_voltage =  (float)adcx * voltage_vrefint_proportion * 10.090909090909090909090909090909f;
	core.percentage_6s = calc_battery_percentage(core.batt_voltage) * 100.0f;
}

static float calc_battery_percentage(float voltage)
{
	float percentage;
	float voltage_2 = voltage * voltage;
	float voltage_3 = voltage_2 * voltage;
	
	if(voltage < 19.5f)
	{
		percentage = 0.0f;
	}
	else if(voltage < 21.9f)
	{
		percentage = 0.005664f * voltage_3 - 0.3386f * voltage_2 + 6.765f * voltage - 45.17f;
	}
	else if(voltage < 25.5f)
	{
		percentage = 0.02269f * voltage_3 - 1.654f * voltage_2 + 40.34f * voltage - 328.4f;
	}
	else
	{
		percentage = 1.0f;
	}
	if(percentage < 0.0f)
	{
		percentage = 0.0f;
	}
	else if(percentage > 1.0f)
	{
		percentage = 1.0f;
	}

	return percentage;
}

uint8_t get_hardware_version(void)
{
	uint8_t hardware_version;
	hardware_version = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)
									| (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)<<1)
									| (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)<<2);



	return hardware_version;
}
