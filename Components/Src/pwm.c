#ifndef _COMPONENT_PWM_C
#define _COMPONENT_PWM_C
#include "pwm.h"
PWM  pwms[PWM_MAX_NUMBER] = {
	[1] = {
		.port        = &htim1,
		.channel     = TIM_CHANNEL_1,
		.enabled     = 1
	},
	[2] = {
		.port        = &htim1,
		.channel     = TIM_CHANNEL_2,
		.enabled     = 1
	},
	[3] = {
		.port        = &htim1,
		.channel     = TIM_CHANNEL_3,
		.enabled     = 1
	},
	[4] = {
		.port        = &htim1,
		.channel     = TIM_CHANNEL_4,
		.enabled     = 1
	},
	[5] = {
		.port        = &htim8,
		.channel     = TIM_CHANNEL_1,
		.enabled     = 1
	},
	[6] = {
		.port        = &htim8,
		.channel     = TIM_CHANNEL_2,
		.enabled     = 1
	}
};
void * pwmsEnd = pwms + PWM_MAX_NUMBER;
HAL_StatusTypeDef PWM_Init(void) {
	for (PWM* pwm = pwms; (void*) pwm != pwmsEnd; ++pwm) {
		if (pwm->enabled) { 
			PWM_Start(pwm);
			uint32_t pluse;
			switch (pwm->channel) {
			  case TIM_CHANNEL_1:
				  pluse = pwm->port->Instance->CCR1;
				  break;
			  case TIM_CHANNEL_2:
				  pluse = pwm->port->Instance->CCR2;
				  break;
			  case TIM_CHANNEL_3:
				  pluse = pwm->port->Instance->CCR3;
				  break;
			  case TIM_CHANNEL_4:
				  pluse = pwm->port->Instance->CCR4;
				  break;
		  }
			pwm->frequency = PWM_DEFAULT_FREQUENCY;
			pwm->dutyRatio = (float) pluse / (pwm->port->Instance->ARR+1);
		}
	}
	return HAL_OK;
}

HAL_StatusTypeDef PWM_DeInit(void) {
	return HAL_OK;
}

HAL_StatusTypeDef PWM_Start(const PWM * pwm) {
	return HAL_TIM_PWM_Start(pwm->port, pwm->channel);
}

HAL_StatusTypeDef PWM_Stop(const PWM * pwm) {
	return HAL_TIM_PWM_Stop(pwm->port, pwm->channel);
}

inline static HAL_StatusTypeDef PWM_SetARR(const PWM * pwm, uint32_t value) {
	pwm->port->Instance->ARR = value;
	return HAL_OK;
}

inline HAL_StatusTypeDef PWM_SetPulse(const PWM * pwm, uint32_t value) {
	if (pwm->channel == TIM_CHANNEL_1) pwm->port->Instance->CCR1 = value;
	if (pwm->channel == TIM_CHANNEL_2) pwm->port->Instance->CCR2 = value;
	if (pwm->channel == TIM_CHANNEL_3) pwm->port->Instance->CCR3 = value;
	if (pwm->channel == TIM_CHANNEL_4) pwm->port->Instance->CCR4 = value;
	return HAL_OK;
}

HAL_StatusTypeDef PWM_SetDutyRatio(PWM * pwm, float ratio) {
	if (ratio < 0 || ratio > 1) return HAL_ERROR;
	pwm->dutyRatio = ratio;
	PWM_SetPulse(pwm, ratio * (pwm->port->Instance->ARR+1));
	return HAL_OK;
}

HAL_StatusTypeDef PWM_SetFrequency(PWM * pwm, uint32_t frequency) {
	PWM_SetARR(pwm, PWM_DEFAULT_FREQUENCY * PWM_DEFAULT_ARR / frequency - 1);
	pwm->frequency = frequency;
	return HAL_OK;
}

HAL_StatusTypeDef PWM_SetPulseLength(PWM * pwm, uint32_t us) {
	return PWM_SetDutyRatio(pwm, us * 1e-6 * pwm->frequency);
}
#endif
