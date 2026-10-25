#ifndef _COMPONENT_PWM_H
#define _COMPONENT_PWM_H
#define PWM_MAX_NUMBER 14
#define PWM_DEFAULT_FREQUENCY 400
#define PWM_DEFAULT_ARR 2500

#include "main.h"
#include <stdbool.h>

typedef struct {
	TIM_HandleTypeDef *  port;
	uint32_t             frequency;
	uint8_t              channel;
	float                dutyRatio;
	bool                 enabled;
} PWM;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;

extern PWM pwms[];
HAL_StatusTypeDef PWM_Init(void);
HAL_StatusTypeDef PWM_DeInit(void);
HAL_StatusTypeDef PWM_Start(const PWM * pwm);
HAL_StatusTypeDef PWM_Stop(const PWM * pwm);
HAL_StatusTypeDef PWM_SetPulse(const PWM * pwm, uint32_t value);
HAL_StatusTypeDef PWM_SetDutyRatio(PWM * pwm, float ratio);
HAL_StatusTypeDef PWM_SetFrequency(PWM * pwm, uint32_t frequency);
HAL_StatusTypeDef PWM_SetPulseLength(PWM * pwm, uint32_t us);

#endif
