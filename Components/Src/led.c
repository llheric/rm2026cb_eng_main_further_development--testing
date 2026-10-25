#include "led.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "os.h"

typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;

extern TIM_HandleTypeDef htim5;
#define LED_HAL_TIM htim5
#define LED_LL_TIM TIM5
#define LED_R_CH TIM_CHANNEL_3
#define LED_G_CH TIM_CHANNEL_2
#define LED_B_CH TIM_CHANNEL_1

// Change this to 1 to set custom LED color and radiance
uint8_t LED_Override = 0;

void LED_SetRGB(uint16_t R_value, uint16_t G_value, uint16_t B_value){
	LED_Override = 1;
	LED_LL_TIM->CCR1 = B_value;
	LED_LL_TIM->CCR2 = G_value;
	LED_LL_TIM->CCR3 = R_value;
}

void LED_Off(){
	LED_LL_TIM->CCR1 = 0;
	LED_LL_TIM->CCR2 = 0;
	LED_LL_TIM->CCR3 = 0;
}

void LED_Init(){
	/* Set PWM output */
  HAL_TIM_PWM_Start(&LED_HAL_TIM, LED_R_CH);
  HAL_TIM_PWM_Start(&LED_HAL_TIM, LED_G_CH);
  HAL_TIM_PWM_Start(&LED_HAL_TIM, LED_B_CH);
}
