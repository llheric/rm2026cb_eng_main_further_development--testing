#ifndef _COMPONENTS_LED_H
#define _COMPONENTS_LED_H
#include "stdint.h"

extern uint8_t LED_Override;

void LED_Init(void);
void LED_SetRGB(uint16_t R_value, uint16_t G_value, uint16_t B_value);
void LED_Off(void);
#endif
