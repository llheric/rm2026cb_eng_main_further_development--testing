#ifndef _VOLTAGE_H
#define _VOLTAGE_H
#include "stm32f4xx.h"

struct Core_Status{
	float batt_voltage;
	float core_temp;
	float percentage_6s;
};

extern struct Core_Status core;

extern void Status_Init(void);
extern void get_temprate(void);
extern void get_battery_voltage(void);
extern uint8_t get_hardware_version(void);
#endif 
