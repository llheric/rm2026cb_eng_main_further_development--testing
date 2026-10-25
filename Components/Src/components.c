#include "components.h"


void Components_Init()
{
	CAN_Driver_Init();
	PWM_Init();
	LED_Init();
	Controller_Init();
	IMU_Init();
	DJIMotor_General_Init();
	DMMotor_General_Init();
	USB_Init();
	Laser_5V_Enable();
}
