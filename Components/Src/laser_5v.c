#include "laser_5v.h"
#include "main.h"

void Laser_5V_Enable(){
	HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
}

void Laser_5V_Disable(){
	HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
}
