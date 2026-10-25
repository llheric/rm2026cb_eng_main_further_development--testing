#include "controller.h"
#include "os.h"
#include "communicate.h"
#include "comm.h"
#include "main.h"
#include "stm32f4xx_hal_uart.h"

extern UART_HandleTypeDef huart3;
#define CONTROLLER_UART cuart3

ControlData control = {0};
volatile uint32_t _lastRCPacketTime = 0, _lastSwitchSourceTime = 0;
volatile uint8_t rc_buf[RC_BUF_SIZE];

osThreadId_t unpackerId;
#define RC_DATA_PENDING 0x0001

void __SignalSource_Controller_ReInit(){
	HAL_GPIO_WritePin(SIGNAL_SELECT_GPIO_Port, SIGNAL_SELECT_Pin, GPIO_PIN_RESET);
	huart3.Init.BaudRate = 100000;
  huart3.Init.WordLength = UART_WORDLENGTH_9B;
  huart3.Init.Parity = UART_PARITY_EVEN;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

void __SignalSource_VideoTransmitter_Reinit(){
	HAL_GPIO_WritePin(SIGNAL_SELECT_GPIO_Port, SIGNAL_SELECT_Pin, GPIO_PIN_SET);
	huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.Parity = UART_PARITY_NONE;
	if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

void __SignalSource_DeInit(){
	HAL_UART_DeInit(CONTROLLER_UART.huart);
}

inline static void avoid_deadzone_float(float* value, float range) {
  if (-range < *value && *value < range) *value = 0;
}

void unpackData(uint8_t volatile *data) {
	control.channel[0] = ((uint16_t)(((data[0] | (data[1] << 8))) & 0x7ff) - 1024) / 660.0f;
	control.channel[1] = ((uint16_t)(((data[1] >> 3) | (data[2] << 5)) & 0x7ff) - 1024) / 660.0f;
	control.channel[2] = ((uint16_t)(((data[2] >> 6) | (data[3] << 2) | data[4] << 10) & 0x7ff) - 1024) / 660.0f;
	control.channel[3] = ((uint16_t)(((data[4] >> 1) | (data[5] << 7)) & 0x7ff) - 1024) / 660.0f;
	avoid_deadzone_float(&control.channel[0], 0.05f);
	avoid_deadzone_float(&control.channel[1], 0.05f);
	avoid_deadzone_float(&control.channel[2], 0.05f);
	avoid_deadzone_float(&control.channel[3], 0.05f);
	control.triSwitch[0] = (data[5] >> 6) & 0x3;
	control.triSwitch[1] = (data[5] >> 4) & 0x34;
	control.mice.x = data[6] | (data[7] << 8);
	control.mice.y = data[8] | (data[9] << 8);
	control.mice.z = data[10] | (data[11] << 8);
	control.mice.leftButton = data[12];
	control.mice.rightButton = data[13];
	control.key = data[14] | (data[15] << 8);
	control.roller1 = data[16];
	control.roller2 = data[17];
}

inline static void unpackCommand_VideoTrasmitter(uint8_t volatile* data, ControlData* control){
	memset(control->channel, 0, sizeof(float) * 4);
	control->triSwitch[L] = DOWN;
	control->triSwitch[R] = MID;
	control->mice.x = ((ext_robot_command_t*)data)->mouse_x;
	control->mice.y = ((ext_robot_command_t*)data)->mouse_y;
	control->mice.z = ((ext_robot_command_t*)data)->mouse_z;
	control->mice.leftButton = ((ext_robot_command_t*)data)->left_button_down;
	control->mice.rightButton = ((ext_robot_command_t*)data)->right_button_down;
	control->key = ((ext_robot_command_t*)data)->keyboard_value;
}


void notifyRCDataUnpacker() {
  osThreadFlagsSet(unpackerId, RC_DATA_PENDING);
}

inline static void Controller_Start_Rx() {
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart3, (uint8_t*) rc_buf, RC_BUF_SIZE);
}

inline static void receiveVideotransmitterData(uint8_t * data){
	memcpy((void *)rc_buf, data, sizeof(ext_robot_command_t));
	_lastRCPacketTime = HAL_GetTick();
	notifyRCDataUnpacker();
}

void RCDataUnpacker(void *_) {
  uint32_t _lastRCUnpackTime = 0;
  Controller_Start_Rx();
	control.signalSource=SIGNAL_VIDEOTRANSMITTER;
  while (1) {
    control.online = (_lastRCPacketTime > _lastRCUnpackTime);
    if (control.online) {
      _lastRCUnpackTime = time();
			if(control.signalSource == SIGNAL_CONTROLLER)
				unpackData(rc_buf);
			else
				unpackCommand_VideoTrasmitter(rc_buf, &control);
		}
		/*
		if(HAL_GetTick() - _lastRCPacketTime >= 1000 && time() - _lastSwitchSourceTime >= 1000){
			_lastSwitchSourceTime = time();
			__SignalSource_DeInit();
			if(control.signalSource == SIGNAL_CONTROLLER){
				control.signalSource = SIGNAL_VIDEOTRANSMITTER;
				__SignalSource_VideoTransmitter_Reinit();
			}else{
				control.signalSource = SIGNAL_CONTROLLER;
				__SignalSource_Controller_ReInit();
			}
		}
		*/
    /* the manual says frame interval is 14ms */
    osThreadFlagsWait(RC_DATA_PENDING, osFlagsWaitAny, 100);
	}
}

void Controller_Init() {
	protocol_register(CMD_VT_CONTROL, receiveVideotransmitterData);
  unpackerId = osThreadCreate("RC", RCDataUnpacker, NULL, osPriorityAboveNormal, 64);
}
