#ifndef _CAN_DRIVER_H
#define _CAN_DRIVER_H

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "string.h"

#define ESC_BASE_ID 0x200

#define CAN_TX_QUEUE_SIZE 64 // actually only CAN_QUEUE_LENGTH - 1 spaces can be used
#define CAN2_FILTER_START_BANK 14

typedef enum {
	CAN_TX_OK,
	CAN_TX_QUEUE_EMPTY,
	CAN_TX_QUEUE_FULL,  // Queue is full, message not sent
	CAN_TX_ERROR,
} can_tx_status_t;

typedef struct {
	uint32_t std_id;
	uint8_t  dlc;        // Data length code (0-8)
	uint8_t  buf[8];
} can_tx_msg_t;

typedef struct {
	can_tx_msg_t msgs[CAN_TX_QUEUE_SIZE];
	volatile uint8_t head;
	volatile uint8_t tail; // pointer to next empty space of the queue
	CAN_HandleTypeDef *hcan;
	volatile uint32_t overflow_count;  // Count of messages dropped due to queue full
	volatile uint32_t error_count;     // Count of transmission errors
} can_tx_queue_t;

void CAN_Driver_Init(void);
can_tx_status_t CAN_Driver_Send(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t dlc, uint8_t buf[8]);


#endif
