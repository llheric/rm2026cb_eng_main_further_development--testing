#ifndef _BOARD_COMMUNICATE_H
#define _BOARD_COMMUNICATE_H

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "string.h"
#include "can_driver.h"
#include "communicate.h"

#define COMM_USE_CAN 	1
#define MAX_PACKERS 	10
#define MAX_HANDLERS 	60

#define PRIO_LOW_CYCLE 53
#define PRIO_NORM_CYCLE 31
#define PRIO_HIGH_CYCLE 17 // suitable for controller
#define PRIO_COUNT 3

typedef enum {
	PRIO_LOW	= 0,
	PRIO_NORM = 1,
	PRIO_HIGH = 2,
} comm_prio_t;


typedef void (*packer_t)(uint8_t *buffer);
typedef void (*handler_t)(uint8_t *data);

typedef struct {
	uint16_t cmd_id;
	packer_t packer;
} comm_packer_t;

typedef struct {
	uint16_t cmd_id;
	handler_t handler;
} comm_handler_t;

typedef struct {
	comm_packer_t list[MAX_PACKERS];
	uint8_t count;
	uint16_t cycle_cnt;
	uint16_t cycle_lim;
} comm_prio_group_t;

void board_communicate_init(void);

void comm_packer_register(uint16_t cmd_id, packer_t packer, comm_prio_t prio);
void comm_handler_register(uint16_t cmd_id, handler_t handler);

void comm_sender(void);
void comm_receiver(uint16_t cmd_id, uint8_t *data);

#endif
