#include "board_communicate.h"

extern CAN_HandleTypeDef hcan1, hcan2;
extern UART_HandleTypeDef huart1, huart3, huart6;

comm_prio_group_t packer_groups[PRIO_COUNT];
comm_handler_t handlers[MAX_HANDLERS];
uint16_t handler_count = 0;

CAN_HandleTypeDef* use_can;

void board_communicate_init(void) {
	uint16_t lims[PRIO_COUNT] = {PRIO_LOW_CYCLE, PRIO_NORM_CYCLE, PRIO_HIGH_CYCLE};
	for (uint8_t i = 0; i < PRIO_COUNT; i++) {
		packer_groups[i].count = 0;
		packer_groups[i].cycle_cnt = 0;
		packer_groups[i].cycle_lim = lims[i];
	}
	use_can = &hcan2;
}

void comm_packer_register(uint16_t cmd_id, packer_t packer, comm_prio_t prio) {
	if (prio >= PRIO_COUNT) return;
	
	comm_prio_group_t *group = &packer_groups[prio];
	if (group->count < MAX_PACKERS) {
		group->list[group->count].cmd_id = cmd_id;
		group->list[group->count].packer = packer;
		group->count ++;
	}
}

void comm_handler_register(uint16_t cmd_id, handler_t handler) {
	if(handler_count >= MAX_HANDLERS) return ;
	
	handlers[handler_count] = (comm_handler_t){.cmd_id = cmd_id, .handler = handler};
	handler_count ++;
}

void CAN_comm_sender(CAN_HandleTypeDef *hcan, comm_prio_group_t *group) {
	static uint8_t buf[8];
	for (uint16_t i = 0; i < group->count; i++) {
		uint32_t std_id = group->list[i].cmd_id;
		group->list[i].packer(buf);
		
		CAN_Driver_Send(hcan, std_id, 8, buf);
	}
}

void comm_sender(void) {
	#if COMM_USE_CAN
	
	for (uint8_t i = 0; i < PRIO_COUNT; i++) {
		if (packer_groups[i].cycle_cnt >= packer_groups[i].cycle_lim) {
			CAN_comm_sender(use_can, &packer_groups[i]);
			packer_groups[i].cycle_cnt = 0;
		}
		else packer_groups[i].cycle_cnt ++;
	}
	
	#endif
}

void comm_receiver(uint16_t cmd_id, uint8_t *data) {
	for(uint16_t i = 0; i < MAX_HANDLERS; i ++){
		if(handlers[i].cmd_id == cmd_id){
			handlers[i].handler(data);
			return ;
		}
	}
}

