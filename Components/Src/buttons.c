#include "buttons.h"

button_group_t btn_group = {0};

void create_int8_btn(button_t *btn, int8_t *raw) {
	if(btn == NULL || raw == NULL) return ;
	
	if(btn_group.size < MAX_BTNS) {
		*btn = (button_t){
			.raw = (data_t){raw, TYPE_INT8},
			.rise = 0, .fall = 0, .level = 0, .ready = 0
		};
		btn_group.btns[btn_group.size++] = btn;
	}
}

void create_uint8_btn(button_t *btn, uint8_t *raw) {
	if(btn == NULL || raw == NULL) return ;
	
	if(btn_group.size < MAX_BTNS) {
		*btn = (button_t){
			.raw = (data_t){raw, TYPE_UINT8},
			.rise = 0, .fall = 0, .level = 0, .ready = 0
		};
		btn_group.btns[btn_group.size++] = btn;
	}
}

void create_int16_btn(button_t *btn, int16_t *raw) {
	if(btn == NULL || raw == NULL) return ;
	
	if(btn_group.size < MAX_BTNS) {
		*btn = (button_t){
			.raw = (data_t){raw, TYPE_INT16},
			.rise = 0, .fall = 0, .level = 0, .ready = 0
		};
		btn_group.btns[btn_group.size++] = btn;
	}
}

void create_uint16_btn(button_t *btn, uint16_t *raw) {
	if(btn == NULL || raw == NULL) return ;
	
	if(btn_group.size < MAX_BTNS) {
		*btn = (button_t){
			.raw = (data_t){raw, TYPE_UINT16},
			.rise = 0, .fall = 0, .level = 0, .ready = 0
		};
		btn_group.btns[btn_group.size++] = btn;
	}
}

uint8_t get_data_value(const data_t *raw) {
	if(raw->data == NULL) return 0;
	switch (raw->type) {
		case TYPE_INT8: 	return (uint8_t)*(int8_t*)raw->data;
		case TYPE_UINT8: 	return (uint8_t)*(uint8_t*)raw->data;
		case TYPE_INT16: 	return (uint8_t)*(int16_t*)raw->data;
		case TYPE_UINT16: return (uint8_t)*(uint16_t*)raw->data;
		case TYPE_INT32: 	return (uint8_t)*(int32_t*)raw->data;
		case TYPE_UINT32: return (uint8_t)*(uint32_t*)raw->data;
		default:
			return 0;
	}
}

void update_button_status(void) {
	for(uint16_t i = 0; i < btn_group.size; i++) {
		button_t *btn = btn_group.btns[i];
		uint8_t raw_data = get_data_value(&btn->raw);
		
		if(raw_data == 1 && btn->level == 0) btn->rise = 1;
		else btn->rise = 0;
		
		if(raw_data == 0 && btn->level == 1) btn->fall = 1;
		else btn->fall = 0;
		
		btn->level = raw_data;
	}
}


