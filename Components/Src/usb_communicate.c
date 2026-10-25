#include "usb_communicate.h"

usb_packer_registry_t  packer_reg;
usb_handler_registry_t handler_reg;

void usb_communicate_init(void) {
	memset(&packer_reg,  0, sizeof(usb_packer_registry_t));
	memset(&handler_reg, 0, sizeof(usb_handler_registry_t));
}

uint8_t usb_packer_register(uint16_t cmd_id, uint16_t length, packer_t packer) {
	if (packer_reg.count >= USB_PACKERS_MAX) return 0;
	
	packer_reg.list[packer_reg.count ++] = (usb_packer_t){
		.cmd_id = cmd_id, .length = length, .packer = packer,
	};
	return 1;
}

uint8_t usb_handler_register(uint16_t cmd_id, handler_t handler) {
	if (handler_reg.count >= USB_HANDLERS_MAX) return 0;
	
	handler_reg.list[handler_reg.count ++] = (usb_handler_t){
		.cmd_id = cmd_id, .handler = handler,
	};
	return 1;
}

void usb_send(void) {
	static uint8_t send_buffer[USB_CMD_LENGTH + 64];
	static uint8_t data_buffer[64];
	
	for (uint8_t i = 0; i < packer_reg.count; i++) {
		usb_packer_t *p = &packer_reg.list[i];
		
		if (p->packer == NULL || p->length == 0) continue;
		if (USB_CMD_LENGTH + p->length > sizeof(send_buffer)) continue;
		
		p->packer(data_buffer);
		
		memcpy(send_buffer, (uint8_t*)&p->cmd_id, USB_CMD_LENGTH);
		memcpy(send_buffer + USB_CMD_LENGTH, data_buffer, p->length);
		
		CDC_Transmit_FS(send_buffer, USB_CMD_LENGTH + p->length);
	}
}

void usb_handle(uint8_t *buf, uint32_t len) {
	if (buf == NULL || len < USB_CMD_LENGTH) {
		return;
	}
	
	uint16_t cmd_id;
	memcpy(&cmd_id, buf, sizeof(uint16_t));
	uint8_t *data = buf + USB_CMD_LENGTH;
	
	for (uint8_t i = 0; i < handler_reg.count; i++) {
		usb_handler_t *h = &handler_reg.list[i];
		if (h->cmd_id == cmd_id) {
			if (h->handler) h->handler(data);
			return ;
		}
	}
}

