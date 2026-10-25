#ifndef USB_COMMUNICATE_H_
#define USB_COMMUNICATE_H_

#include "usbd_cdc_if.h"
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

#define USB_PACKERS_MAX  16
#define USB_HANDLERS_MAX 16

#define USB_CMD_LENGTH 2

typedef void (*packer_t)(uint8_t *buffer);
typedef void (*handler_t)(uint8_t *data);

typedef struct {
	uint16_t cmd_id;
	uint16_t length;
	packer_t packer;
} usb_packer_t;

typedef struct {
	uint16_t  cmd_id;
	handler_t handler;
} usb_handler_t;

typedef struct {
	usb_packer_t list[USB_PACKERS_MAX];
	uint8_t count;
} usb_packer_registry_t;

typedef struct {
	usb_handler_t list[USB_HANDLERS_MAX];
	uint8_t count;
} usb_handler_registry_t;

void usb_communicate_init(void);

void usb_send(void);
void usb_handle(uint8_t *buf, uint32_t len);

uint8_t usb_packer_register(uint16_t cmd_id, uint16_t length, packer_t packer);
uint8_t usb_handler_register(uint16_t cmd_id, handler_t handler);

#endif

