#ifndef _BUTTONS_H
#define _BUTTONS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define MAX_BTNS 16

typedef enum {
	TYPE_INT8,
	TYPE_UINT8,
	TYPE_INT16,
	TYPE_UINT16,
	TYPE_INT32,
	TYPE_UINT32,
} data_type_t;

typedef struct {
	void *data;
	data_type_t type;
} data_t;

typedef struct {
	data_t raw;
	uint8_t level;
	uint8_t rise;
	uint8_t fall;
	uint8_t ready; // placeholder
} button_t;

typedef struct {
	button_t *btns[MAX_BTNS];
	uint16_t size;
} button_group_t;

void create_int8_btn(button_t *btn, int8_t *raw);
void create_uint8_btn(button_t *btn, uint8_t *raw);
void create_int16_btn(button_t *btn, int16_t *raw);
void create_uint16_btn(button_t *btn, uint16_t *raw);
void update_button_status(void);

#endif

