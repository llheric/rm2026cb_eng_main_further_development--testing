#include "usb.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "os.h"
static uint8_t usb_buf[256];

static void usb_printf(const char *fmt,...)
{
	static va_list ap;
	uint16_t len = 0;
	va_start(ap, fmt);
	len = vsprintf((char *)usb_buf, fmt, ap);
	va_end(ap);
	CDC_Transmit_FS(usb_buf, len);
}

void USB_Task(void *_){
	while(1){
		usb_printf("Welcome to Magician Universal Development Platform\n");
		osDelay(1000);
	}
}

void USB_Init(){
	osThreadCreate("usb", USB_Task, NULL, osPriorityNormal, 64);
}
