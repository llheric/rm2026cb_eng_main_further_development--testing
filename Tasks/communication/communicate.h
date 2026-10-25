#ifndef __COMMUNICATE_H
#define __COMMUNICATE_H

#include <stdlib.h>
#include "protocol.h"
#include "video_trans.h"
#include "stm32f4xx.h"

// There is no need to distuinguish them for threads are seperated
#define UART_RX_DATA_RDY 0x0001
#define UART_MAX_NUMBER 3

typedef void (*COMM_UART_RxCallback)(UART_HandleTypeDef * huart,const uint8_t*, uint16_t);

typedef struct{
	UART_HandleTypeDef * 			huart;
	uint8_t            *      buffer1;
	uint8_t            *      buffer2;
	uint8_t 									nextBuffer;
	uint8_t										pendingBufferDataSize;
	uint16_t                  bufferSize;
	uint16_t                  length;
	bool                  		enabled;
	unpack_data_t 						p_obj;
    vt_unpack_data_t                    p_obj_v;
}Comm_UART_t;

extern Comm_UART_t  cuart1;
extern Comm_UART_t  cuart3;
extern Comm_UART_t  cuart6;

void comm_init(void);

void comm_rx_init(UART_HandleTypeDef * uart);
void comm_rx_callback(UART_HandleTypeDef * huart, const uint8_t * data, uint16_t length);
void UART_Start_Receive_UnfixedLength_IT(Comm_UART_t * cuart, uint16_t maxSize);
void uart1_rx_handle_task(void * args);
void uart3_rx_handle_task(void * args);
void uart6_rx_handle_task(void * args);
void HAL_UART_IdleCallback(UART_HandleTypeDef* huart);
void *pvPortMalloc( size_t xWantedSize );
void vPortFree( void *pv );

HAL_StatusTypeDef comm_transmit_data(UART_HandleTypeDef * uart, uint8_t sof, uint16_t cmdid, const uint8_t * data, uint16_t data_length);

#endif
