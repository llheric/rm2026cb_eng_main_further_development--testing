#include "communicate.h"
#include "os.h"

osThreadId_t uart1_thread;
osThreadId_t uart3_thread;
osThreadId_t uart6_thread;
Comm_UART_t  cuart1;
Comm_UART_t  cuart3;
Comm_UART_t  cuart6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

void UART_Start_Receive_UnfixedLength_IT(Comm_UART_t * cuart, uint16_t maxSize){
	cuart->length = (maxSize < cuart->bufferSize)? maxSize : cuart->bufferSize;
	__HAL_UART_ENABLE_IT(cuart->huart, UART_IT_IDLE);
	cuart->nextBuffer = 2;
	HAL_UART_Receive_DMA(cuart->huart, cuart->buffer1, cuart->bufferSize);
}

void HAL_UART_IdleCallback(UART_HandleTypeDef* huart) {
	HAL_UART_AbortReceive(huart);
	Comm_UART_t *cuart_interface;
	osThreadId_t uart_thread;
	if(huart == cuart1.huart){
		cuart_interface = &cuart1;
		uart_thread = uart1_thread;
	}else if(huart == cuart3.huart){
		cuart_interface = &cuart3;
		uart_thread = uart3_thread;
	}else{
		cuart_interface = &cuart6;
		uart_thread = uart6_thread;
	}
	uint16_t size = cuart_interface->bufferSize - cuart_interface->huart->hdmarx->Instance->NDTR;
	if(cuart_interface->nextBuffer == 1){
		HAL_UART_Receive_DMA(huart, cuart_interface->buffer1,cuart_interface->bufferSize);
		cuart_interface->pendingBufferDataSize = size;
		cuart_interface->nextBuffer = 2;
		osThreadFlagsSet(uart_thread,UART_RX_DATA_RDY);
	}else{
		HAL_UART_Receive_DMA(huart, cuart_interface->buffer2,cuart_interface->bufferSize);
		cuart_interface->pendingBufferDataSize = size;
		cuart_interface->nextBuffer = 1;
		osThreadFlagsSet(uart_thread,UART_RX_DATA_RDY);
	}

}

void comm_init(void) {
	uart1_thread = osThreadCreate("uart1_rx_handle_task",uart1_rx_handle_task,NULL,osPriorityAboveNormal,128);
	uart3_thread = osThreadCreate("uart3_rx_hangle_task",uart3_rx_handle_task,NULL,osPriorityAboveNormal,128);
	uart6_thread = osThreadCreate("uart6_rx_hangle_task",uart6_rx_handle_task,NULL,osPriorityAboveNormal,128);
	cuart1.huart = &huart1;
	cuart3.huart = &huart3;
	cuart6.huart = &huart6;
}

/**
 * @brief  mount rx callback to uart
 */
void comm_rx_init(UART_HandleTypeDef * huart) {
	if(huart==cuart1.huart){
		cuart1.bufferSize = 128;
		cuart1.buffer1 =(uint8_t *) pvPortMalloc(cuart1.bufferSize);
		cuart1.buffer2 =(uint8_t *) pvPortMalloc(cuart1.bufferSize);
	}
	else if(huart == cuart3.huart){
		cuart3.bufferSize = 128;
		cuart3.buffer1 =(uint8_t *) pvPortMalloc(cuart3.bufferSize);
		cuart3.buffer2 =(uint8_t *) pvPortMalloc(cuart3.bufferSize);
	}
	else{
		cuart6.bufferSize = 128;
		cuart6.buffer1 =(uint8_t *) pvPortMalloc(cuart6.bufferSize);
		cuart6.buffer2 =(uint8_t *) pvPortMalloc(cuart6.bufferSize);
	}
}

void uart1_rx_handle_task(void * args) {
	while (true) {
		if(cuart1.nextBuffer == 1){
			protocol_unpack_data(cuart1.buffer1, cuart1.pendingBufferDataSize, &(cuart1.p_obj));
		}else{
			protocol_unpack_data(cuart1.buffer2, cuart1.pendingBufferDataSize, &(cuart1.p_obj));
		}
		osThreadFlagsWait(UART_RX_DATA_RDY, osFlagsWaitAny,osWaitForever);
	}
}

void uart3_rx_handle_task(void * args) {
	while (true) {
		if(cuart3.nextBuffer == 1){
			protocol_unpack_data(cuart3.buffer1, cuart3.pendingBufferDataSize, &(cuart3.p_obj));
		}else{
			protocol_unpack_data(cuart3.buffer2, cuart3.pendingBufferDataSize, &(cuart3.p_obj));
		}
		osThreadFlagsWait(UART_RX_DATA_RDY, osFlagsWaitAny,osWaitForever);
	}
}

void uart6_rx_handle_task(void * args) {
	while (true) {
		if(cuart6.nextBuffer == 1){
			if(cuart6.buffer1[0] == 0xA9){
				video_trans_unpack_data(cuart6.buffer1, cuart6.pendingBufferDataSize, &(cuart6.p_obj_v));
			}
			else{
				protocol_unpack_data(cuart6.buffer1, cuart6.pendingBufferDataSize, &(cuart6.p_obj));
			}
		}else{
			if(cuart6.buffer1[0] == 0xA9){
				video_trans_unpack_data(cuart6.buffer2, cuart6.pendingBufferDataSize, &(cuart6.p_obj_v));
			}
			else{
				protocol_unpack_data(cuart6.buffer2, cuart6.pendingBufferDataSize, &(cuart6.p_obj));
			}
		}
		osThreadFlagsWait(UART_RX_DATA_RDY, osFlagsWaitAny,osWaitForever);
	}
}

/**
 * @brief   transmit packed data to specified uart
 * @param   uart         the specified uart
 * @param   sof          sof
 * @param   cmdid        cmdid
 * @param   data         the pointer to data
 * @param   data_length  the length of data (data segment only)
 * @return  status of ABL_UART_Transmit_DMA
 */
HAL_StatusTypeDef comm_transmit_data(UART_HandleTypeDef * uart, uint8_t sof, uint16_t cmdid, const uint8_t * data, uint16_t data_length) {
	uint16_t frame_length = PROTOCOL_HEADER_LEN + PROTOCOL_CMD_LEN + data_length + PROTOCOL_CRC_LEN;
  uint8_t * buffer = (uint8_t *) pvPortMalloc(frame_length); 
  protocol_pack_data(buffer, sof, cmdid, data, data_length);
  HAL_StatusTypeDef status = HAL_UART_Transmit(uart, (uint8_t*) buffer, frame_length, 0x7FFFFFFF);
  vPortFree(buffer);
  return status;
}
