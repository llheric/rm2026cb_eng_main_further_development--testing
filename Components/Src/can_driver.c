#include "can_driver.h"

extern CAN_HandleTypeDef hcan1, hcan2;

can_tx_queue_t can1_tx_queue;
can_tx_queue_t can2_tx_queue;

void CAN_Driver_Init(void) {
	CAN_FilterTypeDef filter;

	filter.FilterIdHigh = ESC_BASE_ID << 5;
	filter.FilterIdLow = CAN_ID_STD << 2 | CAN_RTR_DATA << 1;
	filter.FilterMaskIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x0000;
	filter.FilterFIFOAssignment = CAN_FilterFIFO0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.SlaveStartFilterBank = CAN2_FILTER_START_BANK;
	/* config the filter on CAN1 */
	filter.FilterBank = 0;
	HAL_CAN_ConfigFilter(&hcan1, &filter);
	/* config the filter on CAN2 */
	filter.FilterBank = CAN2_FILTER_START_BANK;
	HAL_CAN_ConfigFilter(&hcan2, &filter);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
		
	can1_tx_queue.hcan = &hcan1;
	can1_tx_queue.head = 0;
	can1_tx_queue.tail = 0;
	can1_tx_queue.overflow_count = 0;
	can1_tx_queue.error_count = 0;
	
	can2_tx_queue.hcan = &hcan2;
	can2_tx_queue.head = 0;
	can2_tx_queue.tail = 0;
	can2_tx_queue.overflow_count = 0;
	can2_tx_queue.error_count = 0;
	
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
}

uint8_t can_tx_queue_full(can_tx_queue_t *queue) {
	return ((queue->tail + 1) % CAN_TX_QUEUE_SIZE) == queue->head;
}

uint8_t can_tx_queue_empty(can_tx_queue_t *queue) {
	return queue->head == queue->tail;
}

static inline can_tx_queue_t* can_get_queue(CAN_HandleTypeDef *hcan) {
	if (hcan == &hcan1) return &can1_tx_queue;
	if (hcan == &hcan2) return &can2_tx_queue;
	else return NULL;
}

can_tx_status_t can_tx_enqueue(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t dlc, uint8_t buf[8]) {
	can_tx_queue_t *queue = can_get_queue(hcan);
	if (queue == NULL) return CAN_TX_ERROR;
	
	if (can_tx_queue_full(queue)) {
		queue->overflow_count++;
		return CAN_TX_QUEUE_FULL;
	}
	
	queue->msgs[queue->tail].std_id = std_id;
	queue->msgs[queue->tail].dlc = dlc;
	memcpy(queue->msgs[queue->tail].buf, buf, dlc);
	queue->tail = (queue->tail + 1) % CAN_TX_QUEUE_SIZE;
	
	return CAN_TX_OK;
}

can_tx_status_t CAN_Driver_Send(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t dlc, uint8_t buf[8]) {
	uint32_t mailbox;
	CAN_TxHeaderTypeDef header = {
		.StdId = std_id, .IDE = CAN_ID_STD,
		.RTR = CAN_RTR_DATA, .DLC = dlc
	};

	can_tx_queue_t *queue = can_get_queue(hcan);
	if (queue == NULL) return CAN_TX_ERROR;

	// If there are pending messages in the queue, enqueue directly
	if (!can_tx_queue_empty(queue)) {
		return can_tx_enqueue(hcan, std_id, dlc, buf);
	}
	
	// Try to send directly
	if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
		if (HAL_CAN_AddTxMessage(hcan, &header, buf, &mailbox) == HAL_OK) {
			return CAN_TX_OK;
		}
	}
	
	// Enqueue if direct send fails
	return can_tx_enqueue(hcan, std_id, dlc, buf);
}

can_tx_status_t can_tx_dequeue(CAN_HandleTypeDef *hcan) {
	uint32_t mailbox;
	can_tx_queue_t *queue = can_get_queue(hcan);
	if (queue == NULL) return CAN_TX_ERROR;
	
	if (can_tx_queue_empty(queue)) return CAN_TX_QUEUE_EMPTY;
	
	can_tx_msg_t *msg = &queue->msgs[queue->head];

	CAN_TxHeaderTypeDef header = {
		.StdId = msg->std_id, .IDE = CAN_ID_STD,
		.RTR = CAN_RTR_DATA, .DLC = msg->dlc
	};
	
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(queue->hcan, &header, msg->buf, &mailbox);
	if (status != HAL_OK) {
		queue->error_count++;
		return CAN_TX_ERROR;
	}

	queue->head = (queue->head + 1) % CAN_TX_QUEUE_SIZE;
	return CAN_TX_OK;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	can_tx_dequeue(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
	can_tx_dequeue(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
	can_tx_dequeue(hcan);
}
