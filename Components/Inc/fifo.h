#ifndef _COMPONENTS_FIFO_H
#define _COMPONENTS_FIFO_H

#include <stdint.h>
#include <stdbool.h>
#define _COMPONENTS_FIFO_USE_MUTEX 0

typedef struct {
  uint8_t *                 arr;
  uint8_t *                 arr_end;
  uint8_t const volatile *  head;
  uint8_t volatile *        tail;
  volatile uint16_t         used;
  volatile uint16_t         free;
#if (_COMPONENTS_FIFO_USE_MUTEX)
  ABL_OS_Mutex    mutex;
#endif
} Components_FIFO;

Components_FIFO * FIFO_Create(uint16_t size);
void FIFO_Dispose(Components_FIFO ** fifo_ptr);

uint16_t FIFO_Put(Components_FIFO * fifo, uint8_t value);
uint16_t FIFO_Puts(Components_FIFO * fifo, void const * buffer, uint16_t size);
uint16_t FIFO_Pop(Components_FIFO * fifo, uint8_t * value);
uint16_t FIFO_Pops(Components_FIFO * fifo, void * buffer, uint16_t size);

uint16_t FIFO_Put_Unsafe(Components_FIFO * fifo, uint8_t value);
uint16_t FIFO_Puts_Unsafe(Components_FIFO * fifo, void const * buffer, uint16_t size);
uint16_t FIFO_Pop_Unsafe(Components_FIFO * fifo, uint8_t * value);
uint16_t FIFO_Pops_Unsafe(Components_FIFO * fifo, void * buffer, uint16_t size);

bool FIFO_IsEmpty(Components_FIFO const * fifo);
bool FIFO_IsFull(Components_FIFO const * fifo);

uint16_t FIFO_TotalSize(Components_FIFO const * fifo);
uint16_t FIFO_UsedSize(Components_FIFO const * fifo);
uint16_t FIFO_FreeSize(Components_FIFO const * fifo);
#define malloc(size) 	pvPortMalloc(size)
#define free(pv)			vPortFree(pv)
#endif
