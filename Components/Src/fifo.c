#define _COMPONENTS_FIFO_C
#include "fifo.h"
#include "cmsis_os.h"

Components_FIFO * FIFO_Create(uint16_t size) {
  if (!size) return 0;
  Components_FIFO * fifo = (Components_FIFO*) malloc(sizeof(*fifo));
  fifo->arr = (uint8_t*) malloc(size);
  fifo->arr_end = fifo->arr + size;
  fifo->head = fifo->arr;
  fifo->tail = fifo->arr;
  fifo->used = 0;
  fifo->free = size;
#if (_COMPONENTS_FIFO_USE_MUTEX)
  fifo->mutex = osMutexCreate(0);
#endif
  return fifo;
}
  
void FIFO_Dispose(Components_FIFO ** fifo_ptr) {
  free((**fifo_ptr).arr);
#if (_COMPONENTS_FIFO_USE_MUTEX)
  osMutexDelete((**fifo_ptr).mutex);
#endif
  free(*fifo_ptr);
  fifo_ptr = 0;
}

uint16_t FIFO_Put(Components_FIFO * fifo, uint8_t value) {
  uint8_t count = 0;
#if (_COMPONENTS_FIFO_USE_MUTEX)
  osMutexWait(fifo->mutex, osWaitForever);
#else
  taskENTER_CRITICAL();
#endif
  if (fifo && fifo->free) {
    *fifo->tail++ = value;
    if (fifo->tail == fifo->arr_end) fifo->tail = fifo->arr;
    ++fifo->used;
    --fifo->free;
    ++count;
  }
#if (_COMPONENTS_FIFO_USE_MUTEX)
  ABL_OS_MutexRelease(fifo->mutex);
#else
  taskEXIT_CRITICAL();
#endif
  return count;
}

uint16_t FIFO_Puts(Components_FIFO * fifo, void const * buffer, uint16_t size) {
  uint8_t * p = (uint8_t*) buffer;
  uint8_t count = 0;
#if (_COMPONENTS_FIFO_USE_MUTEX)
  ABL_OS_MutexAcquire(fifo->mutex);
#else
  taskENTER_CRITICAL();
#endif
  if (fifo && buffer && fifo->free >= size) {
    while (size--) {
      *fifo->tail++ = *p++;
      ++count;
      if (fifo->tail == fifo->arr_end) fifo->tail = fifo->arr;
    }
    fifo->used += count;
    fifo->free -= count;
  }
#if (_COMPONENTS_FIFO_USE_MUTEX)
  ABL_OS_MutexRelease(fifo->mutex);
#else
  taskEXIT_CRITICAL();
#endif
  return count;
}

uint16_t FIFO_Pop(Components_FIFO * fifo, uint8_t * value_ptr) {
  uint8_t count = 0;
#if (_COMPONENTS_FIFO_USE_MUTEX)
  ABL_OS_MutexAcquire(fifo->mutex);
#else
  taskENTER_CRITICAL();
#endif
  if (fifo && value_ptr && fifo->used) {
    *value_ptr = *fifo->head++;
    if (fifo->head == fifo->arr_end) fifo->head = fifo->arr;
    --fifo->used;
    ++fifo->free;
    ++count;
  }
#if (_COMPONENTS_FIFO_USE_MUTEX)
  ABL_OS_MutexRelease(fifo->mutex);
#else
  taskEXIT_CRITICAL();
#endif
  return count;
}

uint16_t FIFO_Pops(Components_FIFO * fifo, void * buffer, uint16_t size) {
  uint8_t * p = (uint8_t*) buffer;
  uint8_t count = 0;
#if (_COMPONENTS_FIFO_USE_MUTEX)
  ABL_OS_MutexAcquire(fifo->mutex);
#else
  taskENTER_CRITICAL();
#endif
  if (fifo && buffer && fifo->used >= size) {
    while (size--) {
      *p++ = *fifo->head++;
      ++count;
      if (fifo->head == fifo->arr_end) fifo->head = fifo->arr;
    }
    fifo->used -= count;
    fifo->free += count;
  }
#if (_COMPONENTS_FIFO_USE_MUTEX)
  ABL_OS_MutexRelease(fifo->mutex);
#else
  taskEXIT_CRITICAL();
#endif
  return count;
}

uint16_t FIFO_Put_Unsafe(Components_FIFO * fifo, uint8_t value) {
  uint8_t count = 0;
  if (fifo && fifo->free) {
    *fifo->tail++ = value;
    if (fifo->tail == fifo->arr_end) fifo->tail = fifo->arr;
    ++fifo->used;
    --fifo->free;
    ++count;
  }
  return count;
}

uint16_t FIFO_Puts_Unsafe(Components_FIFO * fifo, void const * buffer, uint16_t size) {
  uint8_t * p = (uint8_t*) buffer;
  uint8_t count = 0;
  if (fifo && buffer && fifo->free >= size) {
    while (size--) {
      *fifo->tail++ = *p++;
      ++count;
      if (fifo->tail == fifo->arr_end) fifo->tail = fifo->arr;
    }
    fifo->used += count;
    fifo->free -= count;
  }
  return count;
}

uint16_t FIFO_Pop_Unsafe(Components_FIFO * fifo, uint8_t * value_ptr) {
  uint8_t count = 0;
  if (fifo && value_ptr && fifo->used) {
    *value_ptr = *fifo->head++;
    if (fifo->head == fifo->arr_end) fifo->head = fifo->arr;
    --fifo->used;
    ++fifo->free;
    ++count;
  }
  return count;
}

uint16_t FIFO_Pops_Unsafe(Components_FIFO * fifo, void * buffer, uint16_t size) {
  uint8_t * p = (uint8_t*) buffer;
  uint8_t count = 0;
  if (fifo && buffer && fifo->used >= size) {
    while (size--) {
      *p++ = *fifo->head++;
      ++count;
      if (fifo->head == fifo->arr_end) fifo->head = fifo->arr;
    }
    fifo->used -= count;
    fifo->free += count;
  }
  return count;
}

inline bool FIFO_IsEmpty(Components_FIFO const * fifo) {
  return !fifo->used;
}

inline bool FIFO_IsFull(Components_FIFO const * fifo) {
  return !fifo->free;
}

inline uint16_t FIFO_TotalSize(Components_FIFO const * fifo) {
  return fifo->arr_end - fifo->arr;
}

uint16_t FIFO_UsedSize(Components_FIFO const * fifo) {
  return fifo->used;
}

uint16_t FIFO_FreeSize(Components_FIFO const * fifo) {
  return fifo->free;
}
