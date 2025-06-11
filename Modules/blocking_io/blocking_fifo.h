#ifndef BLOCKING_IO_BLOCKING_FIFO
#define BLOCKING_IO_BLOCKING_FIFO

#include <stdint.h>
#include <stdbool.h>

#include <cmsis_os2.h>

/**
 * Blocking FIFO structure.
*/
typedef struct {
    uint32_t writeIdx; ///< Next byte to write
    uint32_t readIdx;  ///< Next byte to read
    uint32_t size;   ///< Size of the circular buffer
    osSemaphoreId_t readSem; ///< Read semaphore
    osSemaphoreId_t writeSem; ///< Write semaphore
    uint8_t *mem;      ///< FIFO memory
} BFifo;

/**
 * Create blocking FIFO.
 * @param fifo pointer to existing, but uninitialized BFifo object
 * @param mem pointer to memory space used for FIFO data storage
 * @param size size of the FIFO in bytes (NOT in element count)
 * @return indicates if BFifo creation was successful
*/
bool bfifo_create(BFifo * fifo, uint8_t * mem, uint32_t size);

/**
 * Destroy a blocking FIFO.
 * @param fifo pointer to a BFifo object initialized using bfifo_create()
*/
void bfifo_destroy(BFifo * fifo);

/**
 * Get free size in a FIFO.
 * @param fifo pointer to BFifo object
 * @return number of free bytes
*/
uint32_t bfifo_get_free(const BFifo * fifo);

/**
 * Get used area size in the FIFO.
 * @param fifo pointer to BFifo object
 * @return number of used bytes
*/
uint32_t bfifo_get_used(const BFifo * fifo);

/**
 * Push data into the FIFO.
 * @param fifo pointer to BFifo object
 * @param data pointer to data getting pushed into the FIFO
 * @param size size attempted to be pushed
 * @return number of bytes successfully pushed
*/
uint32_t bfifo_push(BFifo * fifo, const uint8_t * data, uint32_t size);

/**
 * Push ALL the data into the FIFO.
 * @param fifo pointer to BFifo object
 * @param data pointer to data getting pushed into the FIFO
 * @param size size to be pushed
 * @return number of bytes successfully pushed
*/
uint32_t bfifo_push_all(BFifo * fifo, const uint8_t * data, uint32_t size);

/**
 * Read from the FIFO.
 * @param fifo pointer to BFifo object
 * @param dest pointer to destination
 * @param maxSize maximum read size
 * @return number of bytes read
*/
uint32_t bfifo_read(BFifo * fifo, uint8_t * dest, uint32_t maxSize);

/**
 * Pop size bytes of data.
 * @param fifo pointer to BFifo object
 * @param size number of bytes popped
 * @param timeout pop timeout
 * @return number of bytes popped
*/
uint32_t bfifo_pop(BFifo * fifo, uint32_t size, uint32_t timeout);

/**
 * Wait for available data.
 * @param fifo pointer to BFifo object
*/
void bfifo_wait_avail(BFifo * fifo);

#endif /* BLOCKING_IO_BLOCKING_FIFO */
