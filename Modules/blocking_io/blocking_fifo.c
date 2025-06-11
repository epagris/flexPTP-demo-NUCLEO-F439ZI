#include "blocking_fifo.h"

#include <memory.h>

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

bool bfifo_create(BFifo *fifo, uint8_t *mem, uint32_t size) {
    memset(fifo, 0, sizeof(BFifo));
    fifo->size = size; // store size and memory pointer
    fifo->mem = mem;
    fifo->readIdx = 0; // store read and write indices
    fifo->writeIdx = 0;
    fifo->readSem = osSemaphoreNew(1, 0, NULL); // create reading and writing semaphores
    fifo->writeSem = osSemaphoreNew(1, 1, NULL);
    return (fifo->readSem != NULL) && (fifo->writeSem != NULL);
}

void bfifo_destroy(BFifo *fifo) {
    osSemaphoreDelete(fifo->readSem);
    osSemaphoreDelete(fifo->writeSem);
    memset(fifo, 0, sizeof(BFifo));
}

// get a monotonic write index with respect to read index
uint32_t bfifo_monotonic_write_index(const BFifo *fifo) {
    return (fifo->readIdx > fifo->writeIdx) ? (fifo->writeIdx + fifo->size) : fifo->writeIdx;
}

// uint32_t bfifo_monotonic_read_index(const BFifo *fifo) {
//     return (fifo->writeIdx > fifo->readIdx) ? (fifo->readIdx + fifo->size) : fifo->readIdx;
// }

uint32_t bfifo_get_free(const BFifo *fifo) {
    return fifo->size - bfifo_get_used(fifo);
}

uint32_t bfifo_get_used(const BFifo *fifo) {
    return (bfifo_monotonic_write_index(fifo) - fifo->readIdx);
}

uint32_t bfifo_push(BFifo * fifo, const uint8_t * data, uint32_t size) {
    // shortcut zero length pushes
    if (size == 0) {
        return 0;
    }

    // first, take the write semaphore
    osSemaphoreAcquire(fifo->writeSem, osWaitForever);

    // then, determine the push size
    uint32_t freeSize = bfifo_get_free(fifo);
    uint32_t pushSize = MIN(freeSize, size); // push size cannot exceed free size
    
    // do the actual pushing, if pushing involves an index rollover, then
    // pushing can be performed in two consecutive blocks
    uint32_t bytesToMemEnd = fifo->size - fifo->writeIdx; // calculate the number of bytes to end of the memory block counted from the current write index
    uint32_t firstBlockSize = MIN(pushSize, bytesToMemEnd); // determine the first copy block size
    uint32_t secondBlockSize = pushSize - firstBlockSize; // determine the second copy block size (might by zero)
    
    // copy data
    memcpy(fifo->mem + fifo->writeIdx, data, firstBlockSize); // first block
    if (secondBlockSize > 0) {
        memcpy(fifo->mem, data + firstBlockSize, secondBlockSize); // second block
    }

    // advance write index
    uint32_t newWriteIndex = fifo->writeIdx + pushSize;
    if (newWriteIndex >= fifo->size) {
        newWriteIndex = newWriteIndex - fifo->size;
    }
    fifo->writeIdx = newWriteIndex;

    // handle semaphores
    freeSize = bfifo_get_free(fifo); // get free size again considering the changes
    if (freeSize > 0) { // if there's some free space remaining, then release the write semaphore
        osSemaphoreRelease(fifo->writeSem);
    }

    osSemaphoreRelease(fifo->readSem); // it's ceratain, that the read semaphore can be released after a succesful push

    // return with the pushed size
    return pushSize;
}

uint32_t bfifo_push_all(BFifo *fifo, const uint8_t *data, uint32_t size) {
    uint32_t pushLeft = size;
    uint32_t pushedSize = 0;
    while (pushLeft > 0) {
        uint32_t currentPush = bfifo_push(fifo, data + pushedSize, pushLeft);
        pushedSize += currentPush;
        pushLeft -= currentPush;
    }
    return pushedSize;
}

uint32_t bfifo_read(BFifo *fifo, uint8_t *dest, uint32_t maxSize) {
    // shortcut zero-length reads
    if (maxSize == 0) {
        return 0;
    }

    // determine read size
    uint32_t usedSize = bfifo_get_used(fifo);
    uint32_t readSize = MIN(usedSize, maxSize);

    // read data in two blocks (similar to how pushing worked)
    uint32_t bytesToEnd = fifo->size - fifo->readIdx;
    uint32_t firstBlockSize = MIN(bytesToEnd, readSize);
    uint32_t secondBlockSize = readSize - firstBlockSize;

    // copy data
    memcpy(dest, fifo->mem + fifo->readIdx, firstBlockSize); // first block
    if (secondBlockSize > 0) { // second block
        memcpy(dest + firstBlockSize, fifo->mem, secondBlockSize);
    }

    return readSize;
}

uint32_t bfifo_pop(BFifo * fifo, uint32_t size, uint32_t timeout) {
    // shortcut zero-length pops
    if (size == 0) {
        return 0;
    }

    // acquire read semaphore
    osSemaphoreAcquire(fifo->readSem, timeout);

    // limit pop count to actual data content size
    uint32_t usedSize = bfifo_get_used(fifo);
    uint32_t popSize = MIN(usedSize, size);

    // advance read index
    uint32_t newReadIdx = fifo->readIdx + popSize;
    if (newReadIdx >= fifo->size) {
        newReadIdx -= fifo->size;
    }
    fifo->readIdx = newReadIdx;

    // handle semaphores
    osSemaphoreRelease(fifo->writeSem); // it's ceratin, that the FIFO can be written

    usedSize = bfifo_get_used(fifo);
    if (usedSize > 0) { // if the FIFO is not empty, then allow popping
        osSemaphoreRelease(fifo->readSem);
    }

    return popSize; // return with the number of bytes popped
}

void bfifo_wait_avail(BFifo *fifo) {
    // attempt to acquire read semaphore
    osSemaphoreAcquire(fifo->readSem, osWaitForever);

    // if acquiring was successful, then release it immediately,
    // because taking the semaphore was only needed for synchronization
    osSemaphoreRelease(fifo->readSem);

    return;
}
