#include "etherlib_options.h"

#include <memory.h>

// attributes for creating recursive mutexes
osMutexAttr_t reqMtxAttr = { .attr_bits = osMutexRecursive }; 

// --------

// wrapper function for thread creation
ETHLIB_OS_THREAD_TYPE ETHLIB_OS_THREAD_CREATE(osThreadFunc_t fn, const char * name, void * param, int priority, uint32_t stkSize) {
    osThreadAttr_t attr;
    memset(&attr, 0, sizeof(osThreadAttr_t));
    attr.name = name;
    attr.stack_size = stkSize;
    attr.priority = priority;
    return osThreadNew(fn, param, &attr);
}