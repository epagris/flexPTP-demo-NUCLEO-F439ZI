#ifndef INC_ETHERLIB_OPTIONS
#define INC_ETHERLIB_OPTIONS

#include <embfmt/embformat.h>
#include <standard_output/standard_output.h>

// TODO: maybe a rename would be great
#define SNPRINTF(str,n,fmt, ...) embfmt(str,n,fmt,__VA_ARGS__)

#define ETHLIB_MEMORY_POOL_TOTAL_SIZE (2 * 16384)
#define ETHLIB_MEMORY_POOL_ATTRIBUTES section(".ETHLibPool")

#define ETHLIB_ARPCACHE_SIZE (24)
#define ETHLIB_ARP_RETRY_COUNT (3)
#define ETHLIB_ARP_TIMEOUT_MS (2000)

#define ETHLIB_TMR_SCHED_TABLE_SIZE (20)

#define ETHLIB_DEF_MQ_SIZE (32)

#define ETHLIB_DEF_TCP_QUEUE_SIZE (4)
#define ETHLIB_DEF_TCP_WINDOW_SIZE (2000)

#define ETHLIB_CBD_TABLE_SIZE (16)

#define ETHLIB_DEF_FIFO_SIZE (2000)

// -----------------------------------------

#ifdef CMSIS_OS2
#include <cmsis_os2.h>
#include <memory.h>

typedef osThreadId_t ETHLIB_OS_THREAD_TYPE;
typedef void ThreadReturnType;
typedef void * ThreadParamType;

#define ETHLIB_OS_THREAD_DEFINE(fn,name,prio,stk,param) osThreadDef(name,(fn),(prio),(0),(stk));
ETHLIB_OS_THREAD_TYPE ETHLIB_OS_THREAD_CREATE(osThreadFunc_t fn, const char * name, void * param, int priority, uint32_t stkSize);
#define ETHLIB_OS_THREAD_TERMINATE(thread) osThreadTerminate(thread)

typedef osSemaphoreId_t ETHLIB_OS_SEM_TYPE;

//#define ETHLIB_OS_SEM_RES_CNT (1)
#define ETHLIB_OS_SEM_CREATE(N) osSemaphoreNew(N, 0, NULL);
#define ETHLIB_OS_SEM_DESTROY(sem) osSemaphoreDelete((sem))
#define ETHLIB_OS_SEM_WAIT(sem) osSemaphoreAcquire((sem), osWaitForever)
#define ETHLIB_OS_SEM_POST(sem) osSemaphoreRelease((sem))

typedef osMutexId_t ETHLIB_OS_MTX_TYPE;

extern osMutexAttr_t reqMtxAttr;

#define ETHLIB_OS_MTX_CREATE() osMutexNew(NULL);
#define ETHLIB_OS_RECMTX_CREATE() osMutexNew(&reqMtxAttr);
#define ETHLIB_OS_MTX_DESTROY(mtx) osMutexDelete((mtx))
#define ETHLIB_OS_MTX_LOCK(mtx) osMutexAcquire((mtx), osWaitForever)
#define ETHLIB_OS_MTX_UNLOCK(mtx) osMutexRelease((mtx))

typedef osMessageQueueId_t ETHLIB_OS_QUEUE_TYPE;
#define ETHLIB_OS_QUEUE_CREATE(N, T) osMessageQueueNew((N), sizeof(T), NULL)
#define ETHLIB_OS_QUEUE_DESTROY(q) osMessageQueueDelete((q))
#define ETHLIB_OS_QUEUE_PUSH(q, data) osMessageQueuePut((q), (data), 0, 0);
#define ETHLIB_OS_QUEUE_POP(q, dst) osMessageQueueGet((q), (dst), NULL, osWaitForever);

#define ETHLIB_SLEEP_MS(ms) osDelay((ms))


#elif defined(__linux__)

#include <pthread.h>
#include <semaphore.h>

typedef pthread_t ETHLIB_OS_THREAD_TYPE;

#define ETHLIB_OS_THREAD_VAR_NAME(name) thread_##name
#define ETHLIB_OS_THREAD_DEFINE(fn,name,prio,stk,param) pthread_t ETHLIB_OS_THREAD_VAR_NAME(name); pthread_create(&ETHLIB_OS_THREAD_VAR_NAME(name), NULL, (fn), (param))
#define ETHLIB_OS_THREAD_CREATE(name,param)
#define ETHLIB_OS_THREAD_TERMINATE(thread) pthread_cancel(thread)

typedef sem_t ETHLIB_OS_SEM_TYPE;

#define ETHLIB_OS_SEM_CREATE(sem,N) sem_init((sem), 0, (N))
#define ETHLIB_OS_SEM_DESTROY(sem) sem_destroy((sem))
#define ETHLIB_OS_SEM_WAIT(sem) sem_wait((sem))
#define ETHLIB_OS_SEM_POST(sem) sem_post((sem))

typedef pthread_mutex_t ETHLIB_OS_MTX_TYPE;

#define ETHLIB_OS_MTX_CREATE(mtx) pthread_mutex_init((mtx), NULL)
#define ETHLIB_OS_MTX_DESTROY(mtx) pthread_mutex_destroy((mtx))
#define ETHLIB_OS_MTX_LOCK(mtx) pthread_mutex_lock((mtx))
#define ETHLIB_OS_MTX_UNLOCK(mtx) pthread_mutex_unlock((mtx))

// -----------------------------------------

#include <unistd.h>
//#define ETHLIB_SLEEP_MS(ms) usleep(1000 * (ms))
#define ETHLIB_SLEEP_MS(ms) ;

#endif


#endif /* INC_ETHERLIB_OPTIONS */
