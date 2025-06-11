#ifndef ETHDRV_MAC_DRV
#define ETHDRV_MAC_DRV

#include <stdbool.h>
#include <stm32f4xx.h>

#define MACDRV_PRINT_MISSED_FRAMES (0)
#define DEBUG_RINGBUF (0)

#define MODEINIT_HALF_DUPLEX (0)
#define MODEINIT_FULL_DUPLEX (1 << 8)
#define MODEINIT_SPEED_10MBPS (0)
#define MODEINIT_SPEED_100MBPS (1)

// MUST BE 4-BYTE ALIGNED!
typedef struct {
    uint16_t rxRingLen;     // receive descriptor ring length
    uint16_t txRingLen;     // transmit descriptor ring length
    uint16_t nextTxDescIdx; // index of next available TX descriptor
    uint16_t nextRxDescIdx; // index of first unread RX descriptor
    uint16_t txCntSent;     // sequence number of last transmitted packet
    uint16_t txCntAcked;    // last transmission acknowledged by interrupt
    uint16_t bufSize;       // size of buffers linked to each descriptor
    uint16_t pad0;
} ETHHW_State;

typedef struct {
    uint16_t rxRingLen, txRingLen;  // RX and TX descriptor ring length
    uint8_t *rxRingPtr, *txRingPtr; // pointer to RX and TX descriptor buffers
    uint8_t *bufPtr;                // pointer to RX and TX buffer area
    uint16_t blockSize;             // size of a single buffer
    uint8_t mac[6];                 // MAC-address
    ETHHW_State *statePtr;          // area where ETHHW state is stored, MUST immediately precede rxRingPtr!
} ETHHW_InitOpts;

typedef struct {
    uint32_t DES0, DES1, DES2, DES3, DES4, DES5, DES6, DES7; // descriptor DWORDS
} ETHHW_Desc;

// descriptor extension
// SIZE OF THIS MUST BE DIVISIBLE BY 4!!!
typedef struct {
    uint32_t bufAddr; // buffer address to restore
    uint16_t txCntr;  // transmit counter
    uint16_t pad0;
    uint32_t tsCbPtr; // pointer to timestamp callback function
    uint32_t tsCbArg; // user-defined timestamp parameter
} ETHHW_DescExt;

// full descriptor (DMA descriptor + extension)
// SIZE OF THIS MUST BE DIVISIBLE BY 4!!!
typedef struct {
    ETHHW_Desc desc;   // descriptor
    ETHHW_DescExt ext; // extension
} ETHHW_DescFull;

typedef enum {
    ETHHW_EVT_RX_NOTFY,
    ETHHW_EVT_RX_READ,
    ETHHW_EVT_TX_DONE
} ETHHW_EventType; // TODO shouldn't start with zero

typedef struct {
    uint8_t type; // event type
    union {
        struct {
            uint16_t size;  // packet size
            void *payload;  // pointer to received data
            uint32_t ts_s;  // timestamp seconds
            uint32_t ts_ns; // timestamp nanoseconds
        } rx;
        struct {
            uint32_t tag;   // some arbitrary tag
            uint32_t ts_s;  // timestamp seconds
            uint32_t ts_ns; // timestamp nanoseconds
        } tx;
    } data;
} ETHHW_EventDesc;

#define ETHHW_RET_RX_PROCESSED (1)

typedef enum {
    ETHHW_TXOPT_NONE = 0b00,
    ETHHW_TXOPT_INTERRUPT_ON_COMPLETION = 0b01,
    ETHHW_TXOPT_CAPTURE_TS = 0b11
} ETHHW_TxOpt;

typedef struct {
    uint32_t txTsCbPtr; // pointer to callback function invoked on transmission completion
    uint32_t tag;       // arbitrary tagging
} ETHHW_OptArg_TxTsCap;

void ETHHW_Init(ETH_TypeDef *eth, ETHHW_InitOpts *init);
void ETHHW_Start(ETH_TypeDef *eth);
void ETHHW_Transmit(ETH_TypeDef *eth, const uint8_t *buf, uint16_t len, uint8_t txOpts, void *txOptArgs);

void ETHHW_ProcessRx(ETH_TypeDef *eth);

void ETHHW_SetLinkProperties(ETH_TypeDef * eth, bool fastEthernet, bool fullDuplex);

void ETHHW_ISR(ETH_TypeDef *eth);

uint32_t ETHHW_ReadPHYRegister(ETH_TypeDef *eth, uint32_t PHYAddr, uint32_t PHYReg, uint32_t *pRegValue);
uint32_t ETHHW_WritePHYRegister(ETH_TypeDef *eth, uint32_t PHYAddr, uint32_t PHYReg, uint32_t RegValue);
void ETHHW_ReadErrorState(ETH_TypeDef * eth);

/* ---- PTP CAPABILITIES ---- */

typedef enum {
    ETHHW_PTP_PPS_OFF = 0,
    ETHHW_PTP_PPS_1Hz = 1,
    ETHHW_PTP_PPS_2Hz,
    ETHHW_PTP_PPS_4Hz,
    ETHHW_PTP_PPS_8Hz,
    ETHHW_PTP_PPS_16Hz,
    ETHHW_PTP_PPS_32Hz,
    ETHHW_PTP_PPS_64Hz,
    ETHHW_PTP_PPS_128Hz,
    ETHHW_PTP_PPS_256Hz,
    ETHHW_PTP_PPS_512Hz,
    ETHHW_PTP_PPS_1024Hz,
    ETHHW_PTP_PPS_2048Hz,
    ETHHW_PTP_PPS_4096Hz,
    ETHHW_PTP_PPS_8192Hz,
    ETHHW_PTP_PPS_16384Hz
} ETHHW_PPS_FreqEnum;

void ETHHW_EnablePTPTimeStamping(ETH_TypeDef *eth);                                          // Enable PTP timestamping (currently every frame received gets timestamped)
void ETHHW_DisablePTPTimeStamping(ETH_TypeDef *eth);                                         // Disable PTP timestamping
void ETHHW_InitPTPTime(ETH_TypeDef *eth, uint32_t sec, uint32_t nsec);                       // Initialize PTP clock time
void ETHHW_GetPTPTime(ETH_TypeDef *eth, uint32_t *sec, uint32_t *nsec);                   // Get PTP clock time
void ETHHW_EnablePTPFineCorr(ETH_TypeDef *eth, bool enFineCorr);                             // Enable fine correction method
void ETHHW_UpdatePTPTime(ETH_TypeDef *eth, uint32_t sec, uint32_t nsec, bool add_substract); // Update PTP time forward or backward by a given value
void ETHHW_SetPTPAddend(ETH_TypeDef *eth, uint32_t addend);                                  // Set PTP addend
void ETHHW_SetPTPPPSFreq(ETH_TypeDef *eth, uint32_t freqCode);                               // Set PPS output frequency
void ETHHW_SetPTPSubsecondIncrement(ETH_TypeDef *eth, uint8_t increment);                    // Set PTP clock subsecond increment. Time quantum is 0.467 ns.

#endif /* ETHDRV_MAC_DRV */
