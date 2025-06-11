#include "mac_drv.h"

#include <memory.h>

#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_system.h>

#include <standard_output/standard_output.h>

static ETHHW_State *ETHHW_GetState(ETH_TypeDef *eth) {
    return ((ETHHW_State *)eth->DMARDLAR) - 1; // state is placed right before the receive descriptors
}

__weak uint32_t ETHHW_setupPHY(ETH_TypeDef *eth) {
    (void)eth;
    return MODEINIT_FULL_DUPLEX | MODEINIT_SPEED_100MBPS;
}

static void ETHHW_InitClocks() {
    GPIO_InitTypeDef gpioInit;

    /* Enable GPIOs clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* Ethernet pins configuration ************************************************/
    /*
     RMII_REF_CLK ----------------------> PA1
     RMII_MDIO -------------------------> PA2
     RMII_MDC --------------------------> PC1
     RMII_MII_CRS_DV -------------------> PA7
     RMII_MII_RXD0 ---------------------> PC4
     RMII_MII_RXD1 ---------------------> PC5
     RMII_MII_RXER ---------------------> PB10
     RMII_MII_TX_EN --------------------> PG11
     RMII_MII_TXD0 ---------------------> PG13
     RMII_MII_TXD1 ---------------------> PB13

     PPS -------------------------------> PB5
     */

    /* Configure PA1, PA2 and PA7 */
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Alternate = GPIO_AF11_ETH;
    gpioInit.Pin = GPIO_PIN_1 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    // activate PULLUP on MDIO data line
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    /* Configure PB5, PB10, PB13 */
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Pin = GPIO_PIN_10 | GPIO_PIN_13 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &gpioInit);

    /* Configure PC1, PC4 and PC5 */
    gpioInit.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &gpioInit);

    /* Configure PG11, PG13 */
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Pin = GPIO_PIN_11 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOG, &gpioInit);

    /* Enable the Ethernet global Interrupt */
    HAL_NVIC_SetPriority(ETH_IRQn, 0x7, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);

    /* Enable Ethernet clocks */
    __HAL_RCC_ETHMAC_CLK_ENABLE();
    __HAL_RCC_ETHMACRX_CLK_ENABLE();
    __HAL_RCC_ETHMACTX_CLK_ENABLE();
    __HAL_RCC_ETHMACPTP_CLK_ENABLE();
}

#ifndef CEIL_TO_4
#define CEIL_TO_4(x) ((((x) >> 2) + (((x) & 0b11) ? 1 : 0)) << 2)
#endif

static void ETHHW_InitPeripheral(ETH_TypeDef *eth, ETHHW_InitOpts *init) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    LL_SYSCFG_SetPHYInterface(LL_SYSCFG_PMC_ETHRMII);

    // reset all MAC internal registers and logic
    SET_BIT(eth->DMABMR, ETH_DMABMR_SR);

    // wait for reset completion
    while (READ_BIT(eth->DMABMR, ETH_DMABMR_SR)) {
        __NOP();
    }

    /* ---- MAC initialization ---- */

    // initialize MDIO clock
    WRITE_REG(eth->MACMIIAR, ETH_MACMIIAR_CR_Div102); // for 150-250MHz HCLK

    // setup PHY
    uint32_t initMode = ETHHW_setupPHY(eth);

    // fill-in MAC address
    uint32_t hwaTmp = 0;
    memcpy(&hwaTmp, init->mac, 4);
    WRITE_REG(eth->MACA0LR, hwaTmp);
    hwaTmp = 0;
    memcpy(&hwaTmp, init->mac + 4, 2);
    WRITE_REG(eth->MACA0HR, hwaTmp);

    // receive using perfect matching and multicast matching
    SET_BIT(eth->MACFFR, /*ETH_MACFFR_HPF | */ ETH_MACFFR_PAM | ETH_MACFFR_PM /*| ETH_MACFFR_RA */);

    // set speed and duplex mode based on the ETHHW return value AND turn on checksum validation
    uint32_t mode = 0;
    if (initMode & MODEINIT_FULL_DUPLEX) { // duplex mode
        mode |= ETH_MACCR_DM;
    }
    if (initMode & MODEINIT_SPEED_100MBPS) { // Fast Ethernet
        mode |= ETH_MACCR_FES;
    }
    WRITE_REG(eth->MACCR, mode | ETH_MACCR_IPCO); // TODO: see errata regarding IPv4 checksum offload

    /* ---- DMA configuration ---- */

    // intermediate buffer setup
    WRITE_REG(eth->DMAOMR, ETH_DMAOMR_TSF | ETH_DMAOMR_RSF /*| ETH_DMAOMR_FEF | ETH_DMAOMR_FUGF */); // transmit and receive store and forward ON

    // calculate aligned buffer size
    uint16_t alignedBufSize = CEIL_TO_4(init->blockSize);

    // create RX descriptors; use Enhanced Descriptors
    uint16_t byteSkip = sizeof(ETHHW_DescExt); // calculate skip size in bytes
    uint16_t dwordSkip = byteSkip / 4;         // calculate skip size in dwords (32-bit units), used by the DMA

    // configure DMA system bus mode: write skip between descriptors and activate Enhanced descriptor format
    eth->DMABMR |= (dwordSkip << ETH_DMABMR_DSL_Pos) | (1 << ETH_DMABMR_EDE_Pos);

    // acquire RX ring-related data from init object (so that further modification of init object does not influence calculations)
    ETHHW_DescFull *ring = (ETHHW_DescFull *)init->rxRingPtr; // copy RX ring begin pointer from the init struct
    uint8_t *rxBuf = init->bufPtr;                            // get data buffer pointer
    uint16_t rxRingLen = init->rxRingLen;                     // copy RX ring length from init struct

    // fill RX descriptor area
    memset(ring, 0, sizeof(ETHHW_DescFull) * rxRingLen); // clear descriptors
    for (uint16_t i = 0; i < init->rxRingLen; i++) {
        ETHHW_DescFull *bd = ring + i;                              // acquire descriptor counting from the beginning of the ring
        bd->ext.bufAddr = ((uint32_t)(rxBuf)) + alignedBufSize * i; // compute buffer start and store it for subsequent use when the descriptor field get overwritten by the DMA
        bd->desc.DES1 = alignedBufSize & ((1 << 13) - 1);           // store the buffer size
        if ((i + 1) == rxRingLen) {                                 // mark that it's the last descriptor
            bd->desc.DES1 |= ETH_DMARXDESC_RER;                     // Receive End of Ring
        }
        bd->desc.DES2 = bd->ext.bufAddr; // store Buffer 1 address (RDES2 = BUF1 address)
        bd->desc.DES3 = 0;

        bd->desc.DES0 = ETH_DMARXDESC_OWN; // descriptor is owned by the DMA
    }

    // write receive-related registers
    WRITE_REG(eth->DMARDLAR, (uint32_t)ring); // ring start address

    // transmit descriptor initialization
    uint32_t txRingLen = init->txRingLen; // ...
    ring = (ETHHW_DescFull *)init->txRingPtr;
    memset(ring, 0, sizeof(ETHHW_DescFull) * txRingLen); // clear everything
    uint8_t *txBuf = rxBuf + alignedBufSize * rxRingLen; // fill in later used buffer addresses
    for (uint16_t i = 0; i < txRingLen; i++) {
        ETHHW_DescFull *bd = ring + i;
        bd->ext.bufAddr = ((uint32_t)txBuf) + i * alignedBufSize;
    }

    // write transmit-related registers
    WRITE_REG(eth->DMATDLAR, (uint32_t)ring); // ring start address

    // initialize state
    ETHHW_State *state = ETHHW_GetState(eth);
    state->bufSize = alignedBufSize;
    state->rxRingLen = init->rxRingLen;
    state->txRingLen = init->txRingLen;
    state->nextRxDescIdx = 0;
    state->nextTxDescIdx = 0;
}

static void ETHHW_InitCounters(ETH_TypeDef *eth) {
    ETHHW_State *state = ETHHW_GetState(eth);
    state->txCntSent = 0;
    state->txCntAcked = 0;
}

void ETHHW_Init(ETH_TypeDef *eth, ETHHW_InitOpts *init) {
    ETHHW_InitClocks();
    ETHHW_InitCounters(eth);
    ETHHW_InitPeripheral(eth, init);
}

void ETHHW_Start(ETH_TypeDef *eth) {
    SET_BIT(eth->DMAIER, ETH_DMAIER_NISE); // normal interrupt summary enable
    SET_BIT(eth->DMAIER, ETH_DMAIER_RIE);  // receive interrupt enable
    SET_BIT(eth->DMAIER, ETH_DMAIER_TIE);  // transmit interrupt enable

    // start DMA reception and transmission
    SET_BIT(eth->DMAOMR, ETH_DMAOMR_SR); // start DMA reception
    SET_BIT(eth->DMAOMR, ETH_DMAOMR_ST); // start DMA transmission

    // ----------------------------------

    // start MAC transceiver
    SET_BIT(eth->MACCR, ETH_MACCR_RE); // enable MAC reception
    SET_BIT(eth->MACCR, ETH_MACCR_TE); // enable MAC transmission
}

__weak int ETHHW_EventCallback(ETHHW_EventDesc *evt) {
    (void)evt;
    if (evt->type == ETHHW_EVT_RX_NOTFY) {
        ETHHW_ProcessRx(ETH);
    }
    return 0;
}

__weak int ETHHW_ReadCallback(ETHHW_EventDesc *evt) {
    (void)evt;
    return ETHHW_RET_RX_PROCESSED;
}

ETHHW_DescFull *ETHHW_AdvanceDesc(ETHHW_DescFull *start, uint16_t n, ETHHW_DescFull *bd, int delta) {
    int16_t index = (((uint32_t)(bd)) - ((uint32_t)(start))) / sizeof(ETHHW_DescFull);
    // int16_t startIndex = index;
    index = ((int)index + delta) % n;
    index = (index < 0) ? (index + n) : index;
    // MSG("%d +(%d) = %d mod %u\n", startIndex, delta, index, n);
    return start + index;
}

#define ETHHW_DESC_PREV(s, n, p) ETHHW_AdvanceDesc((s), (n), (p), -1)
#define ETHHW_DESC_NEXT(s, n, p) ETHHW_AdvanceDesc((s), (n), (p), 1)

// TODO: csak az OWN bitet kell visszakapcsolni és esetleg a CTRL-t állítani
// static void ETHHW_RestoreRXDesc(ETHHW_DescFull *bd) {
//     bd->desc.DES0 = bd->ext.bufAddr;                                                          // store Buffer 1 address
//     bd->desc.DES3 = 0 | ETH_DMARXNDESCRF_OWN | ETH_DMARXNDESCRF_IOC | ETH_DMARXNDESCRF_BUF1V; // set flags: OWN, IOC, BUF1V
// }

#if DEBUG_RINGBUF
typedef enum {
    ETHHW_RINGBUF_RX,
    ETHHW_RINGBUF_TX
} ETHHW_RingBufId;

static void ETHHW_PrintRingBufStatus(ETH_TypeDef *eth, ETHHW_RingBufId ringBufId) {
    volatile ETHHW_DescFull *ring = NULL;
    volatile ETHHW_DescFull *currentPtr = NULL;
    uint16_t n = 0;

    // fetch state
    ETHHW_State *state = ETHHW_GetState(eth);

    // fetch pointers based on ringbuffer ID
    bool RX = (ringBufId == ETHHW_RINGBUF_RX);
    if (RX) {
        ring = (ETHHW_DescFull *)eth->DMARDLAR;
        n = state->rxRingLen;
        currentPtr = (ETHHW_DescFull *)eth->DMACHRDR;
        MSG("RX: ");
    } else {
        ring = (ETHHW_DescFull *)eth->DMATDLAR;
        n = state->txRingLen;
        currentPtr = (ETHHW_DescFull *)eth->DMACHTDR;
        MSG("TX: ");
    }

    uint16_t current = n + 1;              // set current to something non-possible so that seeking current descriptor returns with bad data if current was not found
    uint32_t DES0 = ring[n - 1].desc.DES0; // extract DES0

#define IS_DESC_EMPTY(DES0) ((RX) ? ((DES0) & ETH_DMARXDESC_OWN) : !((DES0) & ETH_DMATXDESC_OWN))

    // packet spanning over multiple descriptors; examine last descriptor first to check if first descriptor is an intermediate one
    bool multiDescPkt = (!(IS_DESC_EMPTY(DES0))) && (!(DES0 & ETH_DMARXDESC_LS));

    MSG("|");
    for (uint16_t i = 0; i < n; i++) {
        if ((ring + i) == currentPtr) { // search current descriptor
            current = i;
        }

        DES0 = ring[i].desc.DES0;             // fetch DES0 dword
        bool descEmpty = IS_DESC_EMPTY(DES0); // determine if descriptor is empty

        if (descEmpty) { // descriptor empty (rx: "armed", tx: not yet passed to the DMA)
            if ((!RX) && (ring[i].ext.tsCbPtr != 0) && (DES0 & ETH_DMATXDESC_TTSE)) {
                MSG("t");
            } else {
                MSG("-");
            }
        } else { // descriptor non-empty (rx: packet stored, tx: packet passed to the DMA)
            // multidescriptor packets
            bool FS = RX ? (DES0 & ETH_DMARXDESC_FS) : (DES0 & ETH_DMATXDESC_FS);
            bool LS = RX ? (DES0 & ETH_DMARXDESC_LS) : (DES0 & ETH_DMATXDESC_LS);
            bool packetBoundary = false;

            // detect first descriptor
            if (FS && !LS) {
                multiDescPkt = true;
                packetBoundary = true;
            }

            // detect last descriptor
            if (!FS && LS) {
                multiDescPkt = false;
                packetBoundary = true;
            }

            // print mark accordingly...
            if (packetBoundary && multiDescPkt) { // first descriptor
                MSG(">");
            } else if (packetBoundary && !multiDescPkt) { // last descriptor
                MSG("<");
            } else { // intermediate or non-multi descriptor
                if ((!RX) && (ring[i].ext.tsCbPtr != 0) && (DES0 & ETH_DMATXDESC_TTSE)) {
                    MSG("T");
                } else {
                    MSG("x");
                }
            }
        }

        // print connection between descriptors
        if (multiDescPkt) { // multi-descriptor packets
            MSG("=");
        } else { // single descriptor packets
            MSG(" ");
        }
    }
    MSG("|\n");

    // mark current descriptor position
    MSG("%*c", (uint32_t)2 * current + 5, ' ');
    MSG("^\n");
}
#endif

#define ETHHW_DESC_OWNED_BY_APPLICATION(bd) (!(((bd)->desc.DES0) & ETH_DMARXDESC_OWN))

#define ETH_DMARXDESC_TSV ETH_DMARXDESC_IPV4HCE

// process incoming packet
void ETHHW_ProcessRx(ETH_TypeDef *eth) {
#if DEBUG_RINGBUF
    ETHHW_PrintRingBufStatus(eth, ETHHW_RINGBUF_RX);
#endif

    // fetch state
    ETHHW_State *state = ETHHW_GetState(eth);

    // fetch RX ring
    ETHHW_DescFull *ring = (ETHHW_DescFull *)eth->DMARDLAR;
    uint16_t ringLen = state->rxRingLen;

    // get current descriptor
    // ETHHW_DescFull *bd = (ETHHW_DescFull *)eth->DMACHRDR;
    // ETHHW_DescFull *bd_prev = ETHHW_DESC_PREV(ring, ringLen, bd);

    // // get the first unprocessed descriptor (oldest one)
    // while (ETHHW_DESC_OWNED_BY_APPLICATION(bd_prev)) {
    //     bd = bd_prev;
    //     bd_prev = ETHHW_DESC_PREV(ring, ringLen, bd);
    // }

    // get first unread RX descriptor
    ETHHW_DescFull *bd = ring + state->nextRxDescIdx;

    // iterate over unprocessed descriptors
    while (ETHHW_DESC_OWNED_BY_APPLICATION(bd)) {
        ETHHW_EventDesc evt;
        evt.type = ETHHW_EVT_RX_READ;
        evt.data.rx.size = (bd->desc.DES0 >> 16) & 0x3FFF; // get received frame length
        evt.data.rx.payload = (void *)bd->ext.bufAddr;     // pass payload to the upper layers

        // check if a timestamp had been captured for the packet as well
        bool tsFound = bd->desc.DES0 & ETH_DMARXDESC_TSV;
        if (tsFound) {
            // fetch timestamp
            evt.data.rx.ts_s = bd->desc.DES7;
            evt.data.rx.ts_ns = bd->desc.DES6;
        }

        int ret = ETHHW_ReadCallback(&evt);
        if (ret == ETHHW_RET_RX_PROCESSED) {           // restore RX descriptor
            SET_BIT(bd->desc.DES0, ETH_DMARXDESC_OWN); // set the OWN bit
        }

        // advance read index
        state->nextRxDescIdx = (state->nextRxDescIdx + 1) % state->rxRingLen;

        // advance descriptor
        bd = ETHHW_DESC_NEXT(ring, ringLen, bd);
    }

#if MACDRV_PRINT_MISSED_FRAMES
    // check missed frames
    uint32_t missed_frames = eth->DMAMFBOCR & (ETH_DMAMFBOCR_MFA_Msk | ETH_DMAMFBOCR_MFC_Msk);
    if (missed_frames != 0) {
        MSG("MFA: %u\nMFC: %u\n", missed_frames >> ETH_DMAMFBOCR_MFA_Pos, missed_frames & ETH_DMAMFBOCR_MFC_Msk);
    }
#endif

    // kick-in receive process
    eth->DMARPDR = 1;

    // ETHHW_PrintRingBufStatus(eth, ETHHW_RINGBUF_RX);
}

void ETHHW_ProcessTx(ETH_TypeDef *eth) {
    //    ETHHW_PrintRingBufStatus(eth, ETHHW_RINGBUF_TX);

    // fetch state
    ETHHW_State *state = ETHHW_GetState(eth);

    // fetch ring
    uint16_t ringLen = state->txRingLen;

    uint16_t minIdx = 0;
    int32_t minDelta = 0;
    ETHHW_DescFull *ring = (ETHHW_DescFull *)eth->DMATDLAR;

    uint16_t descsWithTimestamp = 0;

    do {
        // search for oldest timestamp-carrying descriptor
        bool firstIteration = true;
        for (uint16_t i = 0; i < ringLen; i++) {
            uint16_t txCntr = ring[i].ext.txCntr;
            uint32_t DES0 = ring[i].desc.DES0;
            if ((DES0 & ETH_DMATXDESC_TTSS) && !(DES0 & ETH_DMATXDESC_OWN)) { // examine only descriptors holding a timestamp
                descsWithTimestamp++;                                         // desc found with timestamp
                int32_t delta = (int32_t)txCntr - (int32_t)state->txCntAcked;

                if (firstIteration) { // initialize variables with valid data
                    firstIteration = false;
                    minDelta = delta;
                    minIdx = i;
                    continue;
                }

                if (delta == 1) { // consecutive packets found, stop search
                    minIdx = i;
                    minDelta = 1;
                } else { // search for minimum "distance"
                    if (delta < minDelta) {
                        minIdx = i;
                        minDelta = delta;
                    }
                }
            }
        }

        if (descsWithTimestamp > 0) {
            // invoke callback (the descriptor certainly contains a valid callback address,
            // see transmit function why)
            ETHHW_DescFull *bd = ring + minIdx;

            // fetch timestamp
            uint32_t ts_s = bd->desc.DES7;
            uint32_t ts_ns = bd->desc.DES6;

            // invoke timestamp callback if defined
            if (bd->ext.tsCbPtr != 0) {
                ((void (*)(uint32_t, uint32_t, uint32_t))(bd->ext.tsCbPtr))(ts_s, ts_ns, bd->ext.tsCbArg);
            }

            // clear descriptor
            memset((void *)bd, 0, sizeof(ETHHW_Desc));
            bd->ext.tsCbPtr = 0;
            bd->ext.tsCbArg = 0;

            // increment TX acknowledge counter
            state->txCntAcked = bd->ext.txCntr;

            // decrement number of remaining unprocessed descriptors carrying a timestamp
            descsWithTimestamp--;
        }

    } while (descsWithTimestamp > 0);
}

void ETHHW_ISR(ETH_TypeDef *eth) {
    uint32_t csr = READ_REG(eth->DMASR);

    // MSG("ETH [0x%X]\n", csr);

    if (csr & ETH_DMASR_NIS) {    // Normal Interrupt Summary
        if (csr & ETH_DMASR_RS) { // Receive Interrupt
            SET_BIT(eth->DMASR, ETH_DMASR_RS);
            SET_BIT(eth->DMASR, ETH_DMASR_NIS);

            //ETHHW_PrintRingBufStatus(eth, ETHHW_RINGBUF_RX);

            ETHHW_EventDesc evt;
            evt.type = ETHHW_EVT_RX_NOTFY;
            ETHHW_EventCallback(&evt);
        } else if (csr & ETH_DMASR_TS) { // Transmit Interrupt
            SET_BIT(eth->DMASR, ETH_DMASR_TS);

            ETHHW_ProcessTx(eth);
        }
    }

    SET_BIT(ETH->DMASR, ETH_DMASR_NIS);
}

void ETHHW_Transmit(ETH_TypeDef *eth, const uint8_t *buf, uint16_t len, uint8_t txOpts, void *txOptArgs) {
    ETHHW_State *state = ETHHW_GetState(eth);                               // fetch state
    uint16_t nextTxDescIdx = state->nextTxDescIdx;                          // fetch index of descriptor to fill
    ETHHW_DescFull *bd = ((ETHHW_DescFull *)eth->DMATDLAR) + nextTxDescIdx; // get descriptor being filled

    // MSG("%u\n", nextTxDesc);

    // ETHHW_PrintRingBufStatus(eth, ETHHW_RINGBUF_TX);

    while (!ETHHW_DESC_OWNED_BY_APPLICATION(bd)) {
    } // wait for descriptor to become released by the DMA (if needed)

    // erase possible old descriptor data
    memset(bd, 0, sizeof(ETHHW_Desc)); // DON'T erase extension

    uint32_t opts = 0;
    if (txOpts & ETHHW_TXOPT_INTERRUPT_ON_COMPLETION) {
        opts |= ETH_DMATXDESC_IC;
    }

    if ((txOpts == ETHHW_TXOPT_CAPTURE_TS) && (txOptArgs != NULL)) { // arguments are mandatory
        opts |= ETH_DMATXDESC_TTSE;
        ETHHW_OptArg_TxTsCap *arg = (ETHHW_OptArg_TxTsCap *)txOptArgs; // retrieve args
        bd->ext.tsCbPtr = arg->txTsCbPtr;                              // fill-in extension fields
        bd->ext.tsCbArg = arg->tag;
    }

    // fill-in identification field
    bd->ext.txCntr = ++state->txCntSent; // copy AFTER increase

    // copy payload to TX buffer
    memcpy((void *)bd->ext.bufAddr, buf, len);

    // fill in-descriptor fields
    bd->desc.DES0 = opts | ETH_DMATXDESC_FS | ETH_DMATXDESC_LS | ETH_DMATXDESC_CIC_TCPUDPICMP_FULL; // set First Desc. and Last Desc. flags and turn on FULL checksum insertion
    if ((state->nextTxDescIdx + 1) == state->txRingLen) {                                           // if this is the last descriptor in the ring, then set the Transmit end of ring flag
        bd->desc.DES0 |= ETH_DMATXDESC_TER;
    }
    bd->desc.DES1 = (len & 0x3FFF);  // buffer length truncated to 13-bits
    bd->desc.DES2 = bd->ext.bufAddr; // buffer to BUF1 address
    bd->desc.DES3 = 0;

    SET_BIT(bd->desc.DES0, ETH_DMATXDESC_OWN); // pass desciptor to the DMA

    state->nextTxDescIdx = (state->nextTxDescIdx + 1) % (state->txRingLen); // advance index to next descriptor

    // kick-in transmit process
    eth->DMATPDR = 1;

#if DEBUG_RINGBUF
    ETHHW_PrintRingBufStatus(eth, ETHHW_RINGBUF_TX);
#endif
}

void ETHHW_ReadErrorState(ETH_TypeDef * eth) {
    MSG("RX CRC error: %u\n RX alignment error: %u\n\n", eth->MMCRFCECR, eth->MMCRFAECR);
}

void ETHHW_SetLinkProperties(ETH_TypeDef * eth, bool fastEthernet, bool fullDuplex) {
    // fetch register content
    uint32_t reg = eth->MACCR;

    // set Fast Ethernet state
    if (fastEthernet) {
        SET_BIT(reg, ETH_MACCR_FES);
    } else {
        CLEAR_BIT(reg, ETH_MACCR_FES);
    }

    // set duplex state
    if (fullDuplex) {
        SET_BIT(reg, ETH_MACCR_DM);
    } else {
        CLEAR_BIT(reg, ETH_MACCR_DM);
    }

    // write modified register value back
    eth->MACCR = reg;
}

#define MDIO_ACCESS_TIMEOUT_TICK 100

static uint32_t ETHHW_AccessPHYRegister(ETH_TypeDef *eth, uint32_t PHYAddr, uint32_t PHYReg, uint32_t *pRegValue, bool write) {
    /* Check for the Busy flag */
    while (READ_BIT(eth->MACMIIAR, ETH_MACMIIAR_MB) > 0U) {
    }

    /* Prepare the MDIO Address Register value
     - Set the PHY device address
     - Set the PHY register address
     - Set the read mode
     - Set the MII Busy bit */

    uint32_t tmpreg = eth->MACMIIAR & ETH_MACMIIAR_CR_Msk;                                          // keep only the clock division bits
    tmpreg |= ((PHYAddr & 0x1F) << ETH_MACMIIAR_PA_Pos) | ((PHYReg & 0x1F) << ETH_MACMIIAR_MR_Pos); // set PHY address and register
    if (write) {
        tmpreg |= ETH_MACMIIAR_MW; // set write flag
    } else {
        tmpreg &= ~ETH_MACMIIAR_MW; // clear write flag
    }

    eth->MACMIIAR = tmpreg; // write options into the register

    if (write) { // if writing, set data register
        eth->MACMIIDR = *pRegValue;
    }

    SET_BIT(eth->MACMIIAR, ETH_MACMIIAR_MB); // set busy flag

    /* Wait for the Busy flag */
    uint32_t tick0 = HAL_GetTick();
    while (READ_BIT(eth->MACMIIAR, ETH_MACMIIAR_MB) > 0U) {
        if ((HAL_GetTick() - tick0) > MDIO_ACCESS_TIMEOUT_TICK) {
            return 0;
        }
    }

    // on reading, get data register
    if (!write) {
        *pRegValue = eth->MACMIIDR & 0xFFFF;
    }

    return 1;
}

uint32_t ETHHW_ReadPHYRegister(ETH_TypeDef *eth, uint32_t PHYAddr, uint32_t PHYReg, uint32_t *pRegValue) {
    return ETHHW_AccessPHYRegister(eth, PHYAddr, PHYReg, pRegValue, false);
}

uint32_t ETHHW_WritePHYRegister(ETH_TypeDef *eth, uint32_t PHYAddr, uint32_t PHYReg, uint32_t RegValue) {
    return ETHHW_AccessPHYRegister(eth, PHYAddr, PHYReg, &RegValue, true);
}

/* ---- PTP CAPABILITIES ---- */

// #define ETH_PTP_FLAG_TSARFE ((uint32_t)(1 << 8)) // enable timestamping for every received frame
// #define ETH_PTP_FLAG_TSSSR ((uint32_t)(1 << 9)) // subsecond rollover control (1 = rollover on 10^9-1 nsec)
// #define ETH_PTP_FLAG_TSE ((uint32_t)(1 << 0)) // global timestamp enable flag

void ETHHW_EnablePTPTimeStamping(ETH_TypeDef *eth) {
    __IO uint32_t tmpreg = eth->PTPTSCR;

    tmpreg |= (0b11 << ETH_PTPTSCR_TSCNT_Pos) | ETH_PTPTSCR_TSSIPV4FE | ETH_PTPTSCR_TSSPTPOEFE | // mimic P2P transparent clock
              ETH_PTPTSCR_TSPTPPSV2E | ETH_PTPTSCR_TSE | ETH_PTPTSCR_TSSSR; // turn on relevant flags

    eth->PTPTSCR = tmpreg;
}

void ETHHW_DisablePTPTimeStamping(ETH_TypeDef *eth) {
    __IO uint32_t tmpreg = eth->PTPTSCR;

    tmpreg &= ~ETH_PTPTSCR_TSE;

    eth->PTPTSCR = tmpreg;
}

// #define ETH_PTP_FLAG_TSSTI ((uint32_t)(1 << 2)) // initialize PTP time with the values stored in Timestamp high and low registers

void ETHHW_InitPTPTime(ETH_TypeDef *eth, uint32_t sec, uint32_t nsec) {
    // fill registers with time components
    eth->PTPTSHUR = sec;
    eth->PTPTSLUR = nsec;

    // wait for TSINIT to clear
    while (eth->PTPTSCR & (ETH_PTPTSCR_TSSTI)) {
        __NOP();
    }

    // perform time initialization
    __IO uint32_t tmpreg = eth->PTPTSCR;

    tmpreg |= ETH_PTPTSCR_TSSTI;

    eth->PTPTSCR = tmpreg;
}

void ETHHW_GetPTPTime(ETH_TypeDef * eth, uint32_t * sec, uint32_t * nsec) {
    *sec = eth->PTPTSHR;
    *nsec = eth->PTPTSLR;
}

// #define ETH_PTP_FLAG_TSFCU ((uint32_t)(1 << 1)) // flag controlling fine/coarse update methods

void ETHHW_EnablePTPFineCorr(ETH_TypeDef *eth, bool enFineCorr) {
    __IO uint32_t tmpreg = eth->PTPTSCR;

    if (enFineCorr) {
        tmpreg |= ETH_PTPTSCR_TSFCU;
    } else {
        tmpreg &= ~ETH_PTPTSCR_TSFCU;
    }

    eth->PTPTSCR = tmpreg;
}

// #define ETH_PTP_FLAG_TSSTU ((uint32_t)(1 << 3)) // flag initiating time update

void ETHHW_UpdatePTPTime(ETH_TypeDef *eth, uint32_t sec, uint32_t nsec, bool add_substract) { // true = add
    if (add_substract) {
        nsec &= ~((uint32_t)(1 << 31));
    } else {
        nsec = 1000000000 - (nsec >> 1);
        nsec |= (1 << 31);
    }

    // fill registers with time components
    eth->PTPTSHUR = sec;
    eth->PTPTSLUR = nsec;

    // wait for TSUPDT and TSINIT to clear
    while (eth->PTPTSCR & (ETH_PTPTSCR_TSSTU | ETH_PTPTSCR_TSSTI)) {
        __NOP();
    }

    // perform time update
    __IO uint32_t tmpreg = eth->PTPTSCR;

    tmpreg |= ETH_PTPTSCR_TSSTU;

    eth->PTPTSCR = tmpreg;
}

// #define ETH_PTP_FLAG_TSARU ((uint32_t)(1 << 5)) // flag initiating addend register update

void ETHHW_SetPTPAddend(ETH_TypeDef *eth, uint32_t addend) {
    // size_t i = 0;
    // for (i = 0; i < 8; i++) {
    eth->PTPTSAR = addend; // set addend

    // wait for TSADDREG to clear
    while (eth->PTPTSCR & (ETH_PTPTSCR_TSARU)) {
        __NOP();
    }

    // update PTP block internal register
    __IO uint32_t tmpreg = eth->PTPTSCR;

    tmpreg |= ETH_PTPTSCR_TSARU;

    eth->PTPTSCR = tmpreg;
    //}
}

void ETHHW_SetPTPSubsecondIncrement(ETH_TypeDef *eth, uint8_t increment) {
    eth->PTPSSIR = increment;
}

void ETHHW_SetPTPPPSFreq(ETH_TypeDef *eth, uint32_t freqCode) {
    volatile uint32_t *PTPPPSCR = &(eth->PTPTSSR) + 1;

    __IO uint32_t tmpreg = *PTPPPSCR;

    tmpreg = freqCode & 0x0F;

    *PTPPPSCR = tmpreg;
}

// -------------------

void ETH_IRQHandler() {
    ETHHW_ISR(ETH);
}