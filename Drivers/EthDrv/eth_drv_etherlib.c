#include "eth_drv_etherlib.h"

#include <memory.h>

#include <stm32f4xx_hal.h>

#include "mac_drv.h"
#include "phy_drv/phy_common.h"

#include <etherlib/dynmem.h>
#include <etherlib/eth_interface.h>

// -------------------------------------
// --------- Ethernet buffers ----------
// -------------------------------------

#define ETH_RX_BUFFER_SIZE (1536UL)

uint8_t ETHBuffer[(ETH_RX_DESC_CNT + ETH_TX_DESC_CNT) * ETH_BUFFER_SIZE] __attribute__((section(".ETHBufferSection"))); /* Ethernet Receive Buffers */

struct {
    ETHHW_State ETHState;
    ETHHW_DescFull DMARxDscrTab[ETH_RX_DESC_CNT];
    ETHHW_DescFull DMATxDscrTab[ETH_TX_DESC_CNT];
} ETHStateAndDesc __attribute__((section(".ETHStateAndDecripSection")));

// -------------------------------------
// ---------- Global objects -----------
// -------------------------------------

static EthIODef ioDef;

static LinkState linkState = {false, false, 0, false};

static osThreadId_t th;

int ethdrv_send(EthIODef *io, MsgQueue *mq);

int ethdrv_read();

static void fetch_link_properties();

static int ethdrv_trigger_link_change_int(EthIODef *io) {
    fetch_link_properties();
    return 0;
}

static void fetch_link_properties() {
    const PHY_LinkStatus *status = phy_get_link_status();

    // set link up/down state
    if ((linkState.up != status->up) || (!linkState.init)) {
        linkState.up = status->up;

        if (linkState.up) {
            bool fe = status->speed == PHY_LS_100Mbps;
            bool duplex = status->type == PHY_LT_FULL_DUPLEX;
            ETHHW_SetLinkProperties(ETH, fe, duplex);

            // convert "Fast Ethernet" to numerical speed value
            linkState.speed = (status->speed == PHY_LS_100Mbps) ? 100 : 10;

            // save duplex field
            linkState.duplex = duplex;
        }

        // invoke link change notification callback
        if (ioDef.llLinkChg != NULL) {
            ioDef.llLinkChg(&ioDef, status->up, linkState.speed, linkState.duplex);
        }
    }

    linkState.init = true;
}

static void phy_thread(void *arg) {
    while (true) {
        fetch_link_properties();
        osDelay(500);
    }
    return;
}

void ethdrv_init() {
    // ------- Ethernet MAC initialization -----

    ETHHW_InitOpts opts = {
        .statePtr = &(ETHStateAndDesc.ETHState),

        .rxRingLen = ETH_RX_DESC_CNT,
        .bufPtr = ETHBuffer,
        .rxRingPtr = (uint8_t *)ETHStateAndDesc.DMARxDscrTab,

        .txRingLen = ETH_TX_DESC_CNT,
        .txRingPtr = (uint8_t *)ETHStateAndDesc.DMATxDscrTab,
        .blockSize = ETH_BUFFER_SIZE,
        .mac = {MAC_ADDR0, MAC_ADDR1, MAC_ADDR2, MAC_ADDR3, MAC_ADDR4, MAC_ADDR5}};

    ETHHW_Init(ETH, &opts);

    ETHHW_Start(ETH);

    // -------- IODef initialization -----------
    memset(&ioDef, 0, sizeof(EthIODef));

    ioDef.llTxTrigger = ethdrv_send;
    ioDef.llRxRead = ethdrv_read;
    ioDef.llTrigLinkChg = ethdrv_trigger_link_change_int;

    // -------- Process PHY events occured during the initialization phase
    
    // start PHY event processing thread
    osThreadAttr_t attr;
    memset(&attr, 0, sizeof(attr));
    attr.stack_size = 2048;
    attr.name = "phy";
    th = osThreadNew(phy_thread, NULL, &attr);
}

int ethdrv_read() {
    ETHHW_ProcessRx(ETH);
    return 0;
}

int ETHHW_ReadCallback(ETHHW_EventDesc *evt) {
    // packet reception
    if (evt->type != ETHHW_EVT_RX_READ) {
        return 0;
    }

    // allocate raw buffer
    RawPckt pckt;
    uint16_t size = evt->data.rx.size;
    uint8_t *plBuf = dynmem_alloc(size);
    if (plBuf == NULL) {
        MSG("malloc failed in ethdrv_input()!");
        return 0; // processing failed
    }

    // fill-in packet data
    pckt.payload = plBuf;
    pckt.size = size;
    pckt.ext.rx.time_s = evt->data.rx.ts_s;
    pckt.ext.rx.time_ns = evt->data.rx.ts_ns;

    // copy data
    memcpy(pckt.payload, evt->data.rx.payload, size); // copy payload

    if (ioDef.llRxStore != NULL) {
        ioDef.llRxStore(&ioDef, &pckt);
    }

    return ETHHW_RET_RX_PROCESSED;
}

int ETHHW_EventCallback(ETHHW_EventDesc *evt) {
    if (evt->type == ETHHW_EVT_RX_NOTFY) {
        if (ioDef.llRxNotify != NULL) {
            ioDef.llRxNotify(&ioDef);
        }
    }

    return 0; // unhandled event
}

int ethdrv_output(const RawPckt *pckt) {
    // check if timestamping is demanded
    uint8_t opts = ETHHW_TXOPT_NONE;
    bool tsEn = (pckt->ext.tx.txTsCb != NULL);
    ETHHW_OptArg_TxTsCap optArg;

    if (tsEn) {
        opts = ETHHW_TXOPT_CAPTURE_TS;
        optArg.txTsCbPtr = (uint32_t)pckt->ext.tx.txTsCb;
        optArg.tag = pckt->ext.tx.arg;
    }

    // transmit
    ETHHW_Transmit(ETH, pckt->payload, pckt->size, opts, &optArg);

    return 0;
}

int ethdrv_send(EthIODef *io, MsgQueue *mq) {
    uint32_t bytes_sent = 0;
    if (mq_avail(mq) > 0) {
        RawPckt pckt = mq_top(mq);
        mq_pop(mq);
        int ret = ethdrv_output(&pckt);
        if (ret != 0) {
            MSG("Tx ERROR!\n");
        }
        bytes_sent += pckt.size;
        dynmem_free(pckt.payload);
        // MSGraw("TX\r\n");
    }
    return (int)bytes_sent;
}

EthIODef *ethdrv_get() {
    return &ioDef;
}
