#include "eth_drv_lwip.h"

#include <memory.h>

#include <stdbool.h>
#include <stdint.h>
#include <stm32f4xx_hal.h>

#include "cmsis_os2.h"
#include "lwip/arch.h"
#include "lwip/err.h"
#include "lwip/etharp.h"
#include "lwip/netif.h"
#include "lwip/snmp.h"

#include "lwip/tcpip.h"
#include "mac_drv.h"
#include "phy_drv/phy_common.h"

#include "lwip/opt.h"

#include "lwip/def.h"
#include "lwip/etharp.h"
#include "lwip/ethip6.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/snmp.h"
#include "lwip/stats.h"
#include "netif/ppp/pppoe.h"
#include "standard_output/standard_output.h"

#include <string.h>

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

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

static struct netif *if0;

static LinkState linkState = {false, false, 0, false};

static osThreadId_t th;

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
        if (linkState.up) {
            // MSG("UP!\n");
            netif_set_link_up(if0);
        } else {
            netif_set_link_down(if0);
            // MSG("DOWN!\n");
        }
    }

    linkState.init = true;
}

static void phy_thread(void *arg) {
    while (true) {
        tcpip_callback(fetch_link_properties, NULL);
        osDelay(500);
    }
    return;
}
// ------------------------------

/* Define those to better describe your network interface. */
#define IFNAME0 'i'
#define IFNAME1 '0'

/* Forward declarations. */
static void ethernetif_input(struct netif *netif);

/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif) {
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

    // -------- Process PHY events occured during the initialization phase

    // start PHY event processing thread
    osThreadAttr_t attr;
    memset(&attr, 0, sizeof(attr));
    attr.stack_size = 2048;
    attr.name = "phy";
    th = osThreadNew(phy_thread, NULL, &attr);

    // -------------------------------------------------------------------

    /* set MAC hardware address length */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;

    /* set MAC hardware address */
    memcpy(netif->hwaddr, opts.mac, ETHARP_HWADDR_LEN);

    /* maximum transfer unit */
    netif->mtu = 1500;

    /* device capabilities */
    /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP /*| NETIF_FLAG_LINK_UP*/;

#if LWIP_IPV6 && LWIP_IPV6_MLD
    /*
     * For hardware/netifs that implement MAC filtering.
     * All-nodes link-local is handled by default, so we must let the hardware know
     * to allow multicast packets in.
     * Should set mld_mac_filter previously. */
    if (netif->mld_mac_filter != NULL) {
        ip6_addr_t ip6_allnodes_ll;
        ip6_addr_set_allnodes_linklocal(&ip6_allnodes_ll);
        netif->mld_mac_filter(netif, &ip6_allnodes_ll, NETIF_ADD_MAC_FILTER);
    }
#endif /* LWIP_IPV6 && LWIP_IPV6_MLD */

    /* Do whatever else is needed to initialize interface. */
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
static err_t low_level_output(struct netif *netif, struct pbuf *p) {
    struct pbuf *q;
    static u8_t concat_buf[ETH_TX_BUF_SIZE];
    u32_t concat_buf_level = 0;

#if ETH_PAD_SIZE
    pbuf_remove_header(p, ETH_PAD_SIZE); /* drop the padding word */
#endif

    /* Concat pbufs into a single buffer */
    for (q = p; q != NULL; q = q->next) {
        /* Send the data from the pbuf to the interface, one pbuf at a
           time. The size of the data in each pbuf is kept in the ->len
           variable. */
        memcpy(concat_buf + concat_buf_level, q->payload, q->len);
        concat_buf_level += q->len;
    }

    /* check if timestamping is demanded */
    uint8_t opts = ETHHW_TXOPT_NONE;
    bool tsEn = (p->tx_cb != NULL);
    ETHHW_OptArg_TxTsCap optArg;
    memset(&optArg, 0, sizeof(ETHHW_OptArg_TxTsCap));

    if (tsEn) {
        opts = ETHHW_TXOPT_CAPTURE_TS;
        optArg.txTsCbPtr = (uint32_t)(p->tx_cb);
        optArg.tag = (uint32_t)p->tag;
    }

    /* Pass the data to the MAC */
    ETHHW_Transmit(ETH, concat_buf, concat_buf_level, opts, &optArg);

    MIB2_STATS_NETIF_ADD(netif, ifoutoctets, p->tot_len);
    if (((u8_t *)p->payload)[0] & 1) {
        /* broadcast or multicast packet*/
        MIB2_STATS_NETIF_INC(netif, ifoutnucastpkts);
    } else {
        /* unicast packet */
        MIB2_STATS_NETIF_INC(netif, ifoutucastpkts);
    }
    /* increase ifoutdiscards or ifouterrors on error */

#if ETH_PAD_SIZE
    pbuf_add_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

    LINK_STATS_INC(link.xmit);

    return ERR_OK;
}

int ETHHW_ReadCallback(ETHHW_EventDesc *evt) {
    // packet reception
    if (evt->type != ETHHW_EVT_RX_READ) {
        return 0;
    }

    /* return indicator */
    int ret = 0;

    /* move received packet into a new pbuf */
    struct pbuf *p, *q;
    /* We allocate a pbuf chain of pbufs from the pool. */
    p = pbuf_alloc(PBUF_RAW, evt->data.rx.size, PBUF_POOL);

    if (p != NULL) {
        /* save size waiting for being stored */
        u16_t size_left = evt->data.rx.size;

        /* We iterate over the pbuf chain until we have read the entire
         * packet into the pbuf. */
        for (q = p; (q != NULL) && (size_left > 0); q = q->next) {
            /* Read enough bytes to fill this pbuf in the chain. The
             * available data in the pbuf is given by the q->len
             * variable.
             * This does not necessarily have to be a memcpy, you can also preallocate
             * pbufs for a DMA-enabled MAC and after receiving truncate it to the
             * actually received size. In this case, ensure the tot_len member of the
             * pbuf is the sum of the chained pbuf len members.
             */

            /* compute copy size and copy */
            u16_t copy_size = MIN(size_left, q->len);
            memcpy(q->payload, evt->data.rx.payload, copy_size);
            size_left -= copy_size;
        }

        /* Copy the timestamp into the first pbuf */
        p->time_s = evt->data.rx.ts_s;
        p->time_ns = evt->data.rx.ts_ns;

        MIB2_STATS_NETIF_ADD(netif, ifinoctets, p->tot_len);
        if (((u8_t *)p->payload)[0] & 1) {
            /* broadcast or multicast packet*/
            MIB2_STATS_NETIF_INC(netif, ifinnucastpkts);
        } else {
            /* unicast packet*/
            MIB2_STATS_NETIF_INC(netif, ifinucastpkts);
        }

        /* packets has been processed and can be released */
        ret = ETHHW_RET_RX_PROCESSED;
    } else {
        LINK_STATS_INC(link.memerr);
        LINK_STATS_INC(link.drop);
        MIB2_STATS_NETIF_INC(netif, ifindiscards);
    }

    /* if no packet could be read, silently ignore this */
    if (p != NULL) {
        /* pass all packets to ethernet_input, which decides what packets it supports */
        if (if0->input(p, if0) != ERR_OK) {
            LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
            pbuf_free(p);
            p = NULL;
        }
    }

    return ret;
}

int ETHHW_EventCallback(ETHHW_EventDesc *evt) {
    if (evt->type == ETHHW_EVT_RX_NOTFY) {
        ethernetif_input(if0);
    }

    return 0; // unhandled event
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
static void ethernetif_input(struct netif *netif) {
    /* read received packets */
    ETHHW_ProcessRx(ETH);
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif) {
    LWIP_ASSERT("netif != NULL", (netif != NULL));

    /* store netif for later usage */
    if0 = netif;

#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netif->hostname = "if0";
#endif /* LWIP_NETIF_HOSTNAME */

    /*
     * Initialize the snmp variables and counters inside the struct netif.
     * The last argument should be replaced with your link speed, in units
     * of bits per second.
     */
    MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

    netif->state = (void *)&linkState;
    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    /* We directly use etharp_output() here to save a function call.
     * You can instead declare your own function an call etharp_output()
     * from it if you have to do some checks before sending (e.g. if link
     * is available...) */
#if LWIP_IPV4
    netif->output = etharp_output;
#endif /* LWIP_IPV4 */
#if LWIP_IPV6
    netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */
    netif->linkoutput = low_level_output;

    /* initialize the hardware */
    low_level_init(netif);

    /* start up the interface */
    netif_set_up(netif);

    /* refresh and fetch PHY and link status */ 
    phy_refresh_link_status();
    fetch_link_properties();

    return ERR_OK;
}
