#include "ethernet_etherlib.h"

#include <stm32f4xx_hal.h>

#include "etherlib/eth_interface.h"

int udp_recv_cb(const Pckt *packet, PcktSieveLayerTag tag) {
    char str[packet->payloadSize + 1];
    memcpy(str, packet->payload, packet->payloadSize);
    str[packet->payloadSize] = '\0';
    MSG("%s\n", str);
    return 0;
}

EthernetAddress msg_cb_addr = {0x01, 0x60, 0x00, 0x00, 0x00, 0x00};

cbd cb, eth_msg_cb;

#define DUMP_LINE_LENGTH (16)

void dump_packet(EthInterface *intf, const RawPckt *pckt) {
    MSG("# -------------------------\n");
    MSG("# PACKET size = %u\n\n", pckt->size);
    uint16_t i = 0;
    while (i < pckt->size) {
        // determine line start and line end
        uint16_t line_start = i;
        uint16_t line_end = MIN(i + DUMP_LINE_LENGTH, pckt->size);

        // print offset
        MSG("%08X  ", line_start);

        // print the hexdump
        for (uint16_t k = line_start; k < line_end; k++) {
            if (((k % 8) == 0) && (k != line_start)) {
                MSGchar(' ');
            }
            MSG("%02X ", pckt->payload[k]);
        }

        // print padding
        uint8_t padding = (DUMP_LINE_LENGTH - ((line_end - line_start) % DUMP_LINE_LENGTH)) % DUMP_LINE_LENGTH;
        for (uint8_t k = 0; k < padding; k++) {
            MSGraw("   ");
        }
        if (padding > 8) {
            MSGchar(' ');
        }

        // print the ASCII interpretation
        MSGraw("  ");
        for (uint16_t k = line_start; k < line_end; k++) {
            MSGchar(((pckt->payload[k] < ' ') || (pckt->payload[k] > 0x7F)) ? '.' : pckt->payload[k]);
        }

        // line separation
        MSGraw("\r\n");

        // advance index
        i = line_end;
    }

    MSG("\r\n# -------------------------\n");
    MSGraw("\r\n\r\n");
}

void intercept_tx_frame(EthInterface *intf, const RawPckt *pckt) {
    MSG(ANSI_COLOR_BRED "# TX\n");
    dump_packet(intf, pckt);
    MSG(ANSI_COLOR_RESET);
}

void intercept_rx_frame(EthInterface *intf, const RawPckt *pckt) {
    MSG(ANSI_COLOR_BYELLOW "# RX\n");
    dump_packet(intf, pckt);
    MSG(ANSI_COLOR_RESET);
}

void init_ethernet() {
    // initialize EtherLib
    ethlib_init();

    // initialize Ethernet driver
    ethdrv_init();

    // add default interface
    E.ethIntf = ethintf_new(ethdrv_get());
    ethinf_set_capabilities(E.ethIntf, ETHINF_CAP_ALL_RX_TX_CHECKSUM_OFFLOADS);

    EthernetAddress mac = {MAC_ADDR0, MAC_ADDR1, MAC_ADDR2, MAC_ADDR3, MAC_ADDR4, MAC_ADDR5};
    memcpy(E.ethIntf->mac, mac, ETH_HW_ADDR_LEN);

    dhcp_initiate(E.ethIntf);

    icmp_new_connblock(E.ethIntf);

    // ethinf_set_intercept_callback(E.ethIntf, intercept_tx_frame, ETH_INTERCEPT_TX);
    // ethinf_set_intercept_callback(E.ethIntf, intercept_rx_frame, ETH_INTERCEPT_RX);

    // start the interface
    ethinf_up(E.ethIntf);

    // turn on automatic DHCP management
    ethinf_set_automatic_dhcp_management(E.ethIntf, true);

    // print interface info
    MSG("\n---- \n");
    ethinf_print_info(E.ethIntf);
    MSG("---- \n\n");
}
