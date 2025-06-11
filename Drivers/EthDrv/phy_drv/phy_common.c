#include "phy_common.h"

#include <memory.h>

#include <standard_output/standard_output.h>

#include "../mac_drv.h"

#include <cmsis_os2.h>

// -------------------------

static uint32_t phyAddr = EPHY_MDIO_BAD_ADDR; // PHY MDIO address

static ETH_TypeDef *phyEth = NULL; // corresponding Ethernet peripheral
static struct {
    uint32_t OUI;     // Organizationally Unique Identifier
    uint8_t model;    // Vendor Model Number
    uint8_t revision; // Model Revision Number
} phyId = {0};
static const char *phyName = NULL; // printable PHY name

typedef void (*phyIntSetupFn)();                        // typedef for PHY interrupt setup function
typedef int (*phyIntHandlerFn)();                       // typedef for PHY interrupt handling
typedef void (*phyFetchLinkStatus)(PHY_LinkStatus *ls); // typedef for link status fetching

static phyIntSetupFn phyIntSetupCb = NULL;                                   // PHY interrupt setup callback
static phyIntHandlerFn phyIntHandlerCb = NULL;                               // PHY interrupt handler callback
static phyFetchLinkStatus phyFetchLinkStatusCb = NULL;                       // fetch link status callback
static uint32_t macModeInit = MODEINIT_FULL_DUPLEX | MODEINIT_SPEED_100MBPS; // default mode: 100Mbps FD

// -------------------------

static PHY_LinkStatus linkStatus = {0};

// -------------------------

static uint32_t WRITE(uint16_t r, uint32_t v) {
    return ETHHW_WritePHYRegister(phyEth, phyAddr, (r), (v));
}
static uint32_t __READ(uint16_t r, uint32_t *v) {
    return ETHHW_ReadPHYRegister(phyEth, phyAddr, (r), v);
}

#define READ(r, v) __READ((r), &(v))

#define READ_MODIFY_WRITE(r, v) \
    {                           \
        uint32_t regVal;        \
        READ(r, regVal);        \
        regVal |= (v);          \
        WRITE(r, regVal);       \
    }

// -------------------------

/**
 * Sweep and poll MDIO addresses, return with the first one with non-zero
 * response.
 * @param addr0 starting address
 * @return address of connected PHY or EPHY_MDIO_BAD_ADDR
 */
static int phy_sweep_addresses(uint32_t addr0) {
    for (uint32_t addr = addr0; addr <= EPHY_MDIO_MAX_ADDR; addr++) {
        uint32_t regVal = 0;
        ETHHW_ReadPHYRegister(
            phyEth, addr, EPHY_BCR,
            &regVal); // read the BCR, since it certainly will contain zero elements
        if (regVal != 0xFFFF) {
            return addr;
        }
    }
    return EPHY_MDIO_BAD_ADDR;
}

// --------------------------

static void phy_setup_int_DP83848() {
    // clear possible interrupts
    uint32_t regVal;
    READ(EPHY_DP83848_MISR, regVal);

    // enable relevant interrupt sources
    regVal = EPHY_DP83848_MISR_LINK_INT_EN | EPHY_DP83848_MISR_ANC_INT_EN;
    WRITE(EPHY_DP83848_MISR, regVal);

    // global enable interrupts
    regVal = EPHY_DP83848_MICR_INT_OE | EPHY_DP83848_MICR_INTEN;
    READ(EPHY_DP83848_MICR, regVal);
}

static void phy_fetch_link_status_DP83848(PHY_LinkStatus *ls) {
    // read and clear interrupt status
    uint32_t regVal;
    READ(EPHY_DP83848_PHYSTS, regVal);

    // fetch link status flags
    ls->speed = (regVal & EPHY_DP83848_SPEED_STATUS) ? PHY_LS_10Mbps : PHY_LS_100Mbps;
    ls->type = (regVal & EPHY_DP83848_DUPLEX_STATUS) ? PHY_LT_FULL_DUPLEX : PHY_LT_HALF_DUPLEX;

    // fetch link state
    READ(EPHY_BSR, regVal);
    ls->up = (regVal & EPHY_BSR_LINK_STATUS);
}

static int phy_int_handler_DP83848() {
    // read and clear interrupt status
    uint32_t regVal;
    ETHHW_ReadPHYRegister(phyEth, phyAddr, EPHY_DP83848_MISR, &regVal);

    // decide which event has been fired
    if (regVal & EPHY_DP83848_MISR_ANC_INT) {
        linkStatus.up = true;
        phy_fetch_link_status_DP83848(&linkStatus);
        return PHYEVENT_AUTONEGOTIATION_DONE;
    } else if (regVal & EPHY_DP83848_MISR_LINK_INT) {
        linkStatus.up = false;

        return PHYEVENT_LINK_DOWN;
    }

    return PHYEVENT_NO_EVENT;
}

static void phy_setup_int_LAN8720A() {
    // force 10Mbps full duplex mode
    /* uint32_t regVal = 0x2100;
    ETHHW_WritePHYRegister(phyEth, phyAddr, EPHY_BCR, regVal); */

    // enable relevant interrupt sources
    uint32_t regVal = EPHY_LAN8720A_IMR_ANC_INT_EN | EPHY_LAN8720A_IMR_ENERGYON_INT_EN;
    WRITE(EPHY_LAN8720A_IMR, regVal);
}

static void phy_fetch_link_status_LAN8720A(PHY_LinkStatus *ls) {
    // read register containing link state
    uint32_t regVal;
    READ(EPHY_LAN8720A_SCSR, regVal);

    // fetch link status flags
    ls->speed = (regVal & EPHY_LAN8720A_SCSR_100MBPS) ? PHY_LS_100Mbps : PHY_LS_10Mbps;
    ls->type = (regVal & EPHY_LAN8720A_SCSR_FULL_DUPLEX) ? PHY_LT_FULL_DUPLEX : PHY_LT_HALF_DUPLEX;

    // fetch link state
    READ(EPHY_BSR, regVal);
    ls->up = (regVal & EPHY_BSR_LINK_STATUS);
}

static int phy_int_handler_LAN8720A() {
    // read and clear possible interrupts
    uint32_t regVal;
    ETHHW_ReadPHYRegister(phyEth, phyAddr, EPHY_LAN8720A_ISFR, &regVal);

    // decide which event has been fired
    if (regVal & EPHY_LAN8720A_ISFR_ANC_INT) {
        linkStatus.up = true;
        phy_fetch_link_status_LAN8720A(&linkStatus);
        return PHYEVENT_AUTONEGOTIATION_DONE;
    } else if (regVal & EPHY_LAN8720A_ISFR_ENERGYON_INT) {
        ETHHW_ReadPHYRegister(phyEth, phyAddr, EPHY_BSR, &regVal);
        if (!(regVal & EPHY_BSR_LINK_STATUS)) {
            linkStatus.up = false;

            return PHYEVENT_LINK_DOWN;
        } else {
            return PHYEVENT_NO_EVENT;
        }
    }

    return PHYEVENT_NO_EVENT;
}

static void phy_setup_int_RTL8201F() {
    // switch to Page 7
    WRITE(EPHY_RTL8201F_PSR, 0x07);

    // configure RMII mode and configure TXC as clock input instead of the default
    // output mode
    WRITE(EPHY_RTL8201F_RMSR, 0x7FFB); // magic constant from the datasheet

    // enable Link Status Change interrupt
    uint32_t regVal;
    READ(EPHY_RTL8201F_INT_WOL_LED_REG, regVal);
    regVal |= EPHY_RTL8201F_INT_WOL_LED_REG_LINKCHG;
    WRITE(EPHY_RTL8201F_INT_WOL_LED_REG, regVal);

    // switch to Page 0
    WRITE(EPHY_RTL8201F_PSR, 0x00);

    return;
}

static void phy_fetch_link_status_RTL8201F(PHY_LinkStatus *ls) {
    // read register containing link state
    uint32_t regVal;
    READ(EPHY_BCR, regVal);

    // set link status flags
    ls->speed = (regVal & EPHY_BCR_SPEED_SELECTION) ? PHY_LS_100Mbps : PHY_LS_10Mbps;
    ls->type = (regVal & EPHY_BCR_DUPLEX_MODE) ? PHY_LT_FULL_DUPLEX : PHY_LT_HALF_DUPLEX;

    // fetch link state
    READ(EPHY_BSR, regVal);
    ls->up = (regVal & EPHY_BSR_LINK_STATUS);

    return;
}

static int phy_int_handler_RTL8201F() {
    // read and clear possible interrupts
    uint32_t regVal;
    READ(EPHY_RTL8201F_IIR_SNR, regVal);

    if (!(regVal & EPHY_RTL8201F_IIR_SNR_LINKSTATUSCHG)) {
        return PHYEVENT_NO_EVENT;
    }

    // read link status twice (read datasheet why)
    READ(EPHY_BSR, regVal);
    READ(EPHY_BSR, regVal);

    // decide which event has been fired
    if (regVal & EPHY_BSR_LINK_STATUS) {
        linkStatus.up = true;
        phy_fetch_link_status_RTL8201F(&linkStatus);
        return PHYEVENT_AUTONEGOTIATION_DONE;
    } else {
        linkStatus.up = false;
        return PHYEVENT_LINK_DOWN;
    }
}

static void phy_setup_int_DP83TC813() {
    // read and clear possible interrupts
    uint32_t regVal;
    // ETHHW_ReadPHYRegister(phyEth, phyAddr, EPHY_DP83TC813_MISR1, &regVal);

    // enable link status interrupt
    regVal = EPHY_DP83TC813_MISR1_LINK_INT_EN;
    WRITE(EPHY_DP83TC813_MISR1, regVal);

    // turn on master mode
    // WRITE(EPHY_DP83TC813_REGCR, 0x01);
    // WRITE(EPHY_DP83TC813_ADDAR, EPHY_DP83TC813_MMD1_PMA_CTRL_2);
    // WRITE(EPHY_DP83TC813_REGCR, 0x4001);
    // WRITE(EPHY_DP83TC813_ADDAR, EPHY_DP83TC813_MMD1_PMA_CTRL_2_MS_CFG);

    return;
}

static void phy_fetch_link_status_DP83TC813(PHY_LinkStatus *ls) {
    // read and clear interrupt status
    uint32_t regVal;
    READ(EPHY_DP83TC813_MISR1, regVal);

    // read basic control register
    READ(EPHY_BCR, regVal);

    // set link status flags
    ls->speed = PHY_LS_100Mbps; // this PHY is only capable of 100Mbps
    ls->type = (regVal & EPHY_BCR_DUPLEX_MODE) ? PHY_LT_FULL_DUPLEX : PHY_LT_HALF_DUPLEX;

    // fetch link state
    READ(EPHY_BSR, regVal);
    ls->up = (regVal & EPHY_BSR_LINK_STATUS);

    return;
}

static int phy_int_handler_DP83TC813() {
    // read and clear interrupt status
    uint32_t regVal;
    ETHHW_ReadPHYRegister(phyEth, phyAddr, EPHY_DP83TC813_MISR1, &regVal);

    // if link has changed, then
    if (regVal & EPHY_DP83TC813_MISR1_LINK_INT) {
        // get actual link state
        ETHHW_ReadPHYRegister(phyEth, phyAddr, EPHY_BSR, &regVal);
        if (regVal & EPHY_BSR_LINK_STATUS) {
            linkStatus.up = true;
            phy_fetch_link_status_DP83TC813(&linkStatus);
            return PHYEVENT_AUTONEGOTIATION_DONE;
        } else {
            linkStatus.up = false;
            return PHYEVENT_LINK_DOWN;
        }
    }

    return PHYEVENT_NO_EVENT;
}

static void phy_setup_int_LAN8670() {
    // PHY cannot signal link change
    return;
}

static void phy_fetch_link_status_LAN8670(PHY_LinkStatus *ls) {
    // link is always 10Mbps HALF duplex
    ls->up = true;
    ls->speed = PHY_LS_10Mbps;
    ls->type = PHY_LT_HALF_DUPLEX;
    return;
}

static int phy_int_handler_LAN8670() {
    // since the PHY cannot signal link change, we should consider the link is up
    linkStatus.up = true;
    phy_fetch_link_status_LAN8670(&linkStatus);
    return PHYEVENT_AUTONEGOTIATION_DONE;
}

static void phy_setup_int_DP83TD510E() {
    // configure LEDs to operate in active high mode
    WRITE(EPHY_DP83TD510E_REGCR, 0x1F);
    WRITE(EPHY_DP83TD510E_ADDAR, EPHY_DP83TD510E_LEDS_CFG2);
    WRITE(EPHY_DP83TD510E_REGCR, 0x4001);
    WRITE(EPHY_DP83TD510E_ADDAR, EPHY_DP83TD510E_LEDS_CFG2_LED_1_POLARITY | EPHY_DP83TD510E_LEDS_CFG2_LED_0_POLARITY);

    // enable Link Change Interrupt
    WRITE(EPHY_DP83TD510E_INTERRUPT_REG_1, EPHY_DP83TD510E_INTERRUPT_REG_1_LINK_INT_EN);

    // select INT functionality on PWDN pin and globally enable interrupts
    READ_MODIFY_WRITE(EPHY_DP83TD510E_GEN_CFG, EPHY_DP83TD510E_GEN_CFG_INT_EN | EPHY_DP83TD510E_GEN_CFG_INT_OE);
}

static void phy_fetch_link_status_DP83TD510E(PHY_LinkStatus *ls) {
    // read basic control register
    uint32_t regVal;
    ETHHW_ReadPHYRegister(phyEth, phyAddr, EPHY_BCR, &regVal);

    // link is always 10Mbps HALF duplex
    ls->speed = PHY_LS_10Mbps;
    ls->type = (regVal & EPHY_BCR_DUPLEX_MODE) ? PHY_LT_FULL_DUPLEX : PHY_LT_HALF_DUPLEX;

    // fetch link state
    READ(EPHY_BSR, regVal);
    ls->up = (regVal & EPHY_BSR_LINK_STATUS);

    return;
}

static int phy_int_handler_DP83TD510E() {
    // read interrupt status
    uint32_t regVal;
    READ(EPHY_DP83TD510E_INTERRUPT_REG_1, regVal);

    // clear interrupt
    WRITE(EPHY_DP83TD510E_PHYSTS, 0x0000);

    // link has changed
    if (regVal & EPHY_DP83TD510E_INTERRUPT_REG_1_LINK_INT) {
        READ(EPHY_DP83TD510E_PHYSTS, regVal);
        if (regVal & EPHY_DP83TD510E_PHYSTS_LINK_STATUS) {
            linkStatus.up = true;
            phy_fetch_link_status_DP83TD510E(&linkStatus);
            return PHYEVENT_AUTONEGOTIATION_DONE;
        } else {
            linkStatus.up = false;
            return PHYEVENT_LINK_DOWN;
        }
    }

    return PHYEVENT_NO_EVENT;
}

// --------------------------

static bool phy_populate_handlers() {
    switch (phyId.OUI) {
    case EPHY_OUI_TEXAS_INSTRUMENTS: // Texas Instruments PHYs
    case EPHY_OUI_TEXAS_INSTRUMENTS_2:
    case EPHY_OUI_TEXAS_INSTRUMENTS_3:
        switch (phyId.model) {
        case EPHY_MODEL_DP83848:
            phyName = "Texas Instruments DP83848";
            phyFetchLinkStatusCb = phy_fetch_link_status_DP83848;
            phyIntSetupCb = phy_setup_int_DP83848;
            phyIntHandlerCb = phy_int_handler_DP83848;
            break;
        case EPHY_MODEL_DP83TC813:
            phyName = "Texas Instruments DP83TC813";
            phyFetchLinkStatusCb = phy_fetch_link_status_DP83TC813;
            phyIntSetupCb = phy_setup_int_DP83TC813;
            phyIntHandlerCb = phy_int_handler_DP83TC813;
            break;
        case EPHY_MODEL_DP83TD510E:
            phyName = "Texas Instruments DP83TD510E";
            phyFetchLinkStatusCb = phy_fetch_link_status_DP83TD510E;
            phyIntSetupCb = phy_setup_int_DP83TD510E;
            phyIntHandlerCb = phy_int_handler_DP83TD510E;
            macModeInit = MODEINIT_HALF_DUPLEX | MODEINIT_SPEED_10MBPS;
        default:
            break;
        }
        break;

    case EPHY_OUI_SMSC: // SMSC PHYs
        switch (phyId.model) {
        case EPHY_MODEL_LAN8720A:
        case EPHY_MODEL_LAN8742A:
            phyName = (phyId.model == EPHY_MODEL_LAN8720A) ? "SMSC LAN8720A" : "SMSC LAN8742A";
            phyFetchLinkStatusCb = phy_fetch_link_status_LAN8720A;
            phyIntSetupCb = phy_setup_int_LAN8720A;
            phyIntHandlerCb = phy_int_handler_LAN8720A;
            break;
        case EPHY_MODEL_LAN8670:
            phyName = "Microchip LAN8670";
            phyFetchLinkStatusCb = phy_fetch_link_status_LAN8670;
            phyIntSetupCb = phy_setup_int_LAN8670;
            phyIntHandlerCb = phy_int_handler_LAN8670;
            macModeInit = MODEINIT_HALF_DUPLEX | MODEINIT_SPEED_10MBPS;
            break;
        default:
            break;
        }
        break;
    case EPHY_OUI_REALTEK:
        switch (phyId.model) {
        case EPHY_MODEL_RTL8201F:
            phyName = "Realtek RTL8201F";
            phyFetchLinkStatusCb = phy_fetch_link_status_RTL8201F;
            phyIntSetupCb = phy_setup_int_RTL8201F;
            phyIntHandlerCb = phy_int_handler_RTL8201F;
            break;

        default:
            break;
        }
        break;
    default:
        phyName = "\0";
        break;
    }

    return phyName[0] != '\0';
}

// ----------------

static void phy_soft_reset() {
    WRITE(EPHY_BCR, EPHY_BCR_RESET);
    uint32_t r;
    do {
        osDelay(5);
        READ(EPHY_BCR, r);
    } while (r & EPHY_BCR_RESET);
}

uint32_t ETHHW_setupPHY(ETH_TypeDef *eth) {
    phyEth = eth; // store pointer to ETH hardware

    phyAddr = phy_sweep_addresses(0);
    if (phyAddr == EPHY_MDIO_BAD_ADDR) { // no PHY detected
        MSG(ANSI_COLOR_BRED "No Ethernet PHY detected!\n" ANSI_COLOR_RESET);
        return MODEINIT_FULL_DUPLEX | MODEINIT_SPEED_100MBPS;
    }

    // issue a software reset
    phy_soft_reset();

    // read the PHYID registers
    uint32_t id1 = 0x00, id2 = 0x00;
    READ(EPHY_ID1, id1);
    READ(EPHY_ID2, id2);

    // WRITE(0x006C, ((uint16_t)0b100) << 12);

    // construct IDs
    phyId.OUI = (id2 >> 10) | (id1 << 6);
    phyId.model = (id2 >> 4) & (0x3F);
    phyId.revision = id2 & 0x0F;

    // populate management functions based on PHYID
    if (phy_populate_handlers()) {
        phyIntSetupCb(); // invoke management function enabling PHY interrupts
    }

    // print PHY name
    phy_print_full_name();

    // return with mode initialization settings
    return macModeInit;
}

// -------------------

PHY_Event phy_get_event() {
    if (phyIntHandlerCb != NULL) {
        return phyIntHandlerCb();
    } else {
        return PHYEVENT_NO_EVENT;
    }
}

void phy_print_full_name() {
    if (phyName[0] != '\0') {
        MSG("PHY: " ANSI_COLOR_BYELLOW "%s" ANSI_COLOR_RESET
            " (rev. %d) @" ANSI_COLOR_CYAN "%d\n" ANSI_COLOR_RESET,
            phyName, phyId.revision, phyAddr);
    } else {
        MSG("PHY: " ANSI_COLOR_BYELLOW "UNKOWN, OUI: %X, MODEL: %u" ANSI_COLOR_RESET
            " (rev. %d) @" ANSI_COLOR_CYAN "%d\n" ANSI_COLOR_RESET,
            phyId.OUI, phyId.model, phyId.revision, phyAddr);
    }
}

int phy_get_link_state() {
    uint32_t regVal;
    READ(EPHY_BSR, regVal);
    return (regVal & EPHY_BSR_AUTONEGOTIATION_COMPLETE) && (regVal & EPHY_BSR_LINK_STATUS);
}

const PHY_LinkStatus *phy_get_link_status() {
    phyFetchLinkStatusCb(&linkStatus);
    return &linkStatus;
}

void phy_refresh_link_status() {
    if (phyIntHandlerCb != NULL) {
        phyIntHandlerCb();
    }
}

static void phy_read_range(uint32_t firstAddr, uint16_t len) {
    for (uint32_t a = firstAddr; a < (firstAddr + len); a += 2) {
        uint32_t regVal;
        READ(a, regVal);
        if (regVal != 0) {
            MSG(ANSI_COLOR_BYELLOW "%04X" ANSI_COLOR_RESET ": " ANSI_COLOR_CYAN "%04X\n" ANSI_COLOR_RESET, a, regVal);
        }
    }
}

void phy_read_all_regs() { phy_read_range(0, EPHY_MDIO_MAX_ADDR); }

bool phy_read_reg(uint32_t addr, uint32_t *dst) { return READ(addr, *dst); }

bool phy_write_reg(uint32_t addr, uint32_t data) { return WRITE(addr, data); }