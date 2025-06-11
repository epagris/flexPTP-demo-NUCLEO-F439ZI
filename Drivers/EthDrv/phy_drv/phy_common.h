#ifndef PHY_DRV_PHY_COMMON
#define PHY_DRV_PHY_COMMON

#include <stdint.h>

#include <stdbool.h>

typedef enum {
    PHYEVENT_NO_EVENT = 0,
    PHYEVENT_LINK_DOWN,
    PHYEVENT_AUTONEGOTIATION_DONE
} PHY_Event;

typedef enum {
    PHY_LS_10Mbps,
    PHY_LS_100Mbps,
    PHY_LS_1000Mbps
} PHY_LinkSpeed;

typedef enum {
    PHY_LT_HALF_DUPLEX,
    PHY_LT_FULL_DUPLEX
} PHY_LinkType;

typedef struct {
    bool up; // indicates that Ethernet link is up
    PHY_LinkSpeed speed; // Ethernet link speed
    PHY_LinkType type; // link type
} PHY_LinkStatus;

// --------------

#define EPHY_MDIO_MAX_ADDR (31)
#define EPHY_MDIO_BAD_ADDR (EPHY_MDIO_MAX_ADDR + 1)

#define EPHY_BIT(n) ((uint16_t)(1 << (n)))

#define EPHY_BCR ((uint16_t)0x0000) /*!< Transceiver Basic Control Register   */

#define EPHY_BCR_RESET EPHY_BIT(15)
#define EPHY_BCR_LOOPBACK EPHY_BIT(14)
#define EPHY_BCR_SPEED_SELECTION EPHY_BIT(13)
#define EPHY_BCR_AUTONEGOTIATION_ENABLE EPHY_BIT(12)
#define EPHY_BCR_POWER_DOWN EPHY_BIT(11)
#define EPHY_BCR_ISOLATE EPHY_BIT(10)
#define EPHY_BCR_RESTART_AUTONEGOTIATION EPHY_BIT(9)
#define EPHY_BCR_DUPLEX_MODE EPHY_BIT(8)
#define EPHY_BCR_COLLISION_TEST EPHY_BIT(7)

#define EPHY_BSR ((uint16_t)0x0001) /*!< Transceiver Basic Status Register    */

#define EPHY_BSR_100BASET4 EPHY_BIT(15)
#define EPHY_BSR_100BASETX_FD EPHY_BIT(14)
#define EPHY_BSR_100BASETX_HD EPHY_BIT(13)
#define EPHY_BSR_10BASET_FD EPHY_BIT(12)
#define EPHY_BSR_10BASET_HD EPHY_BIT(11)
#define EPHY_BSR_10BASET2_FD EPHY_BIT(10)
#define EPHY_BSR_10BASET2_HD EPHY_BIT(9)
// #define EPHY_BSR_EXTENDED_STATUS EPHY_BIT(8)
#define EPHY_BSR_AUTONEGOTIATION_COMPLETE EPHY_BIT(5)
#define EPHY_BSR_REMOTE_FAULT EPHY_BIT(4)
#define EPHY_BSR_AUTONEGOTIATION_ABILITY EPHY_BIT(3)
#define EPHY_BSR_LINK_STATUS EPHY_BIT(2)
#define EPHY_BSR_JABBER_DETECT EPHY_BIT(1)
#define EPHY_BSR_EXTENDED_CAPABILITY EPHY_BIT(0)

#define EPHY_ID1 ((uint16_t)0x0002) /*!< PHYID1 Register */
#define EPHY_ID2 ((uint16_t)0x0003) /*!< PHYID2 Register */

// ----------------------

#define EPHY_OUI_TEXAS_INSTRUMENTS (0x080017)
#define EPHY_OUI_TEXAS_INSTRUMENTS_2 (0x080028)
#define EPHY_OUI_TEXAS_INSTRUMENTS_3 (0x080000)
#define EPHY_OUI_SMSC (0x0001F0)
#define EPHY_OUI_REALTEK (0x000732)

// 100Base-TX
#define EPHY_MODEL_DP83848 (0x09)
#define EPHY_MODEL_LAN8720A (0x0F)
#define EPHY_MODEL_LAN8742A (0x13)
#define EPHY_MODEL_RTL8201F (0x01)

// 100Base-T1
#define EPHY_MODEL_DP83TC813 (0x21)

// 10Base-T1S
#define EPHY_MODEL_LAN8670 (0x16)

// 10Base-T1L
#define EPHY_MODEL_DP83TD510E (0x18)

// ----------------------

#define EPHY_DP83848_PHYSTS (0x10)
#define EPHY_DP83848_SPEED_STATUS EPHY_BIT(1)
#define EPHY_DP83848_DUPLEX_STATUS EPHY_BIT(2)

#define EPHY_DP83848_MICR (0x11)

#define EPHY_DP83848_MICR_INTEN EPHY_BIT(1)
#define EPHY_DP83848_MICR_INT_OE EPHY_BIT(0)

#define EPHY_DP83848_MISR (0x12)

#define EPHY_DP83848_MISR_LINK_INT EPHY_BIT(13)
#define EPHY_DP83848_MISR_ANC_INT EPHY_BIT(10)
#define EPHY_DP83848_MISR_LINK_INT_EN EPHY_BIT(5)
#define EPHY_DP83848_MISR_ANC_INT_EN EPHY_BIT(2)

// ----

#define EPHY_LAN8720A_MCSR (0x11)

#define EPHY_LAN8720A_MCSR_ALTINT EPHY_BIT(6)

#define EPHY_LAN8720A_ISFR (0x1D)

#define EPHY_LAN8720A_ISFR_ENERGYON_INT EPHY_BIT(7)
#define EPHY_LAN8720A_ISFR_ANC_INT EPHY_BIT(6)
#define EPHY_LAN8720A_ISFR_LINK_DOWN_INT EPHY_BIT(4)

#define EPHY_LAN8720A_IMR (0x1E)

#define EPHY_LAN8720A_IMR_ENERGYON_INT_EN EPHY_BIT(7)
#define EPHY_LAN8720A_IMR_ANC_INT_EN EPHY_BIT(6)
#define EPHY_LAN8720A_IMR_LINK_DOWN_INT_EN EPHY_BIT(4)

#define EPHY_LAN8720A_SCSR (0x1F)
#define EPHY_LAN8720A_SCSR_FULL_DUPLEX EPHY_BIT(4)
#define EPHY_LAN8720A_SCSR_100MBPS EPHY_BIT(3)
#define EPHY_LAN8720A_SCSR_10MBPS EPHY_BIT(2)

// ----

// PAGE 0
#define EPHY_RTL8201F_IIR_SNR (0x1E) // Interrupt Indicator and SNR register
#define EPHY_RTL8201F_IIR_SNR_LINKSTATUSCHG EPHY_BIT(11)

#define EPHY_RTL8201F_PSR (0x1F) // Page Select Register

// PAGE 7
#define EPHY_RTL8201F_RMSR (0x10)  // RMII Mode Setting Register
#define EPHY_RTL8201F_RMSR_CLKDIR EPHY_BIT(12) // RMII clock direction

#define EPHY_RTL8201F_INT_WOL_LED_REG (0x13) // Interrupt, WOL Enable and LEDs Function Register
#define EPHY_RTL8201F_INT_WOL_LED_REG_LINKCHG EPHY_BIT(13)

// ----

#define EPHY_DP83TC813_REGCR (0x0D)
#define EPHY_DP83TC813_ADDAR (0x0E)

//#define EPHY_DP83TC813_PHYSTS (0x10)
//#define EPHY_DP83TC813_PHYSTS_LINK_STATUS EPHY_BIT(0)

#define EPHY_DP83TC813_MISR1 (0x12)

#define EPHY_DP83TC813_MISR1_LINK_INT EPHY_BIT(13)
#define EPHY_DP83TC813_MISR1_LINK_INT_EN EPHY_BIT(5)

#define EPHY_DP83TC813_MMD1_PMA_CTRL_2 (0x0834)
#define EPHY_DP83TC813_MMD1_PMA_CTRL_2_MS_CFG EPHY_BIT(14)

// CRS_DV to RX_DV feature is not documented, these two lines came from an FAQ answer found on the TI's website
// https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1113891/faq-dp83tc812r-q1-how-can-i-connect-phys-back-to-back-over-rmii
#define EPHY_DP83TC813_REG_0X0551 (0x0551)
#define EPHY_DP83TC813_REG_0X0551_RX_DV EPHY_BIT(4)

// ----

#define EPHY_DP83TD510E_REGCR (0x0D)
#define EPHY_DP83TD510E_ADDAR (0x0E)

#define EPHY_DP83TD510E_PHYSTS (0x10)
#define EPHY_DP83TD510E_PHYSTS_LINK_STATUS EPHY_BIT(0)

#define EPHY_DP83TD510E_GEN_CFG (0x11)
#define EPHY_DP83TD510E_GEN_CFG_INT_EN EPHY_BIT(1)
#define EPHY_DP83TD510E_GEN_CFG_INT_OE EPHY_BIT(0)

#define EPHY_DP83TD510E_INTERRUPT_REG_1 (0x12)
#define EPHY_DP83TD510E_INTERRUPT_REG_1_LINK_INT EPHY_BIT(13)
#define EPHY_DP83TD510E_INTERRUPT_REG_1_LINK_INT_EN EPHY_BIT(5)

#define EPHY_DP83TD510E_LEDS_CFG2 (0x469)
#define EPHY_DP83TD510E_LEDS_CFG2_LED_1_POLARITY EPHY_BIT(6)
#define EPHY_DP83TD510E_LEDS_CFG2_LED_0_POLARITY EPHY_BIT(2)

// ----------------------

/**
 * Get PHY event.
 * @return PHY event
*/
PHY_Event phy_get_event();

/**
 * Get link state
 * @return link state
*/
int phy_get_link_state();

/**
 * Print full PHY name and ID.
*/
void phy_print_full_name();

/**
 * Call this to forcibly refresh link status.
 * Link status can be read using phy_get_link_status().
 */
void phy_refresh_link_status();

/**
 * Get link status.
 * @return const pointer to link status
*/
const PHY_LinkStatus * phy_get_link_status();

/**
 * Read and print all PHY registers.
*/
void phy_read_all_regs();

/**
 * Read a single register.
 * @param addr register address
 * @param dst pointer to destination
 * @return indicates if read was successful
*/
bool phy_read_reg(uint32_t addr, uint32_t * dst);

/**
 * Write a single register.
 * @param addr register address
 * @param data data to be written
 * @return indicates if write was successful
*/
bool phy_write_reg(uint32_t addr, uint32_t data);

/**
 * HW reset PHY.
*/
void phy_hw_reset();

#endif /* PHY_DRV_PHY_COMMON */
