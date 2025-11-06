///
/// File            | RFIM_Phy.h
/// Description     | Phy driver for Microchip Polarfire SoC
///
/// Author          | Trajce Nikolov    | trajce.nikolov.nick@gmail.com
///                                     | trajce.nikolov.nick@outlook.com
/// Date            | September 2025
///
/// Copyright       | RFIM Space
///


#ifndef RFIM_PHY_H
#define RFIM_PHY_H

#include <mpfs_hal/common/mss_plic.h>

#include <drivers/mss/mss_mmuart/mss_uart.h>
#include <drivers/mss/mss_ethernet_mac/mss_ethernet_registers.h>
#include <drivers/mss/mss_ethernet_mac/mss_ethernet_mac_sw_cfg.h>
#include <drivers/mss/mss_ethernet_mac/mss_ethernet_mac_regs.h>
#include <drivers/mss/mss_ethernet_mac/mss_ethernet_mac_types.h>
#include <drivers/mss/mss_ethernet_mac/mss_ethernet_mac.h>

#ifdef __cplusplus
extern "C" {
#endif

/// 1) Linux Kernel device tree description
/// https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/arch/riscv/boot/dts/microchip/mpfs-icicle-kit-fabric.dtsi
///
/// 2) Table 3-48. APB Slots and Address Map (continued)
/// Polarfire Soc Technical Reference Manual DS60001702Q Page 47
/// MAC0-CFG and MAC1-CFG
///
/// 3) <register address="0x00000008" description="AMP Mode peripheral mapping register.
/// When the register bit is '0' the peripheral is mapped into the 0x2000_0000 address range using AXI bus 5 from the Coreplex.
/// When the register bit is '1' the peripheral is mapped into the 0x2800_0000 address range using AXI bus 6 from the Coreplex."
/// name="APBBUS_CR">
///
/// It can start from 0x28000000 as well
///
/// MAC addresses
///
#define MAC0_BASE                0x20110000u
#define MAC1_BASE                0x20112000u

/// Polarfire SoC phy addresses
///
#define PHY0_ADDR                0x1F
#define PHY1_ADDR                0x1E


/// Different speeds
///
#define RGMII_100BASE_T4                    (0x1 << 15)
#define RGMII_100BASE_TX_FD                 (0x1 << 14)
#define RGMII_100BASE_TX_HD                 (0x1 << 13)
#define RGMII_10BASE_T_FD                   (0x1 << 12)
#define RGMII_10BASE_T_HD                   (0x1 << 11)
#define RGMII_1000BASE_TX_FD                (0x1 << 9)
#define RGMII_1000BASE_TX_HD                (0x1 << 8)
#define RGMII_ALL                           (                               \
                                                RGMII_10BASE_T_HD   |       \
                                                RGMII_10BASE_T_FD   |       \
                                                RGMII_100BASE_TX_HD |       \
                                                RGMII_100BASE_TX_FD |       \
                                                RGMII_1000BASE_TX_HD|       \
                                                RGMII_1000BASE_TX_FD        \
                                            )
/// Internal speed 'enums'
///
#define PHY_MAC10MBPS           0x00
#define PHY_MAC100MBPS          0x01
#define PHY_MAC1000MBPS         0x02
#define PHY_INVALID_SPEED       0x03

/// Duplex mode
///
#define PHY_HALF_DUPLEX                  0u
#define PHY_FULL_DUPLEX                  1u

/// Leds status
///
#define PHY_LEDS_ON     0x1
#define PHY_LEDS_OFF    0x0

/// Commands for the Phy management register
///
#define MII_WRITE       0x1
#define MII_READ        0x2

/// Link status
///
#define PHY_LINK_UP     0x1
#define PHY_LINK_DOWN   0x0

/// This
///
#define MIIMADDR_REG_ADDR_OFFSET            0x34u
#define MIIMADDR_REG_ADDR_MASK              0x003C0000UL
#define MIIMADDR_REG_ADDR_SHIFT             18u

/// This
///
#define MIIMADDR_PHY_ADDR_OFFSET            0x34u
#define MIIMADDR_PHY_ADDR_MASK              0x07800000UL
#define MIIMADDR_PHY_ADDR_SHIFT             23u

#define MIIMCTRL_REG_OFFSET                 0x34u

/// This
///
#define MIIMCTRL_PHY_CTRL_OFFSET            0x34u
#define MIIMCTRL_PHY_CTRL_MASK              0x30000000UL
#define MIIMCTRL_PHY_CTRL_SHIFT             28u

/// Phy management register address
///
#define MIIMMGMT_REG_ADDR                   0x34u

/// Network status register
///
#define MIINETSTAT_REG_ADDR                 0x0008u


///
///
#define PHY_MII_BMSR                            0x01u   /// Basic mode status register
#define PHY_MII_BMSR_LSTATUS                    0x0004u /// Link status

/// Phy registers. Call RFIM_Phy_dump_regs to fill
///
extern volatile uint16_t Phy0_reg_0[32];
extern volatile uint16_t Phy0_reg_1[16];
extern volatile uint16_t Phy0_reg_2[16];
extern volatile uint16_t Phy0_reg_16[32];

extern volatile uint16_t Phy1_reg_0[32];
extern volatile uint16_t Phy1_reg_1[16];
extern volatile uint16_t Phy1_reg_2[16];
extern volatile uint16_t Phy1_reg_16[32];


/// General purpose initialization - The first to call
///
extern void
RFIM_init(void);

/// Reset using GPIO
///
extern void
RFIM_Phy_reset_w_gpio(uint32_t base_addr);

/// Setup speed advertisement
///
extern void
RFIM_Phy_advertise(uint32_t base_addr, uint16_t phy_addr, uint16_t adv_mask);

/// Init the Phy
///
extern void
RFIM_Phy_init(uint32_t base_addr, uint16_t phy_addr);

/// Init the Phy with default speed
///
extern void
RFIM_Phy_init_w_def_speed(uint32_t base_addr, uint16_t phy_addr, uint8_t speed, uint8_t duplex);

/// Write some registry via MDIO
///
extern void
RFIM_Phy_write_reg(uint32_t base_addr, uint16_t phy_addr, uint32_t reg_addr, uint16_t reg_val);

/// Read some registry via MDIO
///
extern uint16_t
RFIM_Phy_read_reg(uint32_t base_addr, uint16_t phy_addr, uint32_t reg_addr);

/// Prepare the registry for use thru the MDIO
///
extern uint32_t
RFIM_Phy_prepare_mgmt_reg(uint16_t phy_addr, uint32_t reg_addr, uint16_t reg_val, uint8_t cmd);

/// Reset the phy
///
extern void
RFIM_Phy_reset(uint32_t base_addr, uint16_t phy_addr);

/// Setup RGMII
///
extern void
RFIM_Phy_set_rgmii(uint32_t base_addr, uint16_t phy_addr);

/// Setup RGMII clk delay
///
extern void
RFIM_Phy_set_rgmii_tx_rx_delay(uint32_t base_addr, uint16_t phy_addr);

/// Get the link status
///
extern uint8_t
RFIM_Phy_get_link_status(uint32_t base_addr, uint16_t phy_addr, uint8_t* speed, uint8_t* fullduplex);

/// Setup link speed
///
extern void
RFIM_Phy_set_speed(uint32_t base_addr, uint16_t phy_addr, uint8_t speed, uint8_t duplex);

/// Autonegotiate
///
extern void
RFIM_Phy_autonegotiate(uint32_t base_addr, uint8_t phy_addr);

/// Turn on/off leds
///
extern void
RFIM_Phy_leds_onoff(uint32_t base_addr, uint16_t phy_addr, uint8_t on_off);

/// Gets the phy id
///
extern void
RFIM_Phy_get_id(uint32_t base_addr, uint16_t phy_addr, uint16_t* id_1, uint16_t* id_2);

/// Setup registry page
///
extern void
RFIM_Phy_setup_page(uint32_t base_addr, uint16_t phy_addr, uint16_t page);

/// Dump the id on UART
///
extern void
RFIM_Phy_dump_id(uint32_t base_addr, uint32_t phy_addr, mss_uart_instance_t* uart);

/// Dump the link status on UART
///
extern void
RFIM_Phy_dump_link_status(uint32_t base_addr, uint32_t phy_addr, mss_uart_instance_t* uart);

/// Dumps the phy registry and save them into the variables from above
/// look at the top of this header
///
extern void
RFIM_Phy_dump_regs(uint32_t base_addr, uint16_t phy_addr);

/// Dumps the leds status on uart
///
extern void
RFIM_Phy_dump_leds_status(uint32_t base_addr, uint16_t phy_addr, mss_uart_instance_t* uart);

/// Cross autonegoyiate between the
/// two MACs in Loopback mode
///
extern void
RFIM_Phy_loopback_autonegotiate(uint32_t base_addr0, uint8_t phy_addr0, uint32_t base_addr1, uint8_t phy_addr1);

/// Handy macros specialized for speific MACs
///
#define RFIM_MAC0_phy_init()                                \
        RFIM_Phy_init(MAC0_BASE, PHY0_ADDR)

#define RFIM_MAC1_phy_init()                                \
        RFIM_Phy_init(MAC1_BASE, PHY1_ADDR)

#define RFIM_MAC0_phy_init_w_def_speed(speed, duplex)       \
        RFIM_Phy_init_w_def_speed(MAC0_BASE, PHY0_ADDR, speed, duplex)

#define RFIM_MAC1_phy_init_w_def_speed(speed, duplex)       \
        RFIM_Phy_init_w_def_speed(MAC1_BASE, PHY1_ADDR, speed, duplex)

#define RFIM_MAC0_phy_write_reg(reg_addr, reg)              \
        RFIM_Phy_write_reg(MAC0_BASE, PHY0_ADDR, reg_addr, reg)

#define RFIM_MAC1_phy_write_reg(reg_addr, reg)              \
        RFIM_Phy_write_reg(MAC1_BASE, PHY1_ADDR, reg_addr, reg)

#define RFIM_MAC0_phy_read_reg(reg_addr, reg)               \
        RFIM_Phy_read_reg(MAC0_BASE, PHY0_ADDR, reg_addr, reg)

#define RFIM_MAC1_phy_read_reg(reg_addr, reg)               \
        RFIM_Phy_read_reg(MAC1_BASE, PHY0_ADDR, reg_addr, reg)

#define RFIM_MAC0_phy_reset()                               \
        RFIM_Phy_reset(MAC0_BASE, PHY0_ADDR)

#define RFIM_MAC1_phy_reset()                               \
        RFIM_Phy_reset(MAC1_BASE, PHY1_ADDR)

#define RFIM_MAC0_phy_set_rgmii()                           \
        RFIM_Phy_set_rgmii(MAC0_BASE, PHY0_ADDR)

#define RFIM_MAC1_phy_set_rgmii()                           \
        RFIM_Phy_set_rgmii(MAC1_BASE, PHY1_ADDR)

#define RFIM_MAC0_phy_get_link_status(speed, duplex)        \
        RFIM_Phy_get_link_status(MAC0_BASE, PHY0_ADDR, speed, duplex)

#define RFIM_MAC1_phy_get_link_status(speed, duplex)        \
        RFIM_Phy_get_link_status(MAC1_BASE, PHY1_ADDR, speed, duplex)

#define RFIM_MAC0_phy_set_speed(speed, duplex)              \
        RFIM_Phy_set_speed(MAC0_BASE, PHY0_ADDR, speed, duplex)

#define RFIM_MAC1_phy_set_speed(speed, duplex)              \
        RFIM_Phy_set_speed(MAC1_BASE, PHY1_ADDR, speed, duplex)

#define RFIM_MAC0_phy_leds_onoff(on_off)                    \
        RFIM_Phy_leds_onoff(MAC0_BASE, PHY0_ADDR, on_off)

#define RFIM_MAC1_phy_leds_onoff(on_off)                    \
        RFIM_Phy_leds_onoff(MAC1_BASE, PHY1_ADDR, on_off)

#define RFIM_MAC0_phy_get_id(id_1, id_2)                    \
        RFIM_Phy_get_id(MAC0_BASE, PHY0_ADDR, id_1, id_2)

#define RFIM_MAC1_phy_get_id(id_1, id_2)                    \
        RFIM_Phy_get_id(MAC1_BASE, PHY1_ADDR, id_1, id_2)

#define RFIM_MAC0_phy_setup_page(page)                      \
        RFIM_Phy_setup_page(MAC0_BASE, PHY0_ADDR, page)

#define RFIM_MAC1_phy_setup_page(page)                      \
        RFIM_Phy_setup_page(MAC1_BASE, PHY1_ADDR, page)

#define RFIM_MAC0_phy_dump_id(uart)                         \
        RFIM_Phy_dump_id(MAC0_BASE, PHY0_ADDR, uart)

#define RFIM_MAC1_phy_dump_id(uart)                         \
        RFIM_Phy_dump_id(MAC1_BASE, PHY1_ADDR, uart)

#define RFIM_MAC0_phy_dump_link_status(uart)                \
        RFIM_Phy_dump_link_status(MAC0_BASE, PHY0_ADDR, uart)

#define RFIM_MAC1_phy_dump_link_status(uart)                \
        RFIM_Phy_dump_link_status(MAC1_BASE, PHY1_ADDR, uart)

#define RFIM_MAC0_phy_autonegotiate()                       \
        RFIM_Phy_autonegotiate(MAC0_BASE, PHY0_ADDR)

#define RFIM_MAC1_phy_autonegotiate()                       \
        RFIM_Phy_autonegotiate(MAC1_BASE, PHY1_ADDR)

#define RFIM_phy_dump_regs()                                \
        RFIM_Phy_dump_regs(MAC0_BASE, PHY0_ADDR);           \
        RFIM_Phy_dump_regs(MAC1_BASE, PHY1_ADDR)

#define RFIM_MAC0_phy_dump_leds_status(uart)                \
        RFIM_Phy_dump_leds_status(MAC0_BASE, PHY0_ADDR, uart)

#define RFIM_MAC1_phy_dump_leds_status(uart)                \
        RFIM_Phy_dump_leds_status(MAC1_BASE, PHY1_ADDR, uart)

#define RFIM_phy_loopback_autonegotiate()                   \
        RFIM_Phy_loopback_autonegotiate(MAC0_BASE, PHY0_ADDR, MAC1_BASE, PHY1_ADDR)


/// MSS compatible
///
///

extern void
MSS_MAC_RFIM_phy_init(/* mss_mac_instance_t */ const void *v_this_mac, uint8_t phy_addr);

extern void
MSS_MAC_RFIM_phy_set_link_speed(/* mss_mac_instance_t */ void *v_this_mac, uint32_t speed_duplex_select, mss_mac_speed_mode_t speed_mode);

extern void
MSS_MAC_RFIM_phy_autonegotiate(/* mss_mac_instance_ t */ const void *v_this_mac);

extern void
MSS_MAC_RFIM_phy_mac_autonegotiate(/* mss_mac_instance_t */ const void *v_this_mac);

extern uint8_t
MSS_MAC_RFIM_phy_get_link_status(/* mss_mac_instance_t */ const void *v_this_mac, mss_mac_speed_t *speed, uint8_t *fullduplex);

#ifdef __cplusplus
}
#endif

#endif // RFIM_PHY_H
