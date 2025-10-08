///
/// File            | RFIM_Phy.h
/// Description     | Phy driver for Microchip Polarfire SoC
///
/// Author          | Trajce Nikolov | trajce.nikolov.nick@gmail.com | September 2025
///
/// Copyright       | RFIM Space
///


#ifndef RFIM_PHY_H
#define RFIM_PHY_H

#include <stdlib.h>
#include <stdio.h>

#include <drivers/mss/mss_mmuart/mss_uart.h>

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
#define RGMII_100BASE_T2_FD                 (0x1 << 10)
#define RGMII_100BASE_T2_HD                 (0x1 << 9)

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
#define MII_BMSR                            0x01u   /// Basic mode status register
#define MII_BMSR_LSTATUS                    0x0004u /// Link status

/// Init the Phy
///
extern void
RFIM_Phy_init(uint32_t base_addr, uint16_t phy_addr);

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

/// Get the link status
///
extern uint8_t
RFIM_Phy_get_link_status(uint32_t base_addr, uint16_t phy_addr, uint8_t* speed, uint8_t* fullduplex);

/// Setup link speed
///
extern void
RFIM_Phy_set_speed(uint32_t base_addr, uint16_t phy_addr, uint8_t speed);

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

/// Handy macros specialized for speific MACs
///
#define RFIM_MAC0_phy_init()                                \
        RFIM_Phy_init(MAC0_BASE, PHY0_ADDR)

#define RFIM_MAC1_phy_init()                                \
        RFIM_Phy_init(MAC1_BASE, PHY1_ADDR)

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

#define RFIM_MAC0_phy_set_speed(speed)                      \
        RFIM_Phy_set_speed(MAC0_BASE, PHY0_ADDR, speed)

#define RFIM_MAC1_phy_set_speed(speed)                      \
        RFIM_Phy_set_speed(MAC1_BASE, PHY1_ADDR, speed)

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

#define RFIM_MAC0_phy_autonegotiate()                        \
        RFIM_Phy_autonegotiate(MAC0_BASE, PHY0_ADDR)

#define RFIM_MAC1_phy_autonegotiate()                        \
        RFIM_Phy_autonegotiate(MAC1_BASE, PHY1_ADDR)

#endif // RFIM_PHY_H
