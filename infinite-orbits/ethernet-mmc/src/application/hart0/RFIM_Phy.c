///
/// File            | RFIM_Phy.c
/// Description     | Phy driver for Microchip Polarfire SoC
///
/// Author          | Trajce Nikolov | trajce.nikolov.nick@gmail.com | September 2025
///
/// Copyright       | RFIM Space
///

#include "RFIM_Phy.h"

#include <hal/hal.h>
#include <hal/hal_assert.h>
#include <mpfs_hal/mss_hal.h>
#include <drivers/mss/mss_gpio/mss_gpio.h>

#include <string.h>



/// Prints on UART a message
///
static void
print(const uint8_t *const message, mss_uart_instance_t* uart)
{
    uint8_t msg[512];
    sprintf((char*)msg," %s", message);
    MSS_UART_polled_tx_string(uart, (const uint8_t*)msg);
}

/// Setup registry page
///
void
RFIM_Phy_setup_page(uint32_t base_addr, uint16_t phy_addr, uint16_t page)
{
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x1F, page);
}

/// Init the Phy
///
void
RFIM_Phy_init(uint32_t base_addr, uint16_t phy_addr)
{
    /// Do reset
    ///
    mss_config_clk_rst(MSS_PERIPH_GPIO2, (uint8_t) MPFS_HAL_FIRST_HART, PERIPHERAL_ON);

    MSS_GPIO_init(GPIO2_LO);
    MSS_GPIO_config(GPIO2_LO, MSS_GPIO_0, MSS_GPIO_OUTPUT_MODE);
    MSS_GPIO_config(GPIO2_LO, MSS_GPIO_1, MSS_GPIO_OUTPUT_MODE);

    for (uint8_t i = 0; i < 2; ++i)
    {
      /// Start reset
      ///
      MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_0, 1 );
      MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_1, 1 );

      /// Wait as specified in datasheet
      ///
      sleep_ms(200);

      /// Reset over
      ///
      MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_0, 0 );
      MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_1, 0 );

      /// After de-asserting rest, must wait again
      ///
      sleep_ms(200);

    }

    /// Hi again
    ///
    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_0, 1 );
    MSS_GPIO_set_output( GPIO2_LO, MSS_GPIO_1, 1 );

    /// The PHY reset may take up to 0.5 sec
    ///
    sleep_ms(500);

    RFIM_Phy_set_rgmii(base_addr, phy_addr);
    RFIM_Phy_reset(base_addr, phy_addr);

    /// Advertise 100BASE
    ///
    RFIM_Phy_setup_page(base_addr, phy_addr, 0x000);

    volatile uint16_t phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, 0x04);

    phy_reg &= ~((0x1 << 5) | (0x1 << 6) | (0x1 << 7) |  (0x1 << 8) | (0x1 << 9));
    phy_reg |= ((0x1 << 7) |  (0x1 << 8) | (0x1 << 9));

    RFIM_Phy_write_reg(base_addr, phy_addr, 0x04, phy_reg);

    /// Advertise 1000BASE-TX
    ///

    /// 1000Base Control: FDX
    ///
    phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, 0x09);

    /// Enable 1000BASE
    phy_reg |= ((0x1 << 8) | (0x1 << 9));
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x09, phy_reg);

}

/// Write some registry via MDIO
///
void
RFIM_Phy_write_reg(uint32_t base_addr, uint16_t phy_addr, uint32_t reg_addr, uint16_t reg_val)
{
    uint32_t timeout = 100000u;

    /// Disbale interrupts
    ///
    volatile psr_t lev = HAL_disable_interrupts();

    /// Prepare the register for write
    ///
    volatile uint32_t reg = RFIM_Phy_prepare_mgmt_reg(phy_addr, reg_addr, reg_val, MII_WRITE);

    /// Load value to be written in MII Mgmt control register
    ///
    HW_set_32bit_reg(base_addr + MIIMMGMT_REG_ADDR, reg);

    /// After the transaction is completed, bit 2 is set to 1
    /// in the network status register
    uint32_t bit2 = 0x0;

    /// Poll till PHY read cycle is completed or timeout expired
    /// Chek the network status for transaction completition
    ///
    do
    {
        --timeout;
        volatile uint32_t net_stat = HW_get_32bit_reg(base_addr + MIINETSTAT_REG_ADDR);

        bit2 = net_stat & (0x1 << 2u);

    } while ((timeout > 0u) && (bit2 == 0x0));


    HAL_ASSERT(timeout > 0u);

    /// Restore interrupts
    ///
    HAL_restore_interrupts(lev);
}

/// Read some registry via MDIO
///
uint16_t
RFIM_Phy_read_reg(uint32_t base_addr, uint16_t phy_addr, uint32_t reg_addr)
{
    uint32_t timeout = 100000u;
    uint16_t read_val = 0;

    volatile psr_t lev = HAL_disable_interrupts();

    volatile uint32_t result;

    /// Prepare the register for write
    ///
    volatile uint32_t reg = RFIM_Phy_prepare_mgmt_reg(phy_addr, reg_addr, 0x0u, MII_READ);

    ///Load value to be written in MII Mgmt control registe
    ///
    HW_set_32bit_reg(base_addr+MIIMMGMT_REG_ADDR, reg);

    /// Poll till PHY read cycle is completed or timeout expired
    /// Chek the network status for transaction completition
    ///
    uint32_t bit2 = 0x0;

    /// Poll till PHY read cycle is completed or timeout expired
    ///
    do
    {
        --timeout;
        result = HW_get_32bit_reg(base_addr + MIIMMGMT_REG_ADDR);

        volatile uint32_t net_stat = HW_get_32bit_reg(base_addr + MIINETSTAT_REG_ADDR);

        bit2 = net_stat & (0x1 << 2u);

    } while ((timeout > 0u) && (bit2 == 0x0));


    HAL_ASSERT(timeout > 0u);

    /// REad only if no timeout
    ///
    if (timeout > 0u)
    {
        read_val = result & 0xFFFF;
    }

    /// Restore this
    ///
    HAL_restore_interrupts(lev);

    return read_val;
}

/// Prepare the registry for use thru the MDIO
///
uint32_t
RFIM_Phy_prepare_mgmt_reg(uint16_t phy_addr, uint32_t reg_addr, uint16_t reg_val, uint8_t cmd)
{
    volatile uint32_t result = 0x0;

    /// PHY adddress
    ///
    result |= (phy_addr << MIIMADDR_PHY_ADDR_SHIFT);

    /// Register address
    ///
    result |= (reg_addr << MIIMADDR_REG_ADDR_SHIFT);

    /// Register value
    ///
    result |= reg_val;

    /// Read/Write command
    ///
    result |= (cmd << MIIMCTRL_PHY_CTRL_SHIFT);

    /// Write 10
    ///
    result |= (0x2u << 16u);

    /// Write 1
    ///
    result |= (0x1u << 30u);

    return result;
}

/// Reset the phy
///
void
RFIM_Phy_reset(uint32_t base_addr, uint16_t phy_addr)
{
    /// Setup page 0
    ///
    RFIM_Phy_setup_page(base_addr, phy_addr, 0x0000);

    /// Software rest
    ///
    volatile uint32_t phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, 0x00);

    phy_reg = phy_reg | 0x8000;

    RFIM_Phy_write_reg(base_addr, phy_addr, 0x00, phy_reg);

    phy_reg = 0x8000;
    while ((phy_reg & 0x8000) != 0)
        phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, 0x00);
}

/// Setup RGMII
///
void
RFIM_Phy_set_rgmii(uint32_t base_addr, uint16_t phy_addr)
{
    /// Setup page 0
    ///
    RFIM_Phy_setup_page(base_addr, phy_addr, 0x0000);

    volatile uint32_t reg = 0x0;

    /// Setup RGMII
    ///
    reg = (0x0 << 13) | (0x2 << 11);
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x17, reg);
}

/// Get the link status
///
uint8_t
RFIM_Phy_get_link_status(uint32_t base_addr, uint16_t phy_addr, uint8_t* speed, uint8_t* fullduplex)
{
    /// Get the current registry page
    ///
    volatile uint16_t old_page = RFIM_Phy_read_reg(base_addr, phy_addr, 0x1FU);

    /// Write the page 0 we want
    ///
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x1FU, 0x0u);

    /// Read registry 1 - Mode status
    ///
    volatile uint16_t phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, MII_BMSR);

    uint16_t link_up;
    uint8_t  link_status;

    /// Link status, bit Bit 2
    /// 0 : Link is down
    /// 1 : Link is up
    link_up = phy_reg & MII_BMSR_LSTATUS;

    /// Write back the old page
    ///
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x1FU, old_page);

    /// The logic goes here
    ///
    if (link_up != PHY_LINK_DOWN)
    {
        /// Link is up.
        link_status = PHY_LINK_UP;

        if (phy_reg & RGMII_100BASE_T4)
        {
            *fullduplex = PHY_HALF_DUPLEX;
            *speed = PHY_MAC100MBPS;
        }
        if (phy_reg & RGMII_100BASE_TX_FD)
        {
            *fullduplex = PHY_FULL_DUPLEX;
            *speed = PHY_MAC100MBPS;
        }
        else if (phy_reg & RGMII_100BASE_TX_HD)
        {
            *fullduplex = PHY_HALF_DUPLEX;
            *speed = PHY_MAC100MBPS;
        }
        else if (phy_reg & RGMII_10BASE_T_FD)
        {
            *fullduplex = PHY_FULL_DUPLEX;
            *speed = PHY_MAC10MBPS;
        }
        else if (phy_reg & RGMII_10BASE_T_HD)
        {
            *fullduplex = PHY_HALF_DUPLEX;
            *speed = PHY_MAC10MBPS;
        }
        else if (phy_reg & RGMII_100BASE_T2_FD)
        {
            *fullduplex = PHY_FULL_DUPLEX;
            *speed = PHY_MAC100MBPS;
        }
        else if (phy_reg & RGMII_100BASE_T2_HD)
        {
            *fullduplex = PHY_HALF_DUPLEX;
            *speed = PHY_MAC100MBPS;
        }
        else
        {
            *fullduplex = PHY_FULL_DUPLEX;
            *speed = PHY_INVALID_SPEED;
        }
    }
    else
    {
        /// Link is down.
        link_status = PHY_LINK_DOWN;

        *fullduplex = PHY_FULL_DUPLEX;
        *speed = PHY_INVALID_SPEED;
    }

    return link_status;
}

/// Setup link speed
///
void
RFIM_Phy_set_speed(uint32_t base_addr, uint16_t phy_addr, uint8_t speed)
{
    /// Forced speed selection
    ///     00: 10 Mbps
    ///     01: 100 MBps
    ///     10: 1000 Mbps
    ///
    /// Bit 6:  Most significant bit
    /// Bit 13: Least significant bit
    ///
    ///

    #define MSB 6
    #define LSB 13

    uint16_t speed_bit_LSB = 0x0u;
    uint16_t speed_bit_MSB = 0x0u;
    uint16_t speed_bit_LSB_mask = 0x1u << LSB;
    uint16_t speed_bit_MSB_mask = 0x1u << MSB;

    if (PHY_MAC1000MBPS == speed)
    {
        speed_bit_MSB = 0x1u;
    }
    if (PHY_MAC100MBPS == speed)
    {
        speed_bit_LSB = 0x1u;
    }

    /// Get the current registry page
    ///
    volatile uint16_t old_page = RFIM_Phy_read_reg(base_addr, phy_addr, 0x1FU);

    /// Write the page 0 we want
    ///
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x1FU, 0x0u);

    /// Read registry 0 - Mode control
    ///
    volatile uint16_t reg0 = RFIM_Phy_read_reg(base_addr, phy_addr, 0x0u);

    /// Reset the speed LSB and MSB bits
    ///
    reg0 &= ~(speed_bit_LSB_mask | speed_bit_MSB_mask);

    /// Set the speed
    ///
    reg0 |= (speed_bit_LSB << LSB) | (speed_bit_MSB << MSB);

    /// Write back the register 0
    ///
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x0U, reg0);

    /// Write back the old page
    ///
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x1FU, old_page);
}

/// Turn on/off leds
///
void
RFIM_Phy_leds_onoff(uint32_t base_addr, uint16_t phy_addr, uint8_t on_off)
{
    volatile uint16_t reg = 0x0;

    /// Access main registry space
    ///
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x1F, 0x000);

    /// LED mode select
    ///
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x1D, 14 | (14 << 4)); //Force LED Off for LED0 and LED1

    /// Access E2 registry space
    ///
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x1F, 0x002);

    /// Setting the state of the LEDs
    ///
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x11u, (uint16_t)(0x0101u << 10u)); /// Sets the State to 1

    /// Access main registry space
    ///
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x1F, 0x000);

    /// Software rest
    ///
   RFIM_Phy_reset(base_addr, phy_addr);
}

void
RFIM_Phy_autonegotiate(uint32_t base_addr, uint8_t phy_addr)
{
    volatile uint32_t phy_reg;
    uint16_t autoneg_complete;
    volatile uint32_t copper_aneg_timeout = 1000000u;
    volatile uint32_t mii_aneg_timeout = 1000000u;

    RFIM_Phy_setup_page(base_addr, phy_addr, 0x0000);

    phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, 0x00);
    phy_reg |= 0x1200;
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x00, phy_reg);

    do
    {
        phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, 0x01);
        autoneg_complete = phy_reg & 0x0020u;
        --copper_aneg_timeout;
    } while (!autoneg_complete && (copper_aneg_timeout != 0u));

    /// if copper link up
    if ((phy_reg & 0x0004) != 0u)
    {
        uint8_t phy2_addr = 0x12;

        /// enable autonegotiation
        phy_reg = RFIM_Phy_read_reg(base_addr, phy2_addr, 0x00);
        phy_reg |= 0x1000;
        RFIM_Phy_write_reg(base_addr, phy2_addr, 0x00, phy_reg);

        /// restart autonegotiation
        phy_reg = RFIM_Phy_read_reg(base_addr, phy2_addr, 0x00);
        phy_reg |= 0x0200;
        RFIM_Phy_write_reg(base_addr, phy2_addr, 0x00, phy_reg);

        do
        {
            phy_reg = RFIM_Phy_read_reg(base_addr, phy2_addr, 0x01);
            autoneg_complete = phy_reg & 0x0020;
            --mii_aneg_timeout;
        } while ((!autoneg_complete) && (mii_aneg_timeout != 0u));
    }
}

/// Gets the phy id
///
void
RFIM_Phy_get_id(uint32_t base_addr, uint16_t phy_addr, uint16_t* id_1, uint16_t* id_2)
{
    /// Setup page 0
    ///
    RFIM_Phy_setup_page(base_addr, phy_addr, 0x000);

    /// Read the first id
    ///
    *id_1 = RFIM_Phy_read_reg(base_addr, phy_addr, 0x2);

    /// Read the second id
    ///
    *id_2 = RFIM_Phy_read_reg(base_addr, phy_addr, 0x3);
}

/// Dump Phy ID
///
void
RFIM_Phy_dump_id(uint32_t base_addr, uint32_t phy_addr, mss_uart_instance_t* uart)
{
    uint16_t phy_id_1 = 0x0;
    uint16_t phy_id_2 = 0x0;

    RFIM_Phy_get_id(base_addr, phy_addr, &phy_id_1, &phy_id_2);

    uint8_t     info[256];
    uint16_t    gem = base_addr == MAC0_BASE ? 0 : 1;

    if (phy_id_1 == 0x0007) {
        if (phy_id_2 == 0x0771) {
            sprintf((char*)info, "GEM %d PHY Model vsc8541-01 rev b\n\r", gem);
            print(info, uart);
            return;
        }
        if (phy_id_2 == 0x0772) {
            sprintf((char*)info, "GEM %d PHY Model vsc8541-02/-05 rev c\n\r", gem);
            print(info, uart);
            return;
        }
    }

    sprintf((char*)info, "GEM %d PHY Id is %#.4x - %#.4x\n\r", gem, phy_id_1, phy_id_2);
    print(info, uart);
}

/// Dump Phy link status
///
void
RFIM_Phy_dump_link_status(uint32_t base_addr, uint32_t phy_addr, mss_uart_instance_t* uart)
{
    uint8_t speed   = PHY_INVALID_SPEED;
    uint8_t link    = PHY_LINK_DOWN;
    uint8_t duplex  = PHY_HALF_DUPLEX;

    uint8_t speed_info[10];
    uint8_t linkup_info[5];
    uint8_t duplex_info[5];

    link = RFIM_Phy_get_link_status(base_addr, phy_addr, &speed, &duplex);

    sprintf((char*)speed_info,"%s", (speed == PHY_MAC10MBPS ? "10 Mbps" : (speed == PHY_MAC100MBPS ? "100 Mbps" : (speed == PHY_MAC1000MBPS ? "1000 Mbps" : "Unknown")) ));
    sprintf((char*)linkup_info,"%s", link  == PHY_LINK_UP ? "yes" : "no");
    sprintf((char*)duplex_info,"%s", duplex == PHY_FULL_DUPLEX ? "yes" : "no");

    uint16_t gem = base_addr == MAC0_BASE ? 0 : 1;

    uint8_t info[256];
    sprintf((char*)info, "GEM %d link up: %s, speed: %s fullduplex: %s\n\r", gem, linkup_info, speed_info, duplex_info);
    print(info, uart);
}
