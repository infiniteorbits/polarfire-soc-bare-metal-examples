///
/// File            | RFIM_Phy.c
/// Description     | Phy driver for Microchip Polarfire SoC
///
/// Author          | Trajce Nikolov    | trajce.nikolov.nick@gmail.com
///                                     | trajce.nikolov.nick@outlook.com
/// Date            | September 2025
///
/// Copyright       | RFIM Space
///

#include "RFIM_Phy.h"

#include <hal/hal.h>
#include <hal/hal_assert.h>

#include <mpfs_hal/mss_hal.h>

#include <drivers/mss/mss_gpio/mss_gpio.h>


#include <string.h>

volatile uint16_t Phy0_reg_0[32];
volatile uint16_t Phy0_reg_1[16];
volatile uint16_t Phy0_reg_2[16];
volatile uint16_t Phy0_reg_16[32];

volatile uint16_t Phy1_reg_0[32];
volatile uint16_t Phy1_reg_1[16];
volatile uint16_t Phy1_reg_2[16];
volatile uint16_t Phy1_reg_16[32];



/// Prints on UART a message
///
void
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

/// General purpose initialization - The first to call
///
void
RFIM_init(void)
{
    /// Init GPIO
    ///
    mss_config_clk_rst(MSS_PERIPH_GPIO2, (uint8_t) MPFS_HAL_FIRST_HART, PERIPHERAL_ON);

    MSS_GPIO_init(GPIO2_LO);
}

/// Reset using GPIO
///
void
RFIM_Phy_reset_w_gpio(uint32_t base_addr)
{
    uint8_t GPIO_ID = base_addr == MAC0_BASE ? MSS_GPIO_0 : MSS_GPIO_1;

    MSS_GPIO_config(GPIO2_LO, GPIO_ID, MSS_GPIO_OUTPUT_MODE);

    for (uint8_t i = 0; i < 2; ++i)
    {
      /// Start reset
      ///
      MSS_GPIO_set_output( GPIO2_LO, GPIO_ID, 1 );

      /// Wait as specified in datasheet
      ///
      sleep_ms(200);

      /// Reset over
      ///
      MSS_GPIO_set_output( GPIO2_LO, GPIO_ID, 0 );

      /// After de-asserting rest, must wait again
      ///
      sleep_ms(200);

    }

    /// Set it to hi again
    ///
    MSS_GPIO_set_output( GPIO2_LO, GPIO_ID, 1 );

    /// The PHY reset may take up to 0.5 sec
    ///
    sleep_ms(500);
}

/// Setup speed advertisement
///
void
RFIM_Phy_advertise(uint32_t base_addr, uint16_t phy_addr, uint16_t adv_mask)
{
    /// Advertise 100BASE
    ///
    RFIM_Phy_setup_page(base_addr, phy_addr, 0x000);

    volatile uint16_t phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, 0x04);

    /// Reset to nothing
    ///
    phy_reg &= ~((0x1 << 5) | (0x1 << 6) | (0x1 << 7) |  (0x1 << 8) | (0x1 << 9));

    /// :Let use the same enum
    ///
    if (adv_mask & RGMII_10BASE_T_HD)       phy_reg |= (0x1 << 5);
    if (adv_mask & RGMII_10BASE_T_FD)       phy_reg |= (0x1 << 6);
    if (adv_mask & RGMII_100BASE_TX_HD)     phy_reg |= (0x1 << 7);
    if (adv_mask & RGMII_100BASE_TX_FD)     phy_reg |= (0x1 << 8);

    RFIM_Phy_write_reg(base_addr, phy_addr, 0x04, phy_reg);

    /// Advertise 1000BASE-TX
    ///

    /// 1000Base Control: FDX
    ///
    phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, 0x09);

    /// Reset to nothing
    ///
    phy_reg &= ~((0x1 << 8) | (0x1 << 9));

    /// Enable 1000BASE
    if (adv_mask & RGMII_1000BASE_TX_HD)    phy_reg |= (0x1 << 8);
    if (adv_mask & RGMII_1000BASE_TX_FD)    phy_reg |= (0x1 << 9);

    RFIM_Phy_write_reg(base_addr, phy_addr, 0x09, phy_reg);
}

/// Setup RGMII clk delay
///
void
RFIM_Phy_set_rgmii_tx_rx_delay(uint32_t base_addr, uint16_t phy_addr)
{
    /// Reg 20E2
    ///
    RFIM_Phy_setup_page(base_addr, phy_addr, 2);

    volatile uint32_t reg = RFIM_Phy_read_reg(base_addr, phy_addr, 20);

    /// TODO: Setup RGMII tx rx clk delay
    ///

    reg &= ~(0x1 << 11); /// Set bit 11 to 0
    RFIM_Phy_write_reg(base_addr, phy_addr, 20, reg);
}

/// Init the Phy
///
void
RFIM_Phy_init(uint32_t base_addr, uint16_t phy_addr)
{
    RFIM_Phy_reset_w_gpio(base_addr);
    RFIM_Phy_set_rgmii(base_addr, phy_addr);
    RFIM_Phy_reset(base_addr, phy_addr);
    RFIM_Phy_set_rgmii_tx_rx_delay(base_addr, phy_addr);
    RFIM_Phy_advertise(base_addr, phy_addr, RGMII_100BASE_TX_FD | RGMII_1000BASE_TX_FD);
}

/// Init the Phy with default speed
///
void
RFIM_Phy_init_w_def_speed(uint32_t base_addr, uint16_t phy_addr, uint8_t speed, uint8_t duplex)
{
    RFIM_Phy_init(base_addr, phy_addr);
    RFIM_Phy_set_speed(base_addr, phy_addr, speed, duplex);
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


    ASSERT(timeout > 0u);

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


    ASSERT(timeout > 0u);

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
    volatile uint32_t timeout = 1000000u;

    /// Setup page 0
    ///
    RFIM_Phy_setup_page(base_addr, phy_addr, 0x0000);

    /// Software rest
    ///
    volatile uint32_t phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, 0x00);

    phy_reg = phy_reg | 0x8000;

    RFIM_Phy_write_reg(base_addr, phy_addr, 0x00, phy_reg);

    phy_reg = 0x8000;
    while ((phy_reg & 0x8000) != 0 && timeout-- > 0)
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

    volatile uint32_t reg = RFIM_Phy_read_reg(base_addr, phy_addr, 0x17);

    /// Setup RGMII
    ///
    reg |= (0x2 << 11);
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x17, reg);
}

/// Get the link status
///
uint8_t
RFIM_Phy_get_link_status(uint32_t base_addr, uint16_t phy_addr, uint8_t* speed, uint8_t* fullduplex)
{
    /// Write the page 0 we want
    ///
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x1FU, 0x0u);

    /// Read registry 1 - Mode status
    ///
    volatile uint16_t phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, PHY_MII_BMSR);

    uint16_t link_up;
    uint8_t  link_status;

    /// Link status, bit Bit 2
    /// 0 : Link is down
    /// 1 : Link is up
    link_up = phy_reg & PHY_MII_BMSR_LSTATUS;

    /// The logic goes here
    ///
    if (link_up != PHY_LINK_DOWN)
    {
        /// Link is up.
        link_status = PHY_LINK_UP;

        /// Check 1000 BASE
        ///
        phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, 10);

        if (phy_reg & (0x1 << 10u)) /// 1000 HD
        {
            *fullduplex = PHY_HALF_DUPLEX;
            *speed = PHY_MAC1000MBPS;
        }
        else
        if (phy_reg & (0x1 << 11u)) /// 1000 FD
        {
            *fullduplex = PHY_FULL_DUPLEX;
            *speed = PHY_MAC1000MBPS;
        }
        else
        {
            phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, PHY_MII_BMSR);

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
            else
            {
                *fullduplex = PHY_FULL_DUPLEX;
                *speed = PHY_INVALID_SPEED;
            }
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
RFIM_Phy_set_speed(uint32_t base_addr, uint16_t phy_addr, uint8_t speed, uint8_t duplex)
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

    /// Set duplex
    ///
    reg0 &= ~(0x1 << 8); /// Reset first
    reg0 |= (duplex & 0x1) << 8;

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

    /// Access E1 registry space
    ///
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x1F, 0x0001);

    /// Set LED Extended Mode in registry 19E1
    ///
    reg = RFIM_Phy_read_reg(base_addr, phy_addr, 19u);
    reg |= (0x1 << 12);         /// LED 0
    reg |= (0x1 << 13);         /// LED 1
    RFIM_Phy_write_reg(base_addr, phy_addr, 19u, reg);

    /// Access main registry space
    ///
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x1F, 0x0000);

    /// LED mode select
    /// Mode 20 - LEDs Off
    /// Mode 21 - LEDs On
    ///
    /// To understand this, let say mode 20 is 0001 0100
    /// When the extended mode is on, the first half is 0001 and the second should
    /// complement to the byte value to create 20, which is 0100 (this is only 4)
    /// we output the second part  as Mode
    ///
    reg = RFIM_Phy_read_reg(base_addr, phy_addr, 29u); ///LED Mode Select
    reg  |= (on_off == PHY_LEDS_OFF ? 0x4 | (0x4 << 4) : 0x5 | (0x5 << 4));
    RFIM_Phy_write_reg(base_addr, phy_addr, 29u, reg); ///LED Mode Select

    /// Access E2 register space
    ///
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x1F, 0x0002);

    /// Register 17E2
    /// Invert the polarity, set the led state
    ///
    reg = RFIM_Phy_read_reg(base_addr, phy_addr, 17u);
    if (on_off == PHY_LEDS_OFF)
    {
        reg = ((1 << 10) | (1 << 11));
    }
    else
    {
        reg &= ~((1 << 10) | (1 << 11));
    }
    RFIM_Phy_write_reg(base_addr, phy_addr, 17u, reg);
}

void
RFIM_Phy_autonegotiate(uint32_t base_addr, uint8_t phy_addr)
{
    volatile uint32_t phy_reg;
    uint16_t autoneg_complete;
    volatile uint32_t copper_aneg_timeout = 1000000u;
    volatile uint32_t mii_aneg_timeout = 1000000u;
    volatile int32_t downshift_timeout = 10000u;

    /// 20E1
    /// Downshift
    ///
    RFIM_Phy_setup_page(base_addr, phy_addr, 1);

    phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, 20);
    phy_reg |= (0x1 << 4);  /// Enable downshift
    RFIM_Phy_write_reg(base_addr, phy_addr, 20, phy_reg);

    /// 0
    ///
    RFIM_Phy_setup_page(base_addr, phy_addr, 0x0000);

    phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, 0x00);
    phy_reg |= (0x1 << 12); /// Enable autonegotiation
    RFIM_Phy_write_reg(base_addr, phy_addr, 0x00, phy_reg);

    do
    {
        phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, 0x01);
        autoneg_complete = phy_reg & (0x1 << 5);
        /// 20E1
        ///
        RFIM_Phy_setup_page(base_addr, phy_addr, 1);

        downshift_timeout = 10000u;
        do
        {
           phy_reg = RFIM_Phy_read_reg(base_addr, phy_addr, 20);

           /// bit 1 is set if Downshift is required or occured
           ///
        } while ((phy_reg & 0x2u) == 0u &&  ((downshift_timeout--) > 0u));
        --copper_aneg_timeout;
    } while (!autoneg_complete && (copper_aneg_timeout != 0u));
}

void
RFIM_Phy_loopback_autonegotiate(mss_uart_instance_t* uart, uint32_t base_addr0, uint8_t phy_addr0, uint32_t base_addr1, uint8_t phy_addr1)
{
    volatile uint32_t phy_reg0;
    volatile uint32_t phy_reg1;

    uint16_t autoneg_complete;

    volatile uint32_t copper_aneg_timeout = 1000000u;
    volatile uint32_t mii_aneg_timeout = 1000000u;
    volatile int32_t downshift_timeout = 10000u;

    /// 20E1
    /// Downshift
    ///
    RFIM_Phy_setup_page(base_addr1, phy_addr1, 1);

    phy_reg1 = RFIM_Phy_read_reg(base_addr1, phy_addr1, 20);
    phy_reg1 |= (0x1 << 4);  /// Enable downshift
    RFIM_Phy_write_reg(base_addr1, phy_addr1, 20, phy_reg1);

    RFIM_Phy_setup_page(base_addr0, phy_addr0, 0x0000);
    RFIM_Phy_setup_page(base_addr1, phy_addr1, 0x0000);

    phy_reg0 = RFIM_Phy_read_reg(base_addr0, phy_addr0, 0x00);
    phy_reg0 |= (0x1 << 12);
    RFIM_Phy_write_reg(base_addr0, phy_addr0, 0x00, phy_reg0);

    phy_reg1 = RFIM_Phy_read_reg(base_addr1, phy_addr1, 0x00);
    phy_reg1 |= (0x1 << 12);
    RFIM_Phy_write_reg(base_addr1, phy_addr1, 0x00, phy_reg1);

    do
    {
        phy_reg0 = RFIM_Phy_read_reg(base_addr0, phy_addr0, 0x01);
        phy_reg1 = RFIM_Phy_read_reg(base_addr1, phy_addr1, 0x01);

        autoneg_complete = phy_reg0 & (0x1 << 5);
        autoneg_complete = autoneg_complete && (phy_reg1 & (0x1 << 5));

        /// 20E1
        ///
        RFIM_Phy_setup_page(base_addr1, phy_addr1, 1);

        downshift_timeout = 10000u;
        do
        {
           phy_reg1 = RFIM_Phy_read_reg(base_addr1, phy_addr1, 20);

           /// bit 1 is set if Downshift is required or occured
           ///
        } while ((phy_reg1 & 0x2u) == 0u &&  ((downshift_timeout--) > 0u));

        /// Page 0
        ///
        RFIM_Phy_setup_page(base_addr1, phy_addr1, 0x0000);

        --copper_aneg_timeout;
    } while (!autoneg_complete && (copper_aneg_timeout != 0u));

    /// downshift happened
    ///
    if (downshift_timeout > 0u)
        print("Link speed downshift occured\n\r", uart);

    /// if copper link up
    ///
    if (((phy_reg0 & 0x0004) != 0u) && ((phy_reg1 & 0x0004) != 0u))
    {
        /// Page 0
        ///
        RFIM_Phy_setup_page(base_addr1, phy_addr1, 0x0000);

        /// enable autonegotiation
        ///
        phy_reg0 = RFIM_Phy_read_reg(base_addr0, phy_addr0, 0x00);
        phy_reg0 |= 0x1000;
        RFIM_Phy_write_reg(base_addr0, phy_addr0, 0x00, phy_reg0);

        phy_reg1 = RFIM_Phy_read_reg(base_addr1, phy_addr1, 0x00);
        phy_reg1 |= 0x1000;
        RFIM_Phy_write_reg(base_addr1, phy_addr1, 0x00, phy_reg1);

        /// restart autonegotiation
        ///
        phy_reg0 = RFIM_Phy_read_reg(base_addr0, phy_addr0, 0x00);
        phy_reg0 |= (0x1 << 12);
        RFIM_Phy_write_reg(base_addr0, phy_addr0, 0x00, phy_reg0);

        phy_reg1 = RFIM_Phy_read_reg(base_addr1, phy_addr1, 0x00);
        phy_reg1 |= (0x1 << 12);
        RFIM_Phy_write_reg(base_addr1, phy_addr1, 0x00, phy_reg1);

        do
        {
            phy_reg0 = RFIM_Phy_read_reg(base_addr0, phy_addr0, 0x01);
            phy_reg1 = RFIM_Phy_read_reg(base_addr1, phy_addr1, 0x01);

            autoneg_complete = phy_reg0 & (0x1 << 5);
            autoneg_complete = autoneg_complete && (phy_reg1 & (0x1 << 5));
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
    sprintf((char*)duplex_info,"%s", link == PHY_LINK_DOWN ? "Unknown" : duplex == PHY_FULL_DUPLEX ? "yes" : "no");

    uint16_t gem = base_addr == MAC0_BASE ? 0 : 1;

    uint8_t info[256];
    sprintf((char*)info, "GEM %d link up: %s, speed: %s fullduplex: %s\n\r", gem, linkup_info, speed_info, duplex_info);
    print(info, uart);
}

/// Dumps the leds status on uart
///
void
RFIM_Phy_dump_leds_status(uint32_t base_addr, uint16_t phy_addr, mss_uart_instance_t* uart)
{

}



/// Dumps the phy registers
///
void
RFIM_Phy_dump_regs(uint32_t base_addr, uint16_t phy_addr)
{
    volatile int32_t    count;
    volatile uint16_t   page;
    volatile uint16_t   old_page;
    volatile uint16_t*  pdata;

    uint8_t is_phy0 = phy_addr == PHY0_ADDR ? 1 : 0;

    for (page = 0U; page <= 0x10U; page++)
    {
        if (0U == page)
        {
            pdata = is_phy0 ? Phy0_reg_0 : Phy1_reg_0;
        }
        else if (1U == page)
        {
            pdata = is_phy0 ? Phy0_reg_1 : Phy1_reg_1;
        }
        else if (2U == page)
        {
            pdata = is_phy0 ? Phy0_reg_2 : Phy1_reg_2;
        }
        else if (16U == page)
        {
            pdata = is_phy0 ? Phy0_reg_16 : Phy1_reg_16;
        }
        else
        {
            pdata = pdata = is_phy0 ? Phy0_reg_0 : Phy1_reg_0;
        }

        if ((0U == page) || (0x10U == page))
        {
            for (count = 0; count <= 0x1F; count++)
            {
                /// Get the current registry page
                ///
                old_page = RFIM_Phy_read_reg(base_addr, phy_addr, 0x1FU);

                /// Write the page we want
                ///
                RFIM_Phy_write_reg(base_addr, phy_addr, 0x1FU, page);

                /// Read
                ///
                pdata[count] = RFIM_Phy_read_reg(base_addr, phy_addr, count);

                /// Write back the old page
                ///
                RFIM_Phy_write_reg(base_addr, phy_addr, 0x1FU, old_page);
            }
        }
        else
        {
            for (count = 0x10; count <= 0x1F; count++)
            {
                /// Get the current registry page
                ///
                old_page = RFIM_Phy_read_reg(base_addr, phy_addr, 0x1FU);

                /// Write the page we want
                ///
                RFIM_Phy_write_reg(base_addr, phy_addr, 0x1FU, page);

                /// Read
                ///
                pdata[count - 0X10] = RFIM_Phy_read_reg(base_addr, phy_addr, count);

                /// Write back the old page
                ///
                RFIM_Phy_write_reg(base_addr, phy_addr, 0x1FU, old_page);
            }
        }

        if (2U == page)
        {
            page = 0x0FU;
        }
    }
}

/// MSS compatible
///
void
MSS_MAC_RFIM_phy_init(/* mss_mac_instance_t */ const void *v_this_mac, uint8_t phy_addr)
{
    const mss_mac_instance_t* this_mac = (const mss_mac_instance_t*) v_this_mac;
    uint32_t base_addr = (uint64_t)(this_mac->mac_base) & 0xFFFFFFFF;

    RFIM_Phy_init(base_addr, this_mac->phy_addr);
}

void
MSS_MAC_RFIM_phy_set_link_speed(/* mss_mac_instance_t */ void *v_this_mac, uint32_t speed_duplex_select, mss_mac_speed_mode_t speed_mode)
{
    /// #define PHY_MAC10MBPS           0x00
    /// #define PHY_MAC100MBPS          0x01
    /// #define PHY_MAC1000MBPS         0x02
    ///

    /// MSS_MAC_10_HDX     = 0x01, /*!< Link operates in 10M half duplex mode */
    /// MSS_MAC_10_FDX     = 0x02, /*!< Link operates in 10M full duplex mode */
    /// MSS_MAC_100_HDX    = 0x03, /*!< Link operates in 100M half duplex mode */
    /// MSS_MAC_100_FDX    = 0x04, /*!< Link operates in 100M full duplex mode */
    /// MSS_MAC_1000_HDX   = 0x05, /*!< Link operates in 1000M half duplex mode */
    /// MSS_MAC_1000_FDX   = 0x06, /*!< Link operates in 1000M full duplex mode */
    ///

    uint8_t speed = PHY_MAC100MBPS;
    uint8_t duplex = PHY_FULL_DUPLEX;

    switch (speed_mode)
    {
    case MSS_MAC_10_HDX:
        speed = PHY_MAC10MBPS;
        duplex = PHY_HALF_DUPLEX;
        break;
    case MSS_MAC_10_FDX:
        speed = PHY_MAC10MBPS;
        duplex = PHY_FULL_DUPLEX;
        break;
    case MSS_MAC_100_HDX:
        speed = PHY_MAC100MBPS;
        duplex = PHY_HALF_DUPLEX;
        break;
    case MSS_MAC_1000_HDX:
        speed = PHY_MAC1000MBPS;
        duplex = PHY_HALF_DUPLEX;
        break;
    case MSS_MAC_1000_FDX:
        speed = PHY_MAC1000MBPS;
        duplex = PHY_FULL_DUPLEX;
        break;
    }

    const mss_mac_instance_t* this_mac = (const mss_mac_instance_t*) v_this_mac;
    uint32_t base_addr = (uint64_t)(this_mac->mac_base) & 0xFFFFFFFF;

    RFIM_Phy_set_speed(base_addr, this_mac->phy_addr, speed, duplex);
}

void
MSS_MAC_RFIM_phy_autonegotiate(/* mss_mac_instance_ t */ const void *v_this_mac)
{
    const mss_mac_instance_t* this_mac = (const mss_mac_instance_t*) v_this_mac;
    uint32_t base_addr = (uint64_t)(this_mac->mac_base) & 0xFFFFFFFF;

    RFIM_Phy_autonegotiate(base_addr, this_mac->phy_addr);
}

void
MSS_MAC_RFIM_phy_mac_autonegotiate(/* mss_mac_instance_t */ const void *v_this_mac)
{
    (void)v_this_mac;
}

uint8_t
MSS_MAC_RFIM_phy_get_link_status(/* mss_mac_instance_t */ const void *v_this_mac, mss_mac_speed_t *speed, uint8_t *fullduplex)
{
    uint8_t phy_speed = PHY_INVALID_SPEED;
    *fullduplex = PHY_FULL_DUPLEX;

    const mss_mac_instance_t* this_mac = (const mss_mac_instance_t*) v_this_mac;
    uint32_t base_addr = (uint64_t)(this_mac->mac_base) & 0xFFFFFFFF;

    uint8_t linkUp = RFIM_Phy_get_link_status(base_addr, this_mac->phy_addr, &phy_speed, fullduplex);

    switch (phy_speed)
    {
    case PHY_MAC10MBPS:
        *speed = *fullduplex == PHY_FULL_DUPLEX ? MSS_MAC_10_FDX : MSS_MAC_10_HDX;
        break;
    case PHY_MAC100MBPS:
        *speed = *fullduplex == PHY_FULL_DUPLEX ? MSS_MAC_100_FDX : MSS_MAC_100_HDX;
        break;
    case PHY_MAC1000MBPS:
        *speed = *fullduplex == PHY_FULL_DUPLEX ? MSS_MAC_1000_FDX : MSS_MAC_1000_HDX;
        break;
    }

    return linkUp;
}


