/*
 * net.c
 *
 *  Created on: 29 Oct 2025
 *      Author: yacin
 */

#include "RFIM_Phy.h"

#include <ctype.h>
#include <string.h>

#include "mpfs_hal/mss_hal.h"
#include "mpfs_hal/common/nwc/mss_nwc_init.h"
#include "mpfs_hal/common/mss_plic.h"

#include "drivers/mss/mss_gpio/mss_gpio.h"
#include "drivers/mss/mss_mmuart/mss_uart.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_registers.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_mac_sw_cfg.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_mac_regs.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_mac.h"
#include "drivers/mss/mss_ethernet_mac/phy.h"
#include "drivers/mss/mss_gpio/mss_gpio.h"

static mss_mac_instance_t*          g_test_mac = &g_mac0;
static mss_mac_cfg_t                g_mac_config = {0};


#define UART                &g_mss_uart0_lo
#define USE_STDLIB_STDIO

static void PRINT_STRING(uint8_t* x)
{
#ifdef USE_STDLIB_STDIO
    uint8_t message[512];
    sprintf(message," %s", (x));
    MSS_UART_polled_tx_string(UART, (uint8_t *)&message[0]);
#else
    MSS_UART_polled_tx_string(UART, x);
#endif
}

#if defined(MSS_MAC_USE_DDR)
static uint8_t *g_mac_rx_buffer = 0;
#else
static uint8_t g_mac_rx_buffer[MSS_MAC_RX_RING_SIZE][MSS_MAC_MAX_RX_BUF_SIZE]
    __attribute__((aligned(16)));
#endif



void init_net(void)
{


    PLIC_init();
    __enable_irq();

    PLIC_EnableIRQ(PLIC_MAC0_INT_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_QUEUE1_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_QUEUE2_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_QUEUE3_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_EMAC_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_MMSL_INT_OFFSET);

    __enable_irq();

    uint32_t count;

    #if defined(MSS_MAC_USE_DDR)
        g_mac_rx_buffer = g_mss_mac_ddr_ptr;
        g_mss_mac_ddr_ptr += MSS_MAC_RX_RING_SIZE * MSS_MAC_MAX_RX_BUF_SIZE;
    #endif
    /*-------------------------- Initialize the MAC --------------------------*/
    /*
     * Get the default configuration for the Ethernet MAC and change settings
     * to match the system/application. The default values typically changed
     * are:
     *  - interface:
     *      Specifies the interface used to connect the Ethernet MAC to the PHY.
     *      Example choice are MII, GMII, TBI.
     *  - phy_addr:
     *      Specifies the MII management interface address of the external PHY.
     *  - mac_addr:
     *      Specifies the MAC address of the device. This number should be
     *      unique on the network the device is connected to.
     *  - speed_duplex_select:
     *      Specifies the allowed speed and duplex mode for setting up a link.
     *      This can be used to specify the set of allowed speeds and duplex
     *      modes used during auto-negotiation or force the link speed to a
     *      specific speed and duplex mode.
     */
    MSS_MAC_cfg_struct_def_init(&g_mac_config);

    /// Common setup
    ///
    g_mac_config.mac_addr[0] = 0x00;
    g_mac_config.mac_addr[1] = 0xFC;
    g_mac_config.mac_addr[2] = 0x00;
    g_mac_config.mac_addr[3] = 0x12;
    g_mac_config.mac_addr[4] = 0x34;
    g_mac_config.mac_addr[5] = 0x56;
    g_mac_config.interface_type = GMII;
    g_mac_config.tsu_clock_select = 0U;
    g_mac_config.phy_type = MSS_MAC_DEV_PHY_VSC8541;
    g_mac_config.phy_addr = PHY0_ADDR;

    /// Mandatory
    ///
    RFIM_init();

    /// Make sure this one is correct, defined in
    /// drivers/mss/mss_ethernet_mac/mss_ethernet_mac.c
    ///
    /// ASSERT( MSS_MAC0_BASE == MAC0_BASE )
    /// ASSERT( MSS_MAC1_BASE == MAC1_BASE )

    MSS_MAC_init(&g_mac0, &g_mac_config);

    RFIM_MAC0_phy_init();

    PRINT_STRING("\n\r");

    RFIM_MAC0_phy_autonegotiate();
    RFIM_MAC0_phy_dump_id(UART);

    PRINT_STRING("\n\r");
    PRINT_STRING("Default link state:\n\r");

    RFIM_MAC0_phy_dump_link_status(UART);

    PRINT_STRING("\n\r");
    PRINT_STRING("Switching phy leds on\n\r\n\r");

    /*
     * Not doing the tx disable/enable sequence here around the queue allocation
     * adjustment results in tx_go being set which causes the new tx code to
     * hang waiting for the last tx to complete...
     */
    g_test_mac->mac_base->NETWORK_CONTROL &= ~GEM_ENABLE_TRANSMIT;

    /* Allocate all 4 segments to queue 0 as this is our only one... */
    g_test_mac->mac_base->TX_Q_SEG_ALLOC_Q0TO3 = 2;

    g_test_mac->mac_base->NETWORK_CONTROL |= GEM_ENABLE_TRANSMIT;
    g_test_mac->mac_base->NETWORK_CONFIG |= GEM_JUMBO_FRAMES;

    /*
     * Register MAC interrupt handler listener functions. These functions will
     * be called  by the MAC driver when a packet has been sent or received.
     * These callback functions are intended to help managing transmit and
     * receive buffers by indicating when a transmit buffer can be released or
     * a receive buffer has been filled with an rx packet.
     */

    /*
     * Allocate receive buffers.
     *
     * We prime the pump with a full set of packet buffers and then re use them
     * as each packet is handled.
     *
     * This function will need to be called each time a packet is received to
     * hand back the receive buffer to the MAC driver.
     */
    for (count = 0; count < MSS_MAC_RX_RING_SIZE; ++count)
    {
        /*
         * We allocate the buffers with the Ethernet MAC interrupt disabled
         * until we get to the last one. For the last one we ensure the Ethernet
         * MAC interrupt is enabled on return from MSS_MAC_receive_pkt().
         */
#if defined(MSS_MAC_USE_DDR)
        if (count != (MSS_MAC_RX_RING_SIZE - 1))
        {
            MSS_MAC_receive_pkt(g_test_mac,
                                0,
                                g_mac_rx_buffer + count * MSS_MAC_MAX_RX_BUF_SIZE,
                                0,
                                0);
        }
        else
        {
            MSS_MAC_receive_pkt(g_test_mac,
                                0,
                                g_mac_rx_buffer + count * MSS_MAC_MAX_RX_BUF_SIZE,
                                0,
                                -1);
        }
#else
        if (count != (MSS_MAC_RX_RING_SIZE - 1))
        {
            MSS_MAC_receive_pkt(g_test_mac, 0, g_mac_rx_buffer[count], 0, 0);
        }
        else
        {
            MSS_MAC_receive_pkt(g_test_mac, 0, g_mac_rx_buffer[count], 0, -1);
        }
#endif
    }
}

