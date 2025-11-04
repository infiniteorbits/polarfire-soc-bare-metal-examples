/*******************************************************************************
 * Copyright 2019 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * code running on e51
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <float.h>

#include "mpfs_hal/mss_hal.h"
#include "mpfs_hal/common/nwc/mss_nwc_init.h"

#include "drivers/mss/mss_gpio/mss_gpio.h"
#include "drivers/mss/mss_mmuart/mss_uart.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_registers.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_mac_sw_cfg.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_mac_regs.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_mac.h"
#include "drivers/mss/mss_ethernet_mac/phy.h"
#include "drivers/mss/mss_gpio/mss_gpio.h"
#include "inc/common.h"

#include "messaging.h"
#include "inc/RFIM_Phy.h"

/// This one prints things on UART
///
#define DEMO_UART       &g_mss_uart0_lo
void PRINT_STRING(uint8_t* x)
{
    uint8_t message[512];
    sprintf(message," %s", (x));
    MSS_UART_polled_tx_string(DEMO_UART, (uint8_t *)&message[0]);
}

/*
 * Align these on an 8 byte boundary as we might be using IEEE 1588 time
 * stamping and that uses b2 of the buffer pointer to indicate that a timestamp
 * is present in this descriptor.
 */

#if defined(MSS_MAC_USE_DDR)
static uint8_t *g_mac_rx_buffer = 0;
#else
static uint8_t g_mac_rx_buffer[MSS_MAC_RX_RING_SIZE][MSS_MAC_MAX_RX_BUF_SIZE]
    __attribute__((aligned(16)));
#endif

mss_mac_cfg_t               g_mac_config;
mss_mac_instance_t *        g_mac = &g_mac1;
int32_t                     g_graceful_exit_cnt = 1000000;
static uint8_t              g_test_result = 0;
extern volatile uint64_t    g_tick_counter;

/*==============================================================================
 * Network configuration globals.
 */

typedef struct aligned_tx_buf
{
    uint64_t aligner;
    uint8_t packet[MSS_MAC_MAX_PACKET_SIZE];
} ALIGNED_TX_BUF;

ALIGNED_TX_BUF tx_packet;
uint8_t tx_packet_data[60] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x08, 0x06, 0x00,
    0x01, 0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0xEC, 0x08, 0x6B, 0xE2, 0xCA, 0x17, 0xC0, 0xA8,
    0x80, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xA8, 0x80, 0xFC, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t tx_pak_arp[128] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0x00, 0x12, 0x34, 0x56, 0x08, 0x06, 0x00, 0x01,
    0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0xFC, 0x00, 0x12, 0x34, 0x56, 0x0A, 0x02, 0x02, 0x02, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x02, 0x02, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static volatile uint64_t tx_count = 0;

volatile uint32_t g_crc = 0; /* CRC pass through control */
volatile int g_loopback = 0; /* Software loopback control */
volatile int g_phy_dump = 0; /* PHY Register dump control */
volatile int g_tx_add_1 = 0; /* Tx length adjustment control to make loopback
                                packets more visible... */
volatile int g_tx_adjust = 1; /* Adjustment to make to the packet length when
                                 enabled by 'n' */

const uint8_t *g_speed_strings[7] = {"Autonegotiate",
                                   "10M Half Duplex",
                                   "10M Full Duplex",
                                   "100M Half Duplex",
                                   "100M Full Duplex",
                                   "1000M Half Duplex",
                                   "1000M Full Duplex"};

/* Receive packet capture variables */

#define PACKET_IDLE  0
#define PACKET_ARMED 1
#define PACKET_DONE  2
#define PACKET_MAX   16384U

volatile int    g_capture = PACKET_IDLE;
uint8_t         g_packet_data[PACKET_MAX];

/* pair of buffers and flags etc for user mode loopback */

/* States used to track buffer availability */
#define PKT_STATE_FREE 0
#define PKT_STATE_RXED 1
#define PKT_STATE_SENT 2

volatile int        g_user_loopback = 0;
uint8_t             g_packet_data_0[PACKET_MAX];
uint8_t             g_packet_data_1[PACKET_MAX];
volatile uint32_t   g_packet_length_0 = 0;
volatile uint32_t   g_packet_length_1 = 0;
volatile int        g_packet_select = 0;
volatile int        g_packet_state_0 = PKT_STATE_FREE;
volatile int        g_packet_state_1 = PKT_STATE_FREE;
volatile int        g_tx_loop_count = 0;
volatile int        g_rx_dropped = 0;
volatile uint64_t   g_launch_0 = 0;
volatile uint64_t   g_launch_1 = 0;
volatile uint64_t   g_done_0 = 0;
volatile uint64_t   g_done_1 = 0;
volatile uint32_t   g_packet_length = 0;
volatile int        g_reload = PACKET_IDLE;
volatile uint64_t   g_tx_retry = 0;
volatile int        g_link_status = 0;

mss_mac_instance_t*  g_test_mac = &g_mac0;

static volatile uint64_t    g_rx_count = 0;


/******************************************************************************
 * Maximum receiver buffer size.
 *****************************************************************************/
#define MAX_RX_DATA_SIZE                        256

#define ENTER                                   13u
#define TIMEOUT                                 5000u


/**============================================================================
 *
 */

/// Attached to MAC instance as a callback and it is called
/// when packet sent is completed
///
static void
packet_tx_complete_handler(/* mss_mac_instance_t*/ void *this_mac,
                           uint32_t queue_no,
                           mss_mac_tx_desc_t *cdesc,
                           void *caller_info)
{
    (void)caller_info;
    (void)cdesc;
    (void)this_mac;
    (void)queue_no;

    tx_count++;
    if (g_user_loopback)
    {
        /*
         * Free up the currently active buffer and switch to alternate if it is
         * waiting for transmission completion too.
         */
        if (0 == g_packet_select)
        {
            g_done_0 = g_tick_counter;
            g_packet_state_0 = PKT_STATE_FREE;
            if (PKT_STATE_FREE != g_packet_state_1)
            {
                g_packet_select = 1;
            }
        }
        else
        {
            g_done_1 = g_tick_counter;
            g_packet_state_1 = PKT_STATE_FREE;
            if (PKT_STATE_FREE != g_packet_state_0)
            {
                g_packet_select = 0;
            }
        }

        g_tx_loop_count = 0;
    }
}


static void copyby8(uint64_t *dest, uint64_t *source, uint32_t count);
static void
copyby8(uint64_t *dest, uint64_t *source, uint32_t count)
{
    count = (count + 7) / 8;
    while (count--)
        *dest++ = *source++;
}

/**=============================================================================
    Bottom-half of receive packet handler
*/

/// Attached to g_mac instance and its been called when packet
/// kis received
///
static void
mac_rx_callback(
    /* mss_mac_instance_t */ void *this_mac,
    uint32_t queue_no,
    uint8_t *p_rx_packet,
    uint32_t pckt_length,
    mss_mac_rx_desc_t *cdesc,
    void *caller_info)
{
    int32_t tx_status;
    (void)caller_info;
    (void)cdesc;
    (void)queue_no;
    uint64_t retry_start;

    //if (this_mac == g_test_mac) return;

    if (g_user_loopback) /* Check first and ignore others for speed reasons... */
    {
        if (0 == g_packet_select)
        {
            if (PKT_STATE_FREE == g_packet_state_0)
            {
                copyby8((uint64_t *)g_packet_data_0, (uint64_t *)p_rx_packet, pckt_length);
                g_packet_state_0 = PKT_STATE_RXED;
                g_packet_length_0 = pckt_length;
            }
            else if (PKT_STATE_FREE == g_packet_state_1)
            {
                copyby8((uint64_t *)g_packet_data_1, (uint64_t *)p_rx_packet, pckt_length);
                g_packet_state_1 = PKT_STATE_RXED;
                g_packet_length_1 = pckt_length;
            }
            else
            {
                /*
                 * At this point, we have received two packets and not re-sent
                 * them. We disable these RX interrupts for now as we know that
                 * we cannot handle full line rate bursts of 64 byte frames
                 * without starving the rest of the system due to the interrupt
                 * rates involved...
                 */
                g_test_mac->mac_base->INT_DISABLE = GEM_RECEIVE_COMPLETE | GEM_RX_USED_BIT_READ;
                g_rx_dropped++;
            }
        }
        else
        {
            if (PKT_STATE_FREE == g_packet_state_1)
            {
                copyby8((uint64_t *)g_packet_data_1, (uint64_t *)p_rx_packet, pckt_length);
                g_packet_state_1 = PKT_STATE_RXED;
                g_packet_length_1 = pckt_length;
            }
            else if (PKT_STATE_FREE == g_packet_state_0)
            {
                copyby8((uint64_t *)g_packet_data_0, (uint64_t *)p_rx_packet, pckt_length);
                g_packet_state_0 = PKT_STATE_RXED;
                g_packet_length_0 = pckt_length;
            }
            else
            {
                /*
                 * At this point, we have received two packets and not re-sent
                 * them. We disable these RX interrupts for now as we know that
                 * we cannot handle full line rate bursts of 64 byte frames
                 * without starving the rest of the system due to the interrupt
                 * rates involved...
                 */
                g_test_mac->mac_base->INT_DISABLE = GEM_RECEIVE_COMPLETE | GEM_RX_USED_BIT_READ;
                g_rx_dropped++;
            }
        }
    }
    else
    {
        if (PACKET_ARMED == g_capture) /* Looking for packet so grab a copy */
        {
            if (pckt_length > PACKET_MAX)
            {
                pckt_length = PACKET_MAX;
            }

            memcpy(g_packet_data, p_rx_packet, pckt_length);

            g_packet_length = pckt_length;
            g_capture = PACKET_DONE; /* and say we go it */
        }

        if (g_loopback) /* Send what we receive if set to loopback */
        {
            /*
             * We send back any packets we receive (with an optional extra byte to
             * make the returned packets stand out in Wireshark).
             *
             * We may need to wait for the last packet to finish sending as we only
             * have a single transmit queue so 100% back to back operation will not
             * work but... The MSS MAC ISR checks to see if we already handled the
             * current TX so that we don't double dip on the TX handler.
             */

            retry_start = g_tx_retry;
            do
            {
                if (g_tx_add_1)
                {
                    tx_status = MSS_MAC_send_pkt(((mss_mac_instance_t *)this_mac),
                                                 0,
                                                 p_rx_packet,
                                                 (pckt_length + (uint32_t)g_tx_adjust) | g_crc,
                                                 (void *)0);
                }
                else
                {
                    tx_status = MSS_MAC_send_pkt(((mss_mac_instance_t *)this_mac),
                                                 0,
                                                 p_rx_packet,
                                                 pckt_length | g_crc,
                                                 (void *)0);
                }

                if (MSS_MAC_SUCCESS != tx_status)
                {
                    g_tx_retry++;
                    if (((mss_mac_instance_t *)this_mac)->mac_base->INT_STATUS &
                        GEM_TRANSMIT_COMPLETE)
                    {
#if 0 /* This is doing it by the book... */
                    if(0 != ((mss_mac_instance_t *)this_mac)->tx_complete_handler)
                    {
                        ((mss_mac_instance_t *)this_mac)->tx_complete_handler((mss_mac_instance_t *)this_mac,
                                ((mss_mac_instance_t *)this_mac)->tx_caller_info[0]);
                    }
#else /* This is cutting corners because we know we can in this instance... */
                        tx_count++;
#endif

                        ((mss_mac_instance_t *)this_mac)->queue[0].nb_available_tx_desc =
                            MSS_MAC_TX_RING_SIZE; /* Release transmit queue... */
                        ((mss_mac_instance_t *)this_mac)->mac_base->TRANSMIT_STATUS =
                            GEM_STAT_TRANSMIT_COMPLETE;
                        ((mss_mac_instance_t *)this_mac)->mac_base->INT_STATUS =
                            GEM_TRANSMIT_COMPLETE;
                    }
                }
            } while ((tx_status != MSS_MAC_SUCCESS) && ((g_tx_retry - retry_start) < 100));
        }
    }

#if defined(MSS_MAC_UNH_TEST)
    MSS_MAC_receive_pkt_isr((mss_mac_instance_t *)this_mac);
#else
    MSS_MAC_receive_pkt((mss_mac_instance_t *)this_mac, 0, p_rx_packet, 0, 1);
#endif
    g_rx_count++;

}

/**=============================================================================
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */

/// Init PHY and the g_mac instance queue buffers for receiveing data
/// Setup the g_mac networking
static void
low_level_init(void)
{
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
    g_mac_config.speed_duplex_select = MSS_MAC_ANEG_1000M_FD;
    g_mac_config.mac_addr[0] = 0x00;
    g_mac_config.mac_addr[1] = 0xFC;
    g_mac_config.mac_addr[2] = 0x00;
    g_mac_config.mac_addr[3] = 0x12;
    g_mac_config.mac_addr[4] = 0x34;
    g_mac_config.interface_type = GMII;
    g_mac_config.tsu_clock_select = 0U;
    g_mac_config.speed_mode = MSS_MAC_1000_FDX;
    g_mac_config.phy_type = MSS_MAC_DEV_PHY_VSC8541;

#define RFIM_PHY_MSS_COMPATIBLE
#ifdef RFIM_PHY_MSS_COMPATIBLE
    g_mac_config.phy_init = MSS_MAC_RFIM_phy_init;
    g_mac_config.phy_autonegotiate = MSS_MAC_RFIM_phy_autonegotiate;
    g_mac_config.phy_get_link_status = MSS_MAC_RFIM_phy_get_link_status;
    g_mac_config.phy_set_link_speed = MSS_MAC_RFIM_phy_set_link_speed;
    g_mac_config.phy_mac_autonegotiate = MSS_MAC_RFIM_phy_mac_autonegotiate;

    /// Mandatory
    ///
    RFIM_init();
#endif

    /// Make sure this one is correct, defined in
    /// drivers/mss/mss_ethernet_mac/mss_ethernet_mac.c
    ///
    /// ASSERT( MSS_MAC0_BASE == MAC0_BASE )
    /// ASSERT( MSS_MAC1_BASE == MAC1_BASE )

    /// Mac 0
    ///
    g_mac_config.phy_addr = PHY0_ADDR;
    g_mac_config.mac_addr[5] = 0x56;
    MSS_MAC_init(&g_mac0, &g_mac_config);

    /// Mac 1
    ///
    g_mac_config.phy_addr = PHY1_ADDR;
    g_mac_config.mac_addr[5] = 0x57;
    MSS_MAC_init(&g_mac1, &g_mac_config);

    g_test_mac = &g_mac0;

    sleep_ms(100);

#ifdef RFIM_PHY_MSS_COMPATIBLE
    PRINT_STRING("\n\r");

    RFIM_MAC0_phy_dump_id(DEMO_UART);
    RFIM_MAC1_phy_dump_id(DEMO_UART);

    PRINT_STRING("\n\r");
    PRINT_STRING("Default link state:\n\r");

    RFIM_MAC0_phy_dump_link_status(DEMO_UART);
    RFIM_MAC1_phy_dump_link_status(DEMO_UART);

    PRINT_STRING("\n\r");

    PRINT_STRING("\n\r");
    PRINT_STRING("Switching phy leds on\n\r");

    RFIM_MAC0_phy_leds_onoff(PHY_LEDS_ON);
    RFIM_MAC1_phy_leds_onoff(PHY_LEDS_ON);


#else

    RFIM_init();

    RFIM_MAC0_phy_init();
    RFIM_MAC1_phy_init();

    PRINT_STRING("\n\r");

    RFIM_phy_loopback_autonegotiate();

    RFIM_MAC0_phy_dump_id(DEMO_UART);
    RFIM_MAC1_phy_dump_id(DEMO_UART);

    PRINT_STRING("\n\r");
    PRINT_STRING("Default link state:\n\r");

    RFIM_MAC0_phy_dump_link_status(DEMO_UART);
    RFIM_MAC1_phy_dump_link_status(DEMO_UART);

    PRINT_STRING("\n\r");
    PRINT_STRING("Setting speed to 100 Mbps Full Duplex\n\r");

    RFIM_MAC0_phy_set_speed(PHY_MAC100MBPS, PHY_FULL_DUPLEX);
    RFIM_MAC1_phy_set_speed(PHY_MAC100MBPS, PHY_FULL_DUPLEX);

    PRINT_STRING("\n\r");

    RFIM_MAC0_phy_dump_link_status(DEMO_UART);
    RFIM_MAC1_phy_dump_link_status(DEMO_UART);

    PRINT_STRING("\n\r");

    PRINT_STRING("\n\r");
    PRINT_STRING("Switching phy leds on\n\r");

    RFIM_MAC0_phy_leds_onoff(PHY_LEDS_ON);
    RFIM_MAC1_phy_leds_onoff(PHY_LEDS_ON);
#endif

    /// And all works as charm
    ///

    /*
     * Not doing the tx disable/enable sequence here around the queue allocation
     * adjustment results in tx_go being set which causes the new tx code to
     * hang waiting for the last tx to complete...
     */
    g_test_mac->mac_base->NETWORK_CONTROL &= ~GEM_ENABLE_TRANSMIT;

    /* Allocate all 4 segments to queue 0 as this is our only one... */
    g_test_mac->mac_base->TX_Q_SEG_ALLOC_Q0TO3 = 2;

    g_test_mac->mac_base->NETWORK_CONTROL |= GEM_ENABLE_TRANSMIT;
    /// g_test_mac->mac_base->NETWORK_CONFIG |= GEM_JUMBO_FRAMES;
    /*
     * Register MAC interrupt handler listener functions. These functions will
     * be called  by the MAC driver when a packet has been sent or received.
     * These callback functions are intended to help managing transmit and
     * receive buffers by indicating when a transmit buffer can be released or
     * a receive buffer has been filled with an rx packet.
     */

    MSS_MAC_set_tx_callback(g_test_mac, 0, packet_tx_complete_handler);
    MSS_MAC_set_rx_callback(g_test_mac, 0, mac_rx_callback);

  //  MSS_MAC_set_tx_callback(&g_mac1, 0, packet_tx_complete_handler);
 //  MSS_MAC_set_rx_callback(&g_mac1, 0, mac_rx_callback);

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

    delay((DELAY_CYCLES_100MS * 20));

    g_test_mac = &g_mac1;

    g_test_mac->mac_base->NETWORK_CONTROL &= ~GEM_ENABLE_TRANSMIT;

    /* Allocate all 4 segments to queue 0 as this is our only one... */
    g_test_mac->mac_base->TX_Q_SEG_ALLOC_Q0TO3 = 2;

    g_test_mac->mac_base->NETWORK_CONTROL |= GEM_ENABLE_TRANSMIT;
    //// g_test_mac->mac_base->NETWORK_CONFIG |= GEM_JUMBO_FRAMES;
    /*
     * Register MAC interrupt handler listener functions. These functions will
     * be called  by the MAC driver when a packet has been sent or received.
     * These callback functions are intended to help managing transmit and
     * receive buffers by indicating when a transmit buffer can be released or
     * a receive buffer has been filled with an rx packet.
     */

    MSS_MAC_set_tx_callback(g_test_mac, 0, packet_tx_complete_handler);
    MSS_MAC_set_rx_callback(g_test_mac, 0, mac_rx_callback);

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

    delay((DELAY_CYCLES_100MS * 20));
    g_test_mac = &g_mac0;
}

/*==============================================================================
 *
 */

/// The test main call
///
uint8_t test_mac(void);
uint8_t
test_mac(void)
{

    test_display_start("MAC Ethernet Test");
    mac_task(0);
    test_display_finished();

    return g_test_result;
}


static uint32_t stats[2][MSS_MAC_LAST_STAT];
static uint64_t phy0_copper_rx_good = 0; /* P1 reg 18 */
static uint64_t phy1_copper_rx_good = 0; /* P1 reg 18 */
static uint64_t phy0_rx_err = 0; /* P0 reg 19 */
static uint64_t phy1_rx_err = 0; /* P0 reg 19 */
static uint64_t phy0_false_carrier = 0; /* P0 reg 20 */
static uint64_t phy1_false_carrier = 0; /* P0 reg 20 */
static uint64_t phy0_link_disconnect = 0; /* P0 reg 21 */
static uint64_t phy1_link_disconnect = 0; /* P0 reg 21 */
static uint64_t mac_rx_good = 0; /* P3 reg 28 */
static uint64_t mac_rx_err = 0; /* P3 reg 29 */
static uint64_t mac_tx_good = 0; /* P3 reg 15 */
static uint64_t mac_tx_err = 0; /* P3 reg 16 */


/*==============================================================================
 *
 */

/// Dumps gem statistic
///
void
stats_gem(mss_mac_instance_t* mac_instance)
{
    uint8_t info_string[512];
    uint16_t gem = mac_instance == &g_mac0 ? 0 : 1;

    /* Grab the stats up front to minimise skew */
    for (uint32_t count = MSS_MAC_TX_OCTETS_LOW; count != MSS_MAC_LAST_STAT; count++)
    {
        stats[gem][count] += MSS_MAC_read_stat(mac_instance, count);
    }

    sprintf(info_string, "GEM %d Statistics\n\r", gem);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_OCTETS_LOW           % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_TX_OCTETS_LOW]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_OCTETS_HIGH          % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_TX_OCTETS_HIGH]);
    PRINT_STRING(info_string);

    sprintf(info_string, "TX_FRAMES_OK            % 10lu  ", (uint64_t)stats[gem][MSS_MAC_TX_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_BCAST_FRAMES_OK      % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_TX_BCAST_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_MCAST_FRAMES_OK      % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_TX_MCAST_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_PAUSE_FRAMES_OK      % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_TX_PAUSE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_64_BYTE_FRAMES_OK    % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_TX_64_BYTE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_65_BYTE_FRAMES_OK    % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_TX_65_BYTE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_128_BYTE_FRAMES_OK   % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_TX_128_BYTE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_256_BYTE_FRAMES_OK   % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_TX_256_BYTE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_512_BYTE_FRAMES_OK   % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_TX_512_BYTE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_1024_BYTE_FRAMES_OK  % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_TX_1024_BYTE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_1519_BYTE_FRAMES_OK  % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_TX_1519_BYTE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_UNDERRUNS            % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_TX_UNDERRUNS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_SINGLE_COLLISIONS    % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_TX_SINGLE_COLLISIONS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_MULTIPLE_COLLISIONS  % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_TX_MULTIPLE_COLLISIONS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_EXCESSIVE_COLLISIONS % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_TX_EXCESSIVE_COLLISIONS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_LATE_COLLISIONS      % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_TX_LATE_COLLISIONS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_DEFERRED_FRAMES      % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_TX_DEFERRED_FRAMES]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "TX_CRS_ERRORS           % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_TX_CRS_ERRORS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_OCTETS_LOW           % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_RX_OCTETS_LOW]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_OCTETS_HIGH          % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_RX_OCTETS_HIGH]);
    PRINT_STRING(info_string);

    sprintf(info_string, "RX_FRAMES_OK            % 10lu  ", (uint64_t)stats[gem][MSS_MAC_RX_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_BCAST_FRAMES_OK      % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_RX_BCAST_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_MCAST_FRAMES_OK      % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_RX_MCAST_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_PAUSE_FRAMES_OK      % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_RX_PAUSE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_64_BYTE_FRAMES_OK    % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_RX_64_BYTE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_65_BYTE_FRAMES_OK    % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_RX_65_BYTE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_128_BYTE_FRAMES_OK   % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_RX_128_BYTE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_256_BYTE_FRAMES_OK   % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_RX_256_BYTE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_512_BYTE_FRAMES_OK   % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_RX_512_BYTE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_1024_BYTE_FRAMES_OK  % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_RX_1024_BYTE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_1519_BYTE_FRAMES_OK  % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_RX_1519_BYTE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_UNDERSIZE_FRAMES_OK  % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_RX_UNDERSIZE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_OVERSIZE_FRAMES_OK   % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_RX_OVERSIZE_FRAMES_OK]);
    PRINT_STRING(info_string);

    sprintf(info_string, "RX_JABBERS              % 10lu\n\r", (uint64_t)stats[gem][MSS_MAC_RX_JABBERS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_FCS_ERRORS           % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_RX_FCS_ERRORS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_LENGTH_ERRORS        % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_RX_LENGTH_ERRORS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_SYMBOL_ERRORS        % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_RX_SYMBOL_ERRORS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_ALIGNMENT_ERRORS     % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_RX_ALIGNMENT_ERRORS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_RESOURCE_ERRORS      % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_RX_RESOURCE_ERRORS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_OVERRUNS             % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_RX_OVERRUNS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_IP_CHECKSUM_ERRORS   % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_RX_IP_CHECKSUM_ERRORS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_TCP_CHECKSUM_ERRORS  % 10lu\n\r",
            (uint64_t)stats[gem][MSS_MAC_RX_TCP_CHECKSUM_ERRORS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_UDP_CHECKSUM_ERRORS  % 10lu  ",
            (uint64_t)stats[gem][MSS_MAC_RX_UDP_CHECKSUM_ERRORS]);
    PRINT_STRING(info_string);

    sprintf(info_string,
            "RX_AUTO_FLUSHED_PACKETS % 10lu\n\r\n\r",
            (uint64_t)stats[gem][MSS_MAC_RX_AUTO_FLUSHED_PACKETS]);
    PRINT_STRING(info_string);
    PRINT_STRING("\n\r");

}

/// Dumps PHY and GEM statistic
///
void
stats_dump(void)
{
    char info_string[200];
    mss_mac_stat_t count;

    if (g_phy_dump) /* Only do if enabled as it can impact comms response times */
    {
        RFIM_phy_dump_regs();

        /// Read more statistics
        ///
        phy0_rx_err += Phy0_reg_0[19];
        phy0_false_carrier += Phy0_reg_0[20];
        phy0_link_disconnect += Phy0_reg_0[21];

        phy1_rx_err += Phy1_reg_0[19];
        phy1_false_carrier += Phy1_reg_0[20];
        phy1_link_disconnect += Phy1_reg_0[21];


        /// This is in the page 1 registry
        ///
        phy0_copper_rx_good += Phy0_reg_1[2] & 0x7FFF;
        phy1_copper_rx_good += Phy1_reg_1[2] & 0x7FFF;

        PRINT_STRING("\n\r");

        sprintf(info_string, "PHY 0 Statistics\n\r");
        PRINT_STRING(info_string);

        sprintf(info_string, "PHY CU RX GOOD          % 10lu  ", phy0_copper_rx_good);
        PRINT_STRING(info_string);

        sprintf(info_string, "PHY CU RX ERRORS        % 10lu\n\r", phy0_rx_err);
        PRINT_STRING(info_string);

        sprintf(info_string, "PHY FALSE CARRIER ERR   % 10lu  ", phy0_false_carrier);
        PRINT_STRING(info_string);

        sprintf(info_string, "PHY LINK DISCONNECTS    % 10lu\n\r\n\r", phy0_link_disconnect);
        PRINT_STRING(info_string);

        sprintf(info_string, "PHY 1 Statistics\n\r");
        PRINT_STRING(info_string);

        sprintf(info_string, "PHY CU RX GOOD          % 10lu  ", phy1_copper_rx_good);
        PRINT_STRING(info_string);

        sprintf(info_string, "PHY CU RX ERRORS        % 10lu\n\r", phy1_rx_err);
        PRINT_STRING(info_string);

        sprintf(info_string, "PHY FALSE CARRIER ERR   % 10lu  ", phy1_false_carrier);
        PRINT_STRING(info_string);

        sprintf(info_string, "PHY LINK DISCONNECTS    % 10lu\n\r\n\r", phy1_link_disconnect);
        PRINT_STRING(info_string);

        PRINT_STRING("\n\r\n\r");
    }

    stats_gem(&g_mac0);
    stats_gem(&g_mac1);
 }

/*==============================================================================
 *
 */

/// Dumps the received packet
///
void
packet_dump(void)
{
    char info_string[200];
    int dump_address = 0;
    int count;
    char temp_string[10];

    g_capture = PACKET_IDLE;

    uint8_t match = 0;
    switch (g_packet_length)
    {
    case 128:
        match = memcmp(g_packet_data, tx_pak_arp, 128) == 0 ? 1 : 0;
        break;
    case 60:
        match = memcmp(g_packet_data, tx_packet_data, 60) == 0 ? 1 : 0;
        break;
    }
    sprintf(info_string, "Packet match: %s\n\r", (match == 1 ? "successful" : "failed"));
    PRINT_STRING(info_string);

    if (g_test_result == 0)
    {
        if (match == 0)
            g_test_result = -1;
    }

    sprintf(info_string, "%d byte packet captured\n\r", g_packet_length);
    PRINT_STRING(info_string);

    while ((g_packet_length - (uint32_t)dump_address) >= 16)
    {
        sprintf(info_string, "%04X ", dump_address);
        for (count = 0; count < 16; count++)
        {
            sprintf(temp_string, "%02X ", (int)g_packet_data[dump_address + count] & 255);
            strcat(info_string, temp_string);
        }

        for (count = 0; count < 16; count++)
        {
            if ((g_packet_data[dump_address + count] >= 32) &&
                (g_packet_data[dump_address + count] < 127))
            {
                strncat(info_string, (char *)&g_packet_data[dump_address + count], 1);
            }
            else
            {
                strcat(info_string, ".");
            }
        }

        strcat(info_string, "\n\r");
        PRINT_STRING(info_string);

        dump_address += 16;
    }

    if ((g_packet_length - (uint32_t)dump_address) > 0) /* Finish off partial end line */
    {
        sprintf(info_string, "%04X ", dump_address);
        for (count = 0; count < ((int)g_packet_length - dump_address); count++)
        {
            sprintf(temp_string, "%02X ", (int)g_packet_data[dump_address + count] & 255);
            strcat(info_string, temp_string);
        }

        strncat(info_string,
                "                                                ",
                (size_t)((16 - count) * 3)); /* Crude but effective space padding... */
        for (count = 0; count < ((int)g_packet_length - dump_address); count++)
        {
            if ((g_packet_data[dump_address + count] >= 32) &&
                (g_packet_data[dump_address + count] < 127))
            {
                strncat(info_string, (char *)&g_packet_data[dump_address + count], 1);
            }
            else
            {
                strcat(info_string, ".");
            }
        }
        strcat(info_string, "\n\r");
        PRINT_STRING(info_string);
    }

    /* Finally see if we need to reload the capture mechanism... */
    if (g_reload)
    {
        g_capture = g_reload;
    }
}

/*==============================================================================
 *
 */

/// Displays all the clocks info
///
void
display_clocks(void)
{
    char info_string[200];
    uint32_t count0;
    cfmChannelMode chMode;

    memset(&chMode, 0, sizeof(chMode));

    PRINT_STRING("\n\r System Clock Frequencies:\n\r");

    MSS_CLF_clk_configuration(0x1,
                              1,
                              0,
                              0x0,
                              0x1); /* group 1, ref clkis clk in rerf here it is 125 Mhz */
    CFM_REG->clkselReg &= ~(0x1U << CFM_CLK_MONSEL_SHIFT);
    MSS_CFM_runtime_register(0x2710);
    chMode.channel0 = 1;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_0, &count0))
        ;

    sprintf(info_string, "CPU clock         = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    MSS_CFM_runtime_register(10000U);
    chMode.channel0 = 0;
    chMode.channel1 = 1;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_1, &count0))
        ;
    sprintf(info_string, "AXI clock         = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    chMode.channel1 = 0;
    chMode.channel2 = 1;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_2, &count0))
        ;
    sprintf(info_string, "AHB clock         = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    chMode.channel2 = 0;
    chMode.channel3 = 1;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_3, &count0))
        ;
    sprintf(info_string, "Reference clock   = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    chMode.channel3 = 0;
    chMode.channel4 = 1;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_4, &count0))
        ;
    sprintf(info_string, "DFI clock         = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    MSS_CLF_clk_configuration(0x2,
                              1,
                              0,
                              0x0,
                              0x1); /* group 2, ref clkis clk in rerf here it is 125 Mhz */
    chMode.channel0 = 1;
    chMode.channel4 = 0;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_0, &count0))
        ;
    sprintf(info_string, "MAC TX clock      = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    chMode.channel0 = 0;
    chMode.channel1 = 1;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_1, &count0))
        ;
    sprintf(info_string, "MAC TSU clock     = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    chMode.channel1 = 0;
    chMode.channel2 = 1;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_2, &count0))
        ;
    sprintf(info_string, "MAC0 RX clock     = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    chMode.channel2 = 0;
    chMode.channel3 = 1;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_3, &count0))
        ;
    sprintf(info_string, "MAC1 RX clock     = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    chMode.channel3 = 0;
    chMode.channel4 = 1;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_4, &count0))
        ;
    sprintf(info_string, "SGMII CLK C OUT   = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    MSS_CLF_clk_configuration(0x4,
                              1,
                              0,
                              0x0,
                              0x1); /* group 4, ref clkis clk in rerf here it is 125 Mhz */
    chMode.channel0 = 1;
    chMode.channel4 = 0;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_0, &count0))
        ;
    sprintf(info_string, "SGMII PLL clock 0 = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    chMode.channel0 = 0;
    chMode.channel1 = 1;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_1, &count0))
        ;
    sprintf(info_string, "SGMII PLL clock 1 = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    chMode.channel1 = 0;
    chMode.channel2 = 1;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_2, &count0))
        ;
    sprintf(info_string, "SGMII PLL clock 2 = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    chMode.channel2 = 0;
    chMode.channel3 = 1;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_3, &count0))
        ;
    sprintf(info_string, "SGMII PLL clock 3 = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    chMode.channel3 = 0;
    chMode.channel4 = 1;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_4, &count0))
        ;
    sprintf(info_string, "SGMII DLL clock   = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    MSS_CLF_clk_configuration(0x5,
                              1,
                              0,
                              0x0,
                              0x1); /* group 5, ref clkis clk in rerf here it is 125 Mhz */
    chMode.channel2 = 1;
    chMode.channel4 = 0;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_2, &count0))
        ;
    sprintf(info_string, "FAB MAC0 TSU clk  = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);

    chMode.channel2 = 0;
    chMode.channel3 = 1;
    MSS_CFM_channel_mode(chMode);
    MSS_CFM_control_start();
    while (ERROR_INVALID_CFM_BUSY == MSS_CFM_get_count(CFM_COUNT_3, &count0))
        ;
    sprintf(info_string, "FAB MAC1 TSU clk  = %uHz\n\r", count0 * 12500U);
    PRINT_STRING(info_string);
    PRINT_STRING("\n\r");
}

/*==============================================================================
 *
 *
 */


#define ATHENA_CR          ((volatile uint32_t *)(0x20127000u))
#define ATHENA_CR_CSRMERRS ((volatile uint32_t *)(0x2200600C))

/// The acual loop of the tests. Inherited from the repo code
/// where the user supposed to press keyboard keys as commands
/// - refer to README.md for the full list of the commands or
/// go thru the following code. The code has been changed instead of
/// getting the input in the UART from a user, a commands string
/// is defined which contains characters giving ability to run it
/// withouit user interaction .. for example, commands = "xs" will execute
/// 'x' and 's' keypress from the initial code. The loop is being broken
/// after 4 packets sent & receival or timeout. All the commands from the
/// initial code are still present for studying and modifying extending the
/// actual MAC Loopback Test
///
void
mac_task(void *pvParameters)
{
    static uint32_t add_on = 0;
    char            info_string[200];
    uint8_t         rx_buff[1];
    size_t          rx_size = 0;
    volatile uint64_t   delay_count;
    volatile uint32_t   gpio_inputs;

    (void)pvParameters;


/******************************************************************************/

     PLIC_init();
    __disable_local_irq((int8_t)MMUART0_E51_INT);

    PLIC_EnableIRQ(PLIC_MAC0_INT_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_QUEUE1_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_QUEUE2_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_QUEUE3_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_EMAC_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_MMSL_INT_OFFSET);

    MSS_UART_init(DEMO_UART,
                  MSS_UART_115200_BAUD,
                  MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

    PRINT_STRING("\n\n\r PolarFire MSS Ethernet MAC Test program\n\r");

    __enable_irq();

    low_level_init();

    display_clocks();

    uint8_t     commands[] = "xskctTtTs";
    uint32_t    commandCnt = 0;
    uint8_t     packetsCapturedCnt = 0;

    while (1)
    {
        if (PACKET_DONE == g_capture)
        {
            packet_dump();

            packetsCapturedCnt++;
        }

        /// Here we fake user input by popping character commands form string
        ///
        rx_size = MSS_UART_get_rx(DEMO_UART, rx_buff, (uint8_t)sizeof(rx_buff));
        if (commandCnt < strlen(commands))
        {
            rx_size = 1;
            rx_buff[0] = commands[commandCnt++];
        }
        else
            /// 4 packets received, test sucessfull, exit
            if (packetsCapturedCnt == 4)
            {
                PRINT_STRING("MAC Ethernet Loopback Test was successful");
                break;
            }
            else
            /// Graceful exit after timeout
            if (g_graceful_exit_cnt-- < 0)
            {
                PRINT_STRING("MAC Ethernet Loopback Test failed");
                g_test_result = -1;
                break;
            }

        /// Processing of key commands
        ///
        if (rx_size > 0)
        {
            if (rx_buff[0] == 'c')
            {
                PRINT_STRING("Packet capture enabled\n\r");
                g_capture = PACKET_ARMED;
                if (g_reload)
                {
                    g_reload = PACKET_ARMED;
                }
            }
            else if (rx_buff[0] == 'k')
            {
                if (g_reload != PACKET_IDLE)
                {
                    g_reload = PACKET_IDLE;
                }
                else
                {
                    g_reload = PACKET_ARMED;
                }

                sprintf(info_string, "Capture reload is %s\n\r", g_reload ? "enabled" : "disabled");
                PRINT_STRING(info_string);
            }
            else if (rx_buff[0] == 'p')
            {
                if (0 == (g_test_mac->mac_base->NETWORK_CONFIG & GEM_COPY_ALL_FRAMES))
                {
                    g_test_mac->mac_base->NETWORK_CONFIG |= GEM_COPY_ALL_FRAMES;
                    PRINT_STRING("Promiscuous mode enabled\n\r");
                }
                else
                {
                    g_test_mac->mac_base->NETWORK_CONFIG &= (uint32_t)(~GEM_COPY_ALL_FRAMES);
                    PRINT_STRING("Promiscuous mode disabled\n\r");
                }
            }
            else if (rx_buff[0] == 's')
            {
                PRINT_STRING("\n\r");

                RFIM_MAC0_phy_dump_link_status(DEMO_UART);

                sprintf(
                    info_string,
                    "RX %lu (%lu pkts), TX %lu (%lu pkts)\n\r RX Over Flow %lu, TX Retries %lu\n\r",
                    g_mac0.queue[0].ingress,
                    g_rx_count,
                    g_mac0.queue[0].egress,
                    tx_count,
                    g_mac0.queue[0].rx_overflow,
                    g_tx_retry);
                PRINT_STRING(info_string);
                sprintf(info_string,
                        "TX Pause %lu, RX Pause %lu, Pause Elapsed %lu\n\r",
                        g_mac0.tx_pause,
                        g_mac0.rx_pause,
                        g_mac0.pause_elapsed);
                PRINT_STRING(info_string);

                sprintf(info_string,
                        "HRESP not ok %lu RX Restarts %lu\n\r\n\r",
                        g_mac0.queue[0].hresp_error,
                        g_mac0.queue[0].rx_restart);
                PRINT_STRING(info_string);
                PRINT_STRING("\n\r");

                RFIM_MAC1_phy_dump_link_status(DEMO_UART);

                sprintf(
                        info_string,
                        "RX %lu (%lu pkts), TX %lu (%lu pkts)\n\r RX Over Flow %lu, TX Retries %lu\n\r",
                        g_mac1.queue[0].ingress,
                        g_rx_count,
                        g_mac1.queue[0].egress,
                        tx_count,
                        g_mac1.queue[0].rx_overflow,
                        g_tx_retry);
                    PRINT_STRING(info_string);
                    sprintf(info_string,
                            "TX Pause %lu, RX Pause %lu, Pause Elapsed %lu\n\r",
                            g_mac1.tx_pause,
                            g_mac1.rx_pause,
                            g_mac1.pause_elapsed);
                    PRINT_STRING(info_string);

                    sprintf(info_string,
                            "HRESP not ok %lu RX Restarts %lu\n\r\n\r",
                            g_mac1.queue[0].hresp_error,
                            g_mac1.queue[0].rx_restart);
                    PRINT_STRING(info_string);
                    PRINT_STRING("\n\r");


                stats_dump();
            }
            else if (rx_buff[0] == 't')
            {
                PRINT_STRING("\n\r");
                sprintf(info_string, "Sending to %s, Recieving on %s\n\r", (g_mac == &g_mac1 ? "MAC1" : "MAC0"), (g_mac == &g_mac1 ? "MAC0" : "MAC1"));
                PRINT_STRING(info_string);

                int32_t tx_status;

                add_on = 0; /* Reset the count for  command */
                memcpy(&tx_pak_arp[6], g_test_mac->mac_addr, 6);
                tx_status = MSS_MAC_send_pkt(g_mac,//g_test_mac,
                                             0,
                                             tx_pak_arp,
                                             sizeof(tx_pak_arp) | g_crc,
                                             (void *)0);

                sprintf(info_string, "TX status %d\n\r", (int)tx_status);
                PRINT_STRING(info_string);

                delay((DELAY_CYCLES_100MS* 5));
            }
            else if (rx_buff[0] == 'T')
            {
                PRINT_STRING("\n\r");
                sprintf(info_string, "Sending to %s, Recieving on %s\n\r", (g_mac == &g_mac1 ? "MAC1" : "MAC0"), (g_mac == &g_mac1 ? "MAC0" : "MAC1"));
                PRINT_STRING(info_string);

                int32_t tx_status;
                volatile uint32_t *ctrl_reg;

                memcpy(&tx_pak_arp[6], g_test_mac->mac_addr, 6);
                tx_status = MSS_MAC_send_pkt(g_mac,//g_test_mac,
                                             0,
                                             tx_packet_data,
                                             ((60 + add_on) | g_crc),
                                             (void *)0);


                sprintf(info_string,
                        "TX status %d, size %d\n\r",
                        (int)tx_status,
                        (int)(add_on + 60));
                PRINT_STRING(info_string);
                delay((DELAY_CYCLES_100MS* 5));
                ctrl_reg = &g_test_mac->mac_base->NETWORK_CONFIG;
                /* Coarse adjust for jumbo frame mode */
                if (0 != (*ctrl_reg & GEM_JUMBO_FRAMES))
                {
                    add_on += 100;
                    if (add_on > 10440) /* Allow a little extra for testing 10K upper limit */
                    {
                        add_on = 0;
                    }
                }
                else
                {
                    add_on++;
                    if (add_on > 68)
                    {
                        add_on = 0;
                    }
                }
                g_test_mac = &g_mac1;
                g_mac = &g_mac0;
            }
            else if (rx_buff[0] == 'x')
            {
                g_phy_dump = !g_phy_dump;
                if (g_phy_dump)
                {
                    PRINT_STRING("Phy register dump enabled\n\r");
                }
                else
                {
                    PRINT_STRING("Phy register dump disabled\n\r");
                }
            }
        }
    }

    PRINT_STRING("\n\r");
    PRINT_STRING("Switching phy leds off\n\r");

    RFIM_MAC0_phy_leds_onoff(PHY_LEDS_OFF);
    RFIM_MAC1_phy_leds_onoff(PHY_LEDS_OFF);
}
