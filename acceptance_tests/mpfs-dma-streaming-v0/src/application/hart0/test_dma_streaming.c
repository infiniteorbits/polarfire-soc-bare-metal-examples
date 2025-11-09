///
/// File            | test_dma_streaming.c
/// Description     | Streaming from AXI4DMA -> DDR -> Ethernet
///
/// Author          | Trajce Nikolov    | trajce.nikolov.nick@gmail.com
///                                     | trajce.nikolov.nick@outlook.com
/// Date            | October 2025
///
/// Copyright 2025  | RFIM Space
///
///

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "mpfs_hal/mss_hal.h"
#include "mpfs_hal/common/nwc/mss_nwc_init.h"

#include "drivers/mss/mss_mmuart/mss_uart.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_registers.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_mac_sw_cfg.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_mac_regs.h"
#include "drivers/mss/mss_ethernet_mac/mss_ethernet_mac.h"

#include "inc/cond_var.h"
#include "inc/protocol.h"

#include "messaging.h"

extern volatile uint32_t*           DDR_data_ptr;
extern volatile uint64_t            DDR_data_addr;

extern mss_uart_instance_t*         g_uart;
extern mss_mac_instance_t*          g_test_mac;
extern mss_mac_cfg_t                g_mac_config;
struct cond_var_t                   g_cond_var_hart0;
struct cond_var_t                   g_cond_var_hart1;

uint16_t
htons(uint16_t hostshort)
{
    uint16_t result;
    uint8_t* host_ptr = (uint8_t*)&hostshort;
    uint8_t* result_ptr = (uint8_t*)&result;

    #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        result_ptr[0] = host_ptr[1];
        result_ptr[1] = host_ptr[0];
    #else
        result_ptr[0] = host_ptr[0];
        result_ptr[1] = host_ptr[1];
    #endif

    return result;
}

#define UART                g_uart
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

uint8_t
test_dma_streaming(void)
{
    test_display_start("DMA Streaming Test");

    struct eth_frame_t frame;

    /// Set up the destination MAC address
    ///
    unsigned char dst_mac[] = {0xCC, 0x96, 0xE5, 0x15, 0x03, 0x9C};
    memcpy(frame.dst_mac, dst_mac, 6);

    /// Set up the source MAC address
    ///
    memcpy(frame.src_mac, g_mac_config.mac_addr, 6);

    /// Set up the Ethertype
    ///
    frame.eth_type = htons(0x0800);
    struct packet_t packet;

    uint8_t     msg[128] = {0};
    uint16_t    frame_cnt = 0;

    volatile uint32_t* data = DDR_data_ptr;

    mss_mac_tx_desc_t tx_desc_tab[MSS_MAC_TX_RING_SIZE];
    g_test_mac->queue[0].nb_available_tx_desc = (uint32_t)MSS_MAC_TX_RING_SIZE;

    uint32_t chunks_size = CHUNK_SIZE;
    uint32_t packet_size = chunks_size;


    g_test_mac = &g_mac0;
    g_test_mac->mac_base->NETWORK_CONFIG |= GEM_JUMBO_FRAMES;

    uint32_t            num_chunks = RESOLUTION_X * RESOLUTION_Y * 2/ chunks_size;
    volatile uint8_t*   chunk_ptr = (uint8_t*)data;
    struct packet_t*    pckt = (struct packet_t* )frame.payload;

    uint16_t num_full_batches = num_chunks / (MSS_MAC_TX_RING_SIZE - 1);

    while (2U)
    {
        cond_var_wait(&g_cond_var_hart1);

#ifdef MSS_MAC_SPEED_TEST
        data = (uint32_t*)DDR_data_addr;
        uint16_t curr_chunk = 0;
        uint16_t* data_ptr = (uint16_t*)((uint64_t)(data));

        for (uint16_t b = 0; b < num_full_batches; ++b)
        {
            for (volatile uint16_t i = 0; i != (MSS_MAC_TX_RING_SIZE - 1); i++)
            {

                *data_ptr = curr_chunk++;
                /// *(data_ptr+1) = 0xb007u;

                tx_desc_tab[i].addr_low = (uint32_t)(((uint64_t)data_ptr) & 0xFFFFFFFFu);
                tx_desc_tab[i].addr_high = (uint32_t)((((uint64_t)data_ptr) >> 32)& 0xFFFFFFFFu);

                data_ptr += chunks_size /2;

                tx_desc_tab[i].status = (packet_size & GEM_TX_DMA_BUFF_LEN) | GEM_TX_DMA_LAST;
                tx_desc_tab[i + 1].status |= GEM_TX_DMA_WRAP | GEM_TX_DMA_USED;
            }

            g_test_mac->queue[0].nb_available_tx_desc = (uint32_t)MSS_MAC_TX_RING_SIZE;
            uint8_t tx_status = MSS_MAC_send_pkts_fast(g_test_mac,
                                    tx_desc_tab,
                                    (packet_size & 0xFFFF) *
                                    (MSS_MAC_TX_RING_SIZE - 1));

        }

        uint16_t num_remaining_chunks = num_chunks - curr_chunk;
        for (volatile uint16_t i = 0; i < num_remaining_chunks; i++)
        {
            *data_ptr = curr_chunk++;
            /// *(data_ptr+1) = 0xb007u;

            tx_desc_tab[i].addr_low = (uint32_t)(((uint64_t)data_ptr) & 0xFFFFFFFFu);
            tx_desc_tab[i].addr_high = (uint32_t)((((uint64_t)data_ptr) >> 32)& 0xFFFFFFFFu);

            data_ptr += chunks_size /2;

            /// Mark as last buffer for frame
            ///
            tx_desc_tab[i].status = (packet_size & GEM_TX_DMA_BUFF_LEN) | GEM_TX_DMA_LAST;
            tx_desc_tab[i + 1].status |= GEM_TX_DMA_WRAP | GEM_TX_DMA_USED;
        }
        g_test_mac->queue[0].nb_available_tx_desc = (uint32_t)MSS_MAC_TX_RING_SIZE;
        uint8_t tx_status = MSS_MAC_send_pkts_fast(g_test_mac,
                                tx_desc_tab,
                                (packet_size & 0xFFFF) *
                                num_remaining_chunks);

#else
        for (volatile uint32_t i = 0; i < num_chunks; ++i)
        {
            chunk_ptr = (void*)(DDR_data_addr + i*chunks_size);

            init_packet(pckt);
            set_packet_data(pckt, RESOLUTION_X, RESOLUTION_Y, (uint8_t*)chunk_ptr, chunks_size);

            pckt->chunk = i;
            pckt->num_chunks = num_chunks;

            uint8_t tx_status = MSS_MAC_send_pkt(g_test_mac,
                                 0,
                                 (uint8_t const * )&frame,
                                 sizeof(frame) | 0x0 /*CRC*/,
                                 (void *)0);

            sprintf((char*)msg," TX : %d, frame : %d, chunk : %d     \r", (uint16_t)tx_status, frame_cnt, i);
            PRINT_STRING(msg);

        }
#endif

        sprintf((char*)msg,"\n\r Frame sent : %d\n\r", frame_cnt++);
        PRINT_STRING(msg);

        cond_var_signal(&g_cond_var_hart0);


        /// if (++frame_cnt > 20) break;
    }

    test_display_finished();

    return -1;
}

