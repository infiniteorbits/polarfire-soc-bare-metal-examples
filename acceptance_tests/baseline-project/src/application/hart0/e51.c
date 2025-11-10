/*******************************************************************************
 * Copyright 2019-2021 Microchip FPGA Embedded Systems Solution.
 *
 * SPDX-License-Identifier: MIT
 *
 * Application code running on E51.
 *
 * Please refer to README.md file for more details
 */

#include <stdio.h>
#include <string.h>
#include "mpfs_hal/mss_hal.h"
#include "inc/common.h"

mss_uart_instance_t *g_uart= &g_mss_uart0_lo;


volatile uint32_t count_sw_ints_h0 = 0U;

/* Main function for the hart0(E51 processor).
 * Application code running on hart1 is placed here
 *
 * The hart1 is in WFI while booting, hart0 brings it out of WFI when it raises
 * the first Software interrupt.
 */
void e51(void)
{
    char info_string[100];
    volatile uint32_t icount = 0U;
    volatile uint32_t stepcount = 0U;
    uint64_t hartid = read_csr(mhartid);
    uint32_t pattern_offset = 12U;
    HLS_DATA* hls = (HLS_DATA*)(uintptr_t)get_tp_reg();
    HART_SHARED_DATA * hart_share = (HART_SHARED_DATA *)hls->shared_mem;

    /* Clear pending software interrupt in case there was any. */
    clear_soft_interrupt();
    set_csr(mie, MIP_MSIP);

    (void)mss_config_clk_rst(MSS_PERIPH_MMUART0, (uint8_t) MPFS_HAL_FIRST_HART, PERIPHERAL_ON);

    MSS_UART_init( &g_mss_uart0_lo,
            MSS_UART_115200_BAUD,
            MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

    sprintf(info_string, "\r\nHart %u, HLS mem address 0x%lx, Shared mem 0x%lx",\
                                                              hls->my_hart_id, (uint64_t)hls, (uint64_t)hls->shared_mem);
    spinlock(&hart_share->mutex_uart0);
    MSS_UART_polled_tx(g_uart, (const uint8_t*)info_string,(uint32_t)strlen(info_string));
    spinunlock(&hart_share->mutex_uart0);

    /* Raise software interrupt to wake harts */
    raise_soft_interrupt(1U);
    raise_soft_interrupt(2U);
    raise_soft_interrupt(3U);
    raise_soft_interrupt(4U);

    __enable_irq();

    while (1U)
    {
        icount++;
        if (0x1000000U == icount)
        {
            icount = 0U;
            stepcount++;
            sprintf(info_string,"Hart %d, step %d\r\n", hartid, stepcount);
            spinlock(&hart_share->mutex_uart0);
            MSS_UART_polled_tx(g_uart, info_string, strlen(info_string));
            spinunlock(&hart_share->mutex_uart0);
        }
    }
    /* never return */
}

/* hart0 Software interrupt handler */
void Software_h0_IRQHandler(void)
{
    uint64_t hart_id = read_csr(mhartid);
    count_sw_ints_h0++;
}
