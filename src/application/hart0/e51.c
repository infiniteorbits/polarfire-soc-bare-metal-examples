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

#include <drivers/mss/mss_mmuart/mss_uart.h>

#include <mpfs_hal/mss_hal.h>
#include <mpfs_hal/common/mss_clint.h>
#include <mpfs_hal/common/nwc/mss_nwc_init.h>
#include <mpfs_hal/mpfs_hal_version.h>

#include "tests.h"
#include "messaging.h"

mss_uart_instance_t*    g_uart= &g_mss_uart0_lo ;
mss_uart_instance_t*    g_debug_uart= &g_mss_uart0_lo ;
volatile uint32_t       g_count_sw_ints_h0 = 0U;
volatile uint32_t       g_10ms_count = 0U;
volatile uint64_t       g_tick_counter = 0U;

void test_init_peripherials(void)
{
    write_csr(mscratch, 0);
    write_csr(mcause, 0);
    write_csr(mepc, 0);

    /// Disable other harts. we enable as needed
    ///
    CLINT->MSIP[1] = 1;
    CLINT->MSIP[2] = 0;
    CLINT->MSIP[3] = 0;
    CLINT->MSIP[4] = 0;

    (void)mss_config_clk_rst(MSS_PERIPH_MMUART0, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_MMUART1, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_MMUART2, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_MMUART3, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_MMUART4, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_CFM, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_FIC0, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_FIC1, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_FIC2, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_FIC3, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_MAC0, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_MAC1, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_CFM, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_SPI0, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_QSPIXIP, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_GPIO1, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_I2C0, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_I2C1, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_CAN0, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_CAN1, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_EMMC, (uint8_t) MPFS_HAL_FIRST_HART,
            PERIPHERAL_ON);


    SYSREG->SOFT_RESET_CR = 0U;
    SYSREG->SUBBLK_CLOCK_CR = 0xFFFFFFFFUL;

    PLIC_init();

    SysTick_Config();
}

void display_test_result(const uint8_t* test_name, uint8_t test_result)
{
    uint8_t message[256];
    sprintf((char*)message, " %s %s\n\r", (const char*)test_name, (test_result == 0 ? "was successful" : test_result == 1 ? "was not performed" : "failed" ));

    MSS_UART_polled_tx_string(g_uart,(const uint8_t*)message);
}

void display_version(void)
{
    const char* version = "v0";

    delay(DELAY_CYCLES_100MS * 10);

    MSS_UART_polled_tx_string(g_uart,(const uint8_t*)"\n\n\r \033[48;5;24m\033[90m                                                      \033[0m\n\r");
    MSS_UART_polled_tx_string(g_uart,(const uint8_t*)" \033[48;5;24m\033[38;5;0m         Orbsight Software v2 RFIM Space              \033[0m\n\r");
    MSS_UART_polled_tx_string(g_uart,(const uint8_t*)" \033[48;5;24m\033[90m                                                      \033[0m\n\r");

    uint8_t message[256];
    sprintf((char*)message, "\n\r\n\r Microchip Polarfire SoC FPGA\n\r DMA Streaming over ethernet %s %s %s\n\r", version, __DATE__, __TIME__);
    MSS_UART_polled_tx_string(g_uart,(const uint8_t*)message);

    sprintf((char*)message, " Copyright \u00A9 2025 RFIM Space Ltd.\n\r\n\r");
    MSS_UART_polled_tx_string(g_uart,(const uint8_t*)message);
}

void e51(void)
{
    uint8_t test_result_dma         = 1;
    uint8_t test_result_mac         = 1;

    test_init_peripherials();

    display_version();

    test_result_mac         = test_mac();                   /// This one works!
    test_result_dma         = test_dma_streaming();

    test_display_start("Tests Results Summary");
    MSS_UART_polled_tx_string(g_uart,(const uint8_t*)"\n\r");

    display_test_result((const uint8_t* )"DMA Streaming Test", test_result_dma);
    display_test_result((const uint8_t* )"MAC Ethernet Loopback Test", test_result_mac);

    while (1);
}

void SysTick_Handler_h0_IRQHandler(void)
{
    g_count_sw_ints_h0  ++;
    g_tick_counter      += HART0_TICK_RATE_MS;
    g_10ms_count        = g_tick_counter / (HART0_TICK_RATE_MS * 2);
}

