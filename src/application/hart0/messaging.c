
#include <stdio.h>
#include <string.h>
#include "mpfs_hal/mss_hal.h"
#include "mpfs_hal/common/nwc/mss_nwc_init.h"
#include "mpfs_hal/mpfs_hal_version.h"
#include "inc/common.h"


extern mss_uart_instance_t*         g_uart;

void test_display_start(const char* message)
{
    delay(DELAY_CYCLES_100MS * 10);

    MSS_UART_polled_tx_string(g_uart,(const uint8_t*)"\n\n\r \033[48;5;80m\033[38;5;0m                                                      \033[0m\n\r");
    MSS_UART_polled_tx_string(g_uart,(const uint8_t*)"\n\n\r           Starting                                    \n\r             ");
    MSS_UART_polled_tx_string(g_uart,(const uint8_t*)message);
    MSS_UART_polled_tx_string(g_uart,(const uint8_t*)"\n\n\r");
    //MSS_UART_polled_tx_string(g_uart,(const uint8_t*)"\n\n\r ****************************************************** \n\r");

    delay(DELAY_CYCLES_100MS * 10);
}

void test_display_finished(void)
{
    /// MSS_UART_polled_tx_string(g_uart,(const uint8_t*)"\n\n\r ****************************************************** \n\r");
    MSS_UART_polled_tx_string(g_uart,(const uint8_t*)"\n\n\n\r             Finished");
    /// MSS_UART_polled_tx_string(g_uart,(const uint8_t*)"\n\n\r ****************************************************** \n\r");

    delay(DELAY_CYCLES_100MS * 10);
}
