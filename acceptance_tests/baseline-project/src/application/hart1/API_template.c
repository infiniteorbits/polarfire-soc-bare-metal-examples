/**
 * @file API_template.c
 * @brief Generic peripheral API implementation template for OrbSight acceptance tests.
 */

// TODO: Configure hardware-specific API functions by replacing PERIPHERAL for (GPIO, CAN, DMA, etc.)
// Example:
/*
uint8_t CAN_init(void)
{
    CAN_log("CAN", "Peripheral initialized successfully.");
    return 0;
}
*/

#include "API_template.h"
#include <stdio.h>      // For printf() in log
#include <string.h>     // For memset, memcpy
#include "inc/common.h"
#include "mpfs_hal/mss_hal.h"

/* Example internal state */
static uint8_t peripheral_initialized = 0;
char info_string[100];

uint8_t PERIPHERAL_init(void)
{
    // TODO: Add hardware-specific initialization (GPIO, CAN, DMA, etc.)
    // Example:
    // CAN_EnableClock();
    // CAN_Reset();
    // CAN_ConfigRegisters();

    peripheral_initialized = 1;
    PERIPHERAL_log("PERIPHERAL", "Peripheral initialized successfully.");
    return 0;
}

uint8_t PERIPHERAL_read(uint8_t *buffer, size_t size)
{
    if (!peripheral_initialized) {
        PERIPHERAL_log("PERIPHERAL", "Peripheral not initialized.");
        return 1;
    }

    // TODO: Replace with actual read operation
    memset(buffer, 0xAB, size);  // Dummy data for testing
    PERIPHERAL_log("PERIPHERAL", "Read operation completed.");
    return 0;
}

uint8_t PERIPHERAL_write(const uint8_t *buffer, size_t size)
{
    if (!peripheral_initialized) {
        PERIPHERAL_log("PERIPHERAL", "Peripheral not initialized.");
        return 1;
    }

    // TODO: Replace with actual write operation
    (void)buffer;
    (void)size;
    PERIPHERAL_log("PERIPHERAL", "Write operation completed.");
    return 0;
}

void PERIPHERAL_log(const char *peripheral, const char *msg)
{
    if (g_uart == NULL) {
        return; // Logging unavailable
    }

    /* Format log message */
    sprintf(info_string, "[%s] %s\r\n", peripheral, msg);

    /* Transmit over UART */
    MSS_UART_polled_tx(g_uart, (const uint8_t *)info_string, strlen(info_string));
}

uint8_t PERIPHERAL_deinit(void)
{
    if (!peripheral_initialized)
        return 0;

    // TODO: Add deinitialization or stop sequence
    peripheral_initialized = 0;
    PERIPHERAL_log("PERIPHERAL", "Peripheral deinitialized.");
    return 0;
}
