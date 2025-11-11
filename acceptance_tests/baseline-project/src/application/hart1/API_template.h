/**
 * @file API_template.h
 * @brief Generic peripheral API template for OrbSight acceptance test applications.
 * @details
 * Each baremetal test application shall implement this minimal API structure,
 * providing consistent initialization, read/write access, and logging across all peripherals.
 *
 * @note Replace `PERIPHERAL_` with the specific name (e.g., CAN_, DMA_, UART_, etc.)
 *       when creating the actual API implementation.
 */

#ifndef API_TEMPLATE_H
#define API_TEMPLATE_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Initialize the peripheral and verify accessibility.
 *
 * @return uint8_t  0 on success, non-zero error code on failure.
 */
uint8_t PERIPHERAL_init(void);

/**
 * @brief Perform a read or capture operation from the peripheral.
 *
 * @param buffer Pointer to buffer where the read data will be stored.
 * @param size   Size of the buffer in bytes.
 * @return uint8_t  0 on success, non-zero error code on failure.
 */
uint8_t PERIPHERAL_read(uint8_t *buffer, size_t size);

/**
 * @brief Perform a write or transmission operation to the peripheral.
 *
 * @param buffer Pointer to data to be written/transmitted.
 * @param size   Size of the data in bytes.
 * @return uint8_t  0 on success, non-zero error code on failure.
 */
uint8_t PERIPHERAL_write(const uint8_t *buffer, size_t size);

/**
 * @brief Log a message or event related to the peripheral operation.
 *
 * @param level Log level (e.g., INFO, WARN, ERROR).
 * @param msg   Message string to be logged.
 */
void PERIPHERAL_log(const char *level, const char *msg);

/**
 * @brief Deinitialize or stop the peripheral safely.
 *
 * @return uint8_t  0 on success, non-zero error code on failure.
 */
uint8_t PERIPHERAL_deinit(void);

#endif /* API_TEMPLATE_H */
