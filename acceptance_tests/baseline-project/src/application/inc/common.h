/*******************************************************************************
 * Copyright 2019-2021 Microchip FPGA Embedded Systems Solution.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>
#include <stdbool.h>
#include "drivers/mss/mss_mmuart/mss_uart.h"
#include "mpfs_hal/startup_gcc/system_startup.h"

extern mss_uart_instance_t *g_uart;

#endif /* COMMON_H_ */
