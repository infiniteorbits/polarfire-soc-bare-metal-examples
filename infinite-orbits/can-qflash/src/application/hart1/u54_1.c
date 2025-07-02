/*******************************************************************************
 * Copyright 2019-2022 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * Application code running on U54_1
 *
 * MSS QSPI driver example
 * Please refer README.md in the root folder of this example project
 */
#include "mpfs_hal/mss_hal.h"
#include "drivers/off_chip/micron_mt25q/micron_mt25q.h"
#include "drivers/mss/mss_mmuart/mss_uart.h"
#include "drivers/mss/mss_qspi/mss_qspi.h"
#include "drivers/mss/mss_mmuart/mss_can.h"
#include "inc/helper.h"
#include <stdio.h>
#include <string.h>


#define FLASH_MEMORY_SIZE                       0x2000000 /* (32MBytes Micron N25Q256A)*/
#define FLASH_PAGE_LENGTH                       2048

static uint8_t rd_buf[10] __attribute__ ((aligned (4)));
static uint8_t g_flash_wr_buf[FLASH_PAGE_LENGTH] __attribute__ ((aligned (4)));
static uint8_t g_flash_rd_buf[FLASH_PAGE_LENGTH] __attribute__ ((aligned (4)));
static uint8_t verify_write(uint8_t* write_buff, uint8_t* read_buff,
                            uint32_t size);

mss_uart_instance_t *g_uart = &g_mss_uart1_lo;
volatile uint32_t xip_read[10] = {0};
uint64_t uart_lock;
uint8_t g_ui_buf[500];
const uint8_t g_greeting_msg[] =
"\r\n\n\n **** PolarFire SoC MSS CAN - QSPI flash ****\n\n\r";
uint32_t page_num;

/*------------------------------------------------------------------------------
 * MSS CAN
 */
mss_can_instance_t* g_mss_can_1 = &g_mss_can_1_lo;
mss_can_filterobject pfilter;
mss_can_msgobject rx_buf;
mss_can_rxmsgobject rx_msg;
uint8_t ret_status;
uint32_t g_bytes_received = 0;
uint32_t g_head = 0;
uint32_t g_tail = 0;
uint32_t g_block_count = 0u;
uint8_t g_tx_buff[FLASH_PAGE_LENGTH] = {0};
uint8_t g_rx_buff[FLASH_PAGE_LENGTH] = {0};
char message[20];

/* Main function for the HART1(U54_1 processor).
 * Application code running on HART1 is placed here.
 */
void u54_1(void)
{
    int8_t info_string[100];
    uint8_t rx_buff[1];
    uint8_t rx_size = 0;
    uint32_t loop_count;
    uint8_t error=0;

#if (IMAGE_LOADED_BY_BOOTLOADER == 0)
    /* Clear pending software interrupt in case there was any.
     * Enable only the software interrupt so that the E51 core can bring this
     * core out of WFI by raising a software interrupt In case of external,
     * bootloader not present
     */

    clear_soft_interrupt();
    set_csr(mie, MIP_MSIP);

    /*Put this hart into WFI.*/

    do
    {
        __asm("wfi");
    }while(0 == (read_csr(mip) & MIP_MSIP));

    /* The hart is out of WFI, clear the SW interrupt. Hear onwards Application
     * can enable and use any interrupts as required */
    clear_soft_interrupt();
#endif
    (void)mss_config_clk_rst(MSS_PERIPH_MMUART1, (uint8_t) MPFS_HAL_FIRST_HART, PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_MMUART2, (uint8_t) MPFS_HAL_FIRST_HART, PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_MMUART3, (uint8_t) MPFS_HAL_FIRST_HART, PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_MMUART4, (uint8_t) MPFS_HAL_FIRST_HART, PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_CFM, (uint8_t) MPFS_HAL_FIRST_HART, PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_QSPIXIP, (uint8_t) MPFS_HAL_FIRST_HART, PERIPHERAL_ON);

    MSS_UART_init( g_uart,
            MSS_UART_115200_BAUD,
            MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

    PLIC_init();
    __enable_irq();

    PLIC_SetPriority(QSPI_PLIC,2);
    PLIC_EnableIRQ(QSPI_PLIC);

    /* Initialize the write and read buffers */
    for(loop_count = 0; loop_count < (FLASH_PAGE_LENGTH); loop_count++)
    {
        g_flash_wr_buf[loop_count] = (0x15 + loop_count);
        g_flash_rd_buf[loop_count] = 0x00;
    }

    /***************************CAN Initialization******************************/

    (void)mss_config_clk_rst(MSS_PERIPH_CAN0, (uint8_t) 1, PERIPHERAL_ON);
   (void)mss_config_clk_rst(MSS_PERIPH_CAN1, (uint8_t) 1, PERIPHERAL_ON);

   PLIC_DisableIRQ(CAN0_PLIC);
   PLIC_DisableIRQ(CAN1_PLIC);

   ret_status = MSS_CAN_init(g_mss_can_1, CAN_SPEED_8M_1M,
                                    (pmss_can_config_reg)0, 6u, 6u);
   MSS_CAN_set_mode(g_mss_can_1, CANOP_MODE_NORMAL);
   MSS_CAN_start(g_mss_can_1);

   /* Configure for receive */
   rx_msg.ID = 0x0u;
   rx_msg.DATAHIGH = 0u;
   rx_msg.DATALOW = 0u;
   rx_msg.AMR.L = 0xFFFFFFFFu;
   rx_msg.ACR.L = 0x00000000u;
   rx_msg.AMR_D = 0xFFFFFFFFu;
   rx_msg.ACR_D = 0x00000000u;
   rx_msg.RXB.DLC = 8u;
   rx_msg.RXB.IDE = 0u;

   rx_msg.ID = 0x00;
   ret_status = MSS_CAN_config_buffer_n(g_mss_can_1, 1, &rx_msg);
   if (CAN_OK != ret_status)
   {
       MSS_UART_polled_tx_string(g_uart,
              (const uint8_t *)"\n\r CAN 1 Message Buffer configuration Error");
   }else
   {
       MSS_UART_polled_tx_string(g_uart,
                      (const uint8_t *)"\n\r CAN 1 Message Buffer configuration Completed");
   }

    /***************************SPI Initialization******************************/

    MSS_UART_polled_tx_string(g_uart, g_greeting_msg);

    MSS_UART_polled_tx_string (g_uart, "\r\n\n\nUsing Normal IO format\r\n");
    Flash_init(MSS_QSPI_QUAD_FULL);

    MSS_UART_polled_tx_string (g_uart, "\r\nDevice ID\r\n");
    Flash_readid(rd_buf);
    display_output(rd_buf, 3);

    /*************************SPI test erase**********************/
   /* Flash_read(g_rx_buff, (0*FLASH_PAGE_LENGTH), FLASH_PAGE_LENGTH);
    sprintf((char*)&g_ui_buf[0], "\r\n\r\nPage num1 = %d\r\n", 0);
    MSS_UART_polled_tx_string (g_uart, (uint8_t*)g_ui_buf);
    display_output(g_rx_buff, FLASH_PAGE_LENGTH);*/

    g_rx_buff[0] = Flash_erase();
    sprintf((char*)&g_ui_buf[0], "\r\n\r\nErase operation complete. status = %x\r\n", g_rx_buff[0]);
    MSS_UART_polled_tx_string (g_uart, (uint8_t*)g_ui_buf);

   /* Flash_read(g_rx_buff, (0*FLASH_PAGE_LENGTH), FLASH_PAGE_LENGTH);
    sprintf((char*)&g_ui_buf[0], "\r\n\r\nPage num = %d\r\n\r\n\r", 0);
    MSS_UART_polled_tx_string (g_uart, (uint8_t*)g_ui_buf);
    display_output(g_rx_buff, FLASH_PAGE_LENGTH);*/

    while (1)
   {
      ret_status = MSS_CAN_get_message_n(g_mss_can_1, 1u, &rx_buf);

      if (CAN_VALID_MSG == ret_status)
      {
          // Copy CAN message data to buffer
          uint32_t remaining_bytes = FLASH_PAGE_LENGTH - g_bytes_received;
          uint32_t bytes_to_copy = rx_buf.DLC < remaining_bytes ? rx_buf.DLC : remaining_bytes;

          for (uint32_t loop_count = 0u; loop_count < bytes_to_copy; loop_count++)
          {
              g_tx_buff[g_head] = rx_buf.DATA[loop_count];
              g_head = (g_head + 1) % FLASH_PAGE_LENGTH;
              if (g_head == g_tail)
              {
                  // Buffer is full, move tail pointer to overwrite oldest data
                  g_tail = (g_tail + 1) % FLASH_PAGE_LENGTH;
              }
          }

          // Increment byte count by bytes copied
          g_bytes_received += bytes_to_copy;

          // If the buffer is full, post-process and reset the buffer
          if (g_bytes_received >= FLASH_PAGE_LENGTH)
          {
              sprintf(message, "\rBlocks written: %d ", g_block_count);
               MSS_UART_polled_tx_string(g_uart, message);

               ret_status = Flash_program(g_tx_buff, (g_block_count * FLASH_PAGE_LENGTH), FLASH_PAGE_LENGTH);
               Flash_read(g_rx_buff, (g_block_count*FLASH_PAGE_LENGTH), FLASH_PAGE_LENGTH);
               //display_output(g_rx_buff, FLASH_PAGE_LENGTH);

               ret_status = verify_write(g_tx_buff, g_rx_buff, FLASH_PAGE_LENGTH);
               if (ret_status)
               {
                   sprintf((char*)&g_ui_buf[0], "\r\n\r\nfailed at Page number - %d", g_block_count);
                   MSS_UART_polled_tx_string (g_uart, (uint8_t*)g_ui_buf);
                   break;
               }

              memset(g_tx_buff, 0, sizeof(g_tx_buff));
              memset(g_rx_buff, 0, sizeof(g_rx_buff));
              g_block_count++;
              g_head = 0;
              g_tail = 0;
              g_bytes_received = 0;

          }
      }
  }
}

/***************************************************************************//**
 * Read the date from SPI flash and compare the same with write buffer.
 */
static uint8_t verify_write(uint8_t* write_buff, uint8_t* read_buff,
        uint32_t size)
{
    uint8_t error = 0u;
    uint32_t index = 0u;

    while (size != 0u)
    {
        if (write_buff[index] != read_buff[index])
        {
            error = 1u;
            sprintf((char*)&g_ui_buf[0], "\r\nExpected:0x%x   Received:0x%x   Index:0x%x\r\n",
                                          write_buff[index],
                                          read_buff[index],
                                          index);
            MSS_UART_polled_tx_string (g_uart, (uint8_t*)g_ui_buf);

        }

        index++;
        size--;
    }

    return error;
}

/* HART1 Software interrupt handler */
void Software_h1_IRQHandler (void)
{

}

