#include <stdio.h>
#include <string.h>
#include "mpfs_hal/mss_hal.h"
#include "inc/common.h"
#include "inc/helper.h"
#include "drivers/mss/mss_sys_services/mss_sys_services.h"
#include "drivers/mss/mss_mmuart/mss_can.h"
#include "drivers/mss/CoreSPI/core_spi.h"
#include "drivers/mss/mt25ql01gbbb/micron1gflash.h"

/******************************************************************************/
#define GOLDEN_IMAGE_SPI_ADDRESS 0x400
#define UPDATE_IMAGE_SPI_ADDRESS 0xA00000
#define IAP_IMAGE_SPI_ADDRESS    0x1400000
#define BLOCK_SIZE_BYTES         512
#define SIZE_64KB                0x00010000
#define ERASE_LIMIT             (500 * 1024)  // 500 KB

/******************************************************************************/
const uint8_t g_greeting_msg0[] = "\r\n\r\n  ******* PolarFire SoC CAN - SPI Flash *******\n\r\
        \r\nReceive payload over CAN and Write on the SPI flash \r\n";
char message[20];

uint8_t ret_status;
uint16_t sector_number = 0;
uint32_t g_bytes_received = 0;
uint32_t g_head = 0;
uint32_t g_tail = 0;
uint32_t g_block_count = 0u;
uint8_t manufacturer_id, device_id;
uint32_t start_address = 0x03000000;

/*------------------------------------------------------------------------------
 * MSS CAN
 */
mss_can_instance_t* g_mss_can_1 = &g_mss_can_1_lo;
mss_can_filterobject pfilter;
mss_can_msgobject rx_buf;
mss_can_rxmsgobject rx_msg;

/*------------------------------------------------------------------------------
 * MSS
 */
mss_uart_instance_t *g_uart= &g_mss_uart1_lo;
uint8_t g_mmc_tx_buff[BLOCK_SIZE_BYTES] = {0};
uint8_t g_mmc_rx_buff[BLOCK_SIZE_BYTES] = {0};

/*------------------------------------------------------------------------------
 * Private functions.
 */
void read_flash(uint32_t address, size_t num_bytes, uint8_t num_blocks, uint8_t* buf, uint8_t* message);
static int8_t verify_write(uint8_t* write_buff, uint8_t* read_buff, uint32_t size)
{
    int8_t error = 0u;
    uint32_t index = 0u;
    uint16_t len = size;

    while (len)
    {
        if (write_buff[index] != read_buff[index])
        {
            error = 1;
            MSS_UART_polled_tx_string(g_uart,
                           (const uint8_t *)"\n\r\n\rError\n\r");
            break;
        }

        index++;
        len--;
    }
    return error;
}
void delay1(volatile uint32_t n)
{
    while(n)
        n--;
}

void read_flash(uint32_t address, size_t num_bytes, uint8_t num_blocks, uint8_t* buf, uint8_t* message) {
    uint32_t index = 0;
    for(uint8_t i = 0; i < num_blocks; i++) {
        FLASH_read(index + address, buf, num_bytes);

        index += num_bytes;
        for(uint16_t j = 0; j < num_bytes; j++) {
            snprintf(message, sizeof(message), "%02X ", buf[j]);
            MSS_UART_polled_tx_string(g_uart, (const uint8_t*)message);
        }
    }
}

void write_flash(uint32_t address, size_t num_bytes, uint8_t num_blocks, uint8_t* buf, uint8_t* message) {
    uint32_t index = 0;
    for(uint8_t i = 0; i < num_blocks; i++) {
        FLASH_program(index + address, buf, num_bytes);
        index += num_bytes;
        for(uint16_t j = 0; j < num_bytes; j++) {
            snprintf(message, sizeof(message), "%02X ", buf[j]);
            MSS_UART_polled_tx_string(g_uart, (const uint8_t*)message);
        }
    }
}

void erase_flash(uint32_t address, uint8_t* message) {
    FLASH_erase_64k_block(address);
    delay1(500);
}

/*------------------------------------------------------------------------------
 * Global Functions
 */

void u54_1(void)
{


#if (IMAGE_LOADED_BY_BOOTLOADER == 0)
    /* Clear pending software interrupt in case there was any.
     * Enable only the software interrupt so that the E51 core can bring this
     * core out of WFI by raising a software interrupt. */
    clear_soft_interrupt();
    set_csr(mie, MIP_MSIP);

    /*Put this hart into WFI.*/
    do
    {
        __asm("wfi");
    }while(0 == (read_csr(mip) & MIP_MSIP));
#endif

    /* The hart is out of WFI, clear the SW interrupt. Hear onwards Application
     * can enable and use any interrupts as required */
    clear_soft_interrupt();
    /* Reset FPGA to access the SDIO register at FIC3 */
    SYSREG->SOFT_RESET_CR   &= (uint32_t)~(SOFT_RESET_CR_FPGA_MASK);

    mss_config_clk_rst(MSS_PERIPH_MMUART1, (uint8_t) 1, PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_CAN0, (uint8_t) 1, PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_CAN1, (uint8_t) 1, PERIPHERAL_ON);

    PLIC_DisableIRQ(CAN0_PLIC);
    PLIC_DisableIRQ(CAN1_PLIC);

    MSS_UART_init(g_uart,
            MSS_UART_115200_BAUD,
            MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);
    MSS_UART_polled_tx_string (g_uart, g_greeting_msg0);

    ret_status = MSS_CAN_init(g_mss_can_1, CAN_SPEED_8M_1M,
                                     (pmss_can_config_reg)0, 6u, 6u);
    MSS_CAN_set_mode(g_mss_can_1, CANOP_MODE_NORMAL);
    MSS_CAN_start(g_mss_can_1);

    PLIC_init();
    __enable_irq();

    /* ----------------------- CAN - 1 Initialization   ----------------- */
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
    }

    /***************************SPI Initialization******************************/
    FLASH_init();
    FLASH_global_unprotect();
    FLASH_read_device_id(&manufacturer_id, &device_id);

    /***********************Verify Device and Manufacturer ID******************/
    snprintf(message, sizeof(message), "Device ID: %u\n\r", device_id);
    MSS_UART_polled_tx_string(g_uart, (const  uint8_t*)message);
    snprintf(message, sizeof(message), "Manufacturer ID: %u\n\r", manufacturer_id);
    MSS_UART_polled_tx_string(g_uart, (const  uint8_t*)message);

    /*************************SPI read, erase, write test**********************/
    //MSS_UART_polled_tx_string(g_uart, (const uint8_t *)"Reading\n\r");
    //read_flash(start_address, BLOCK_SIZE_BYTES, 1, g_mmc_rx_buff, message);

    MSS_UART_polled_tx_string(g_uart, (const uint8_t *)"\n\rStart erase\n\r");
    for (uint32_t offset = 0; offset < ERASE_LIMIT; offset += SIZE_64KB) {
        erase_flash(start_address + offset, message);
    }
    MSS_UART_polled_tx_string(g_uart, (const uint8_t *)"End erase\n\r");

    //MSS_UART_polled_tx_string(g_uart, (const uint8_t *)"Reading\n\r");
    //read_flash(start_address, BLOCK_SIZE_BYTES, 1, g_mmc_rx_buff, message);
    MSS_UART_polled_tx_string(g_uart, (const uint8_t *)"\n\rStart uploading\n\n\r");

    while (1)
    {
       ret_status = MSS_CAN_get_message_n(g_mss_can_1, 1u, &rx_buf);

       if (CAN_VALID_MSG == ret_status)
       {
           // Copy CAN message data to buffer
           uint32_t remaining_bytes = BLOCK_SIZE_BYTES - g_bytes_received;
           uint32_t bytes_to_copy = rx_buf.DLC < remaining_bytes ? rx_buf.DLC : remaining_bytes;

           for (uint32_t loop_count = 0u; loop_count < bytes_to_copy; loop_count++)
           {
               g_mmc_tx_buff[g_head] = rx_buf.DATA[loop_count];
               g_head = (g_head + 1) % BLOCK_SIZE_BYTES;
               if (g_head == g_tail)
               {
                   // Buffer is full, move tail pointer to overwrite oldest data
                   g_tail = (g_tail + 1) % BLOCK_SIZE_BYTES;
               }
           }

           // Increment byte count by bytes copied
           g_bytes_received += bytes_to_copy;

           // If the buffer is full, post-process and reset the buffer
           if (g_bytes_received >= BLOCK_SIZE_BYTES)
           {
               sprintf(message, "\rContents of g_mmc_tx_buff before write: %d ", g_block_count);
               MSS_UART_polled_tx_string(g_uart, message);
               FLASH_program(start_address+ g_block_count, g_mmc_tx_buff, BLOCK_SIZE_BYTES);
               FLASH_read(start_address+ g_block_count, g_mmc_rx_buff, BLOCK_SIZE_BYTES);
               if(verify_write(g_mmc_tx_buff, g_mmc_rx_buff, BLOCK_SIZE_BYTES)) break;


               memset(g_mmc_tx_buff, 0, sizeof(g_mmc_tx_buff));
               memset(g_mmc_rx_buff, 0, sizeof(g_mmc_rx_buff));
               g_block_count+=BLOCK_SIZE_BYTES;
               g_head = 0;
               g_tail = 0;
               g_bytes_received = 0;

           }
       }
   }
}
