#include <stdio.h>
#include <string.h>
#include "mpfs_hal/mss_hal.h"
#include "inc/common.h"
#include "inc/helper.h"
#include "drivers/mss/mss_sys_services/mss_sys_services.h"
#include "drivers/mss/CoreSPI/core_spi.h"
#include "drivers/mss/mt25ql01gbbb/micron1gflash.h"

/******************************************************************************/
#define MSS_SYS_MAILBOX_DATA_OFFSET                           0u
#define IMAGE_IDX                               2u
#define GOLDEN_IMAGE_SPI_ADDRESS 0x400
#define UPDATE_IMAGE_SPI_ADDRESS 0xA00000
#define IAP_IMAGE_SPI_ADDRESS 0x1400000

/******************************************************************************/
mss_uart_instance_t *g_uart= &g_mss_uart0_lo;

/******************************************************************************/
const uint8_t g_greeting_msg0[] = "\r\n\r\n\
       ******* PolarFire SoC SPI Flash testing *******\n\r";

const uint8_t g_greeting_msg[] =
"\r\n\r\n\t  ******* PolarFire SoC system services testing *******\n\n\n\r\
\n\n\r\
Note: This application demonstrates the execution of some of the system services. \n\r\
\r\n\n\
\r\n0. Print the greeting message \r\n\
\r\n1. Get Design Information \r\n\
\r\n2. Bitstream authentification \r\n\
\r\n3. IAP authentification\r\n\
\r\n4. Auto_update \n\r\
\r\n5. Execute IAP \n\r\
\r\n6. Device Certificate Service \n\r";

uint8_t g_message1[] =
"\r\n\r\n\
Write-Read test on the SPI flash successful\r\n\
Data stored in g_flash_rd_buf is identical from the data stored in g_flash_wr_buf";

uint8_t g_message2[] =
"\r\n\r\n\
Write-Read test on the SPI flash failed\r\n\
\r\n\r\n\
Data stored in g_flash_rd_buf is different from the data stored in g_flash_wr_buf";

/*------------------------------------------------------------------------------
 * Private functions.
 */
static int8_t verify_write(uint8_t* write_buff, uint8_t* read_buff, uint32_t size)
{
    int8_t error = 0u;
    uint32_t index = 0u;
    uint8_t p_buff[100];

    while (size != 0u)
    {
        if (write_buff[index] != read_buff[index])
        {
            error = -2;
            break;
        }

        index++;
        size--;
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
        snprintf(message, sizeof(message), "\n\n\rreading %0X: ", index + address);
        MSS_UART_polled_tx_string(g_uart, (const uint8_t*)message);
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
        snprintf(message, sizeof(message), "\n\n\rwriting %0X: ", index + address);
        MSS_UART_polled_tx_string(g_uart, (const uint8_t*)message);
        index += num_bytes;
        for(uint16_t j = 0; j < num_bytes; j++) {
            snprintf(message, sizeof(message), "%02X ", buf[j]);
            MSS_UART_polled_tx_string(g_uart, (const uint8_t*)message);
        }
    }
}

void erase_flash(uint32_t address, uint8_t* message) {
    snprintf(message, sizeof(message), "\n\n\rerasing %0X ", address);
    MSS_UART_polled_tx_string(g_uart, (const uint8_t*)message);
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

    mss_config_clk_rst(MSS_PERIPH_MMUART0, (uint8_t) 1, PERIPHERAL_ON);

    MSS_UART_init(g_uart,
            MSS_UART_115200_BAUD,
            MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

    MSS_UART_polled_tx_string(g_uart, (const uint8_t*)g_greeting_msg);

    MSS_UART_polled_tx_string (g_uart, g_greeting_msg0);

    /***************************SPI Initilization******************************/
    uint8_t manufacturer_id, device_id;
    FLASH_init();
    FLASH_global_unprotect();
    FLASH_read_device_id(&manufacturer_id, &device_id);

    /***********************Verify Device and Manufacturer ID******************/
    char message[20];
    snprintf(message, sizeof(message), "Device ID: %u\n", device_id);
    MSS_UART_polled_tx_string(g_uart, (const  uint8_t*)message);
    MSS_UART_polled_tx_string(g_uart, (const  uint8_t*)"\r\n");
    snprintf(message, sizeof(message), "Manufacturer ID: %u\n", manufacturer_id);
    MSS_UART_polled_tx_string(g_uart, (const  uint8_t*)message);

    /*********************Initialize Read and Write Buffer*********************/
    static uint8_t g_flash_wr_buf[BUFFER_SIZE];
    static uint8_t g_flash_rd_buf[BUFFER_SIZE];
    for (uint32_t loop_count = (BUFFER_SIZE); loop_count < BUFFER_SIZE; loop_count++)  {
        g_flash_wr_buf[loop_count] = 0x33U;
        g_flash_rd_buf[loop_count] = 0x00U;
    }

    /******************Program the SPI flash, Read back the data***************/
    uint32_t flash_address = 0x00000000; // Starting address to read from
    FLASH_program(flash_address, g_flash_wr_buf, sizeof(g_flash_wr_buf));
    FLASH_read(flash_address, g_flash_rd_buf, sizeof(g_flash_rd_buf));
    volatile uint32_t errors = verify_write(g_flash_wr_buf, g_flash_rd_buf, sizeof(g_flash_wr_buf));
    //check if there are errors
    if (0 == errors) {
       MSS_UART_polled_tx_string(g_uart, (const  uint8_t*)g_message1);
    }
    else {
        MSS_UART_polled_tx_string(g_uart, (const  uint8_t*)g_message2);
    }

    /*************************SPI read, erase, write test**********************/
    uint32_t address1 = GOLDEN_IMAGE_SPI_ADDRESS;
    uint32_t index = 0;
    size_t num_bytes = 256;
    uint8_t g_write_buf[num_bytes];
    uint8_t g_read_buf[num_bytes];
    uint8_t num_blocks = 5;

    // Initialize write buffer
    for (uint16_t i = 0; i < num_bytes; i++) {
        g_write_buf[i] = 0xAB;
    }

    // Reading
    read_flash(address1, num_bytes, num_blocks, g_read_buf, message);

    // Erasing
    erase_flash(address1, message);

    // Reading after erasing
    read_flash(address1, num_bytes, num_blocks, g_read_buf, message);

    // Writing
    write_flash(address1, num_bytes, num_blocks, g_write_buf, message);

    // Reading after writing
    read_flash(address1, num_bytes, num_blocks, g_read_buf, message);

    /**********************************IAP test********************************/
    // IAP service and result reporting
    uint32_t spi_idx = 0;
    uint32_t flash_addresses[3] = {GOLDEN_IMAGE_SPI_ADDRESS,UPDATE_IMAGE_SPI_ADDRESS,IAP_IMAGE_SPI_ADDRESS};
    uint8_t result = MSS_SYS_execute_iap(MSS_SYS_IAP_PROGRAM_BY_SPIADDR_CMD, flash_addresses[2]);
    char buff[20];
    sprintf(buff, "%d", result);
    MSS_UART_polled_tx(g_uart, (uint8_t *)buff, strlen(buff));

    // IAP authentication
    uint32_t spi_directory = 0x00000000;
    FLASH_program(spi_directory, (uint8_t*)&flash_addresses[0], sizeof(uint32_t));
    result = MSS_SYS_authenticate_iap_image(spi_idx);
    sprintf(buff, "%d", result);
    MSS_UART_polled_tx(g_uart, (uint8_t *)buff, strlen(buff));

    // Bitstream authentication
    uint16_t mb_offset = 0;
    result = MSS_SYS_authenticate_bitstream(IAP_IMAGE_SPI_ADDRESS, mb_offset);
    sprintf(buff, "%d", result);
    MSS_UART_polled_tx(g_uart, (uint8_t *)buff, strlen(buff));
}
