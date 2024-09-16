/*******************************************************************************
 * Copyright 2019-2021 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * Application code running on U54_1
 *
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mpfs_hal/mss_hal.h"
#include "drivers/mss/mss_mmc/mss_mmc.h"
#include "drivers/mss/mss_mmuart/mss_can.h"
#include "inc/common.h"

//spi includes

#include "drivers/mss/CoreSPI/core_spi.h"
#include "drivers/mss/mss_mmuart/mss_uart_regs.h"
#include "drivers/mss/mt25ql01gbbb/micron1gflash.h"
#include "drivers/mss/mss_mmuart/coresysservicespf_regs.h"
#include "drivers/mss/mss_mmuart/core_sysservices_pf.h"

// Define state enum
typedef enum {
    IDLE,
    READ_SD_CARD,
    WAIT_FOR_CAN0,
    TRANSMIT_CAN1,
    WRITE_TO_EMMC,
} state_t;

#define LIM_BASE_ADDRESS        0x08000000u
#define LIM_SIZE                0x200000u
#define ERROR_INTERRUPT         0x8000u
#define TRANSFER_COMPLETE       0x1u
#define SECT_NO                 16384u
//#define BUFFER_SIZE             4096u // multiple block data size
#define USE_SDMA                0u
#define USE_ADMA2               1u
#define RX_BUFF_SIZE            10u
#define BLOCK_SIZE              512u



#define   ENTER               0x0DU
#define NUM_BLOCKS 1364  // replace with actual size of BIOS boot partition
#define BUFFER_SIZE (512)  // buffer size is 4 blocks (2KB)
#define FLASH_IMAGE_ADDRESS 0x00000000
#define GOLDEN_IMAGE_SPI_ADDRESS 0x400
#define UPDATE_IMAGE_SPI_ADDRESS 0xA00000
#define IAP_IMAGE_SPI_ADDRESS 0x1400000
#define CAN_MSG_DATA_LEN 8

uint8_t job_buffer[512];
uint8_t job_read_buffer[512];
spi_instance_t g_core_spi;
uint32_t g_flash_address = 0;

uint8_t g_read_buf[BUFFER_SIZE];
uint32_t flash_address[3] = {GOLDEN_IMAGE_SPI_ADDRESS,UPDATE_IMAGE_SPI_ADDRESS,IAP_IMAGE_SPI_ADDRESS};
uint32_t flash_image = FLASH_IMAGE_ADDRESS;
//spi function defs
static void display_hex_values(const uint8_t *, uint32_t);

/**********************************SPI FUNCTIONS**************************************/
void write_flash_directory()
{
    uint32_t address1 = 0x400,address2 = 0xA00000,address3 = 0x1400000;
    uint8_t buf[4];
    buf[3] = (uint8_t)((address1 >> 24) & 0xFF);
    buf[2] = (uint8_t)((address1 >> 16) & 0xFF);
    buf[1] = (uint8_t)((address1 >> 8) & 0xFF);
    buf[0] = (uint8_t)(address1 & 0xFF);
    FLASH_program(0 , buf, 4);
    FLASH_read(0,g_read_buf,4);
    buf[3] = (uint8_t)((address2 >> 24) & 0xFF);
    buf[2] = (uint8_t)((address2 >> 16) & 0xFF);
    buf[1] = (uint8_t)((address2 >> 8) & 0xFF);
    buf[0] = (uint8_t)(address2 & 0xFF);
    FLASH_program(4 , buf, 4);
    FLASH_read(4,g_read_buf,4);
    buf[3] = (uint8_t)((address3 >> 24) & 0xFF);
    buf[2] = (uint8_t)((address3 >> 16) & 0xFF);
    buf[1] = (uint8_t)((address3 >> 8) & 0xFF);
    buf[0] = (uint8_t)(address3 & 0xFF);
    FLASH_program(8 , buf, 4);
    FLASH_read(8,g_read_buf,4);
}
void delay1(volatile uint32_t n)
{
    while(n)
        n--;
}


/*------------------------------------------------------------------------------
/*------------------------------------------------------------------------------
 * Private functions.
 */
static void display_greeting(void);
static uint8_t get_data_frm_uart(void);
static void display_hex_values(const uint8_t *, uint32_t);
static void ascii_to_hex(uint8_t *, uint32_t );
static void display_option(void);
static void check_rx_buffer(void);
static int8_t single_block_read(uint32_t sector_number);
static int8_t multi_block_read_transfer(uint32_t sector_number);
static int8_t multi_block_read(uint32_t sector_number, uint8_t dma_type);
static void display_hex_values(const uint8_t * in_buffer, uint32_t byte_length);
static void can1_check_rx_buffer(void);
static int8_t multi_block_write_transfer(uint32_t sector_number);
static int8_t multi_block_write(uint32_t sector_number, uint8_t dma_type);

/*------------------------------------------------------------------------------
 * Static Variables.
 */
static uint8_t g_uart_to_can[BUFFER_SIZE]= {1};
static uint8_t g_temp[64];
static uint8_t g_can_to_uart[8];
static uint8_t bios_buffer[BUFFER_SIZE];

/*------------------------------------------------------------------------------
 * Global Variables.
 */
mss_can_filterobject pfilter;
mss_can_msgobject pmsg;
mss_can_msgobject pmsg1;

mss_can_msgobject rx_buf;
mss_can_rxmsgobject rx_msg;



/*------------------------------------------------------------------------------
 * MSS UART instance for UART1
 */

uint64_t uart_lock;

mss_can_instance_t* g_mss_can_0 = &g_mss_can_0_lo;
mss_can_instance_t* g_mss_can_1 = &g_mss_can_1_lo;



uint8_t circular_buffer[BUFFER_SIZE];
uint32_t read_pos = 0;
uint32_t write_pos = 0;
uint32_t num_items = 0;
uint8_t data_array[BUFFER_SIZE];




uint8_t g_mmc_rx_buff[BUFFER_SIZE] = {1};
uint8_t g_mmc_tx_buff[BUFFER_SIZE] = {1};
uint8_t display_buffer [BUFFER_SIZE] = {1};
mss_mmc_cfg_t g_mmc;

volatile uint32_t count_sw_ints_h1 = 0U;

volatile uint8_t g_rx_size = 0u;
mss_uart_instance_t *g_uart= &g_mss_uart0_lo;
static uint8_t g_mmc_initialized = 0u;

const uint8_t g_greeting_msg[] =
"\r\n\r\n\t******* PolarFire SoC MSS eMMC/SD Driver Example *******\n\n\n\r\
Data transfer status:\n\r\
  0 = Success TEST \n\r\
 -1 = Wrong DMA mode selected.(multi-block transfers only) \n\r\
 -2 = Read buffer did not match written values \r\n\
 status > 0 = Transfer failed. Refer mss_mmc_status_t in mss_mmc.h \r\n\n\
Note: Ensure that the Libero design supports the desired memory device. \r\n\n\
Refer README.md in the project root folder for more details.\r\n\n\n\
Menu:\r\n\
\r\n1. Execute data transfers on eMMC device \r\n\
\r\n2. Execute data transfers on SD card\r\n";

static int8_t verify_write(uint8_t* write_buff, uint8_t* read_buff, uint32_t size);
static int8_t mmc_init_sdcard(void);
static int8_t mmc_init_emmc(void);
static int8_t multi_block_transfer(uint32_t sector_number);
static int8_t multi_block_wr_rd(uint32_t sector_number, uint8_t dma_type);
static int8_t single_block_wr_rd(uint32_t sector_number);
static void report_status(int8_t status);
void transfer_complete_handler(uint32_t status);
void circular_buffer_add(uint8_t* data, uint32_t length);
static uint8_t g_can_to_uart[8];
void delay(uint32_t count)
{
    for (uint32_t i = 0; i < count; i++)
    {
        // Do nothing
    }
}
/* Main function for the hart1(U54_1 processor).
 * Application code running on hart1 is placed here
 *
 * The hart1 goes into WFI. hart0 brings it out of WFI when it raises the first
 * Software interrupt to this hart.
 */


 /**
    1.Checks if a CAN message has been received and validated (CAN_VALID_MSG == ret_status).
    2.If a valid message is received, copies the CAN message data into a circular buffer (g_mmc_tx_buff) until the buffer is full or all available message data has been received.
    3.When the buffer is full, the oldest data is overwritten by moving the g_tail pointer forward.
    4.Once the buffer is full, the data is written to an eMMC memory device using a multi-block transfer.
    5.The buffer is cleared and its pointers are reset to their initial values.
    6.The process starts over again, waiting for a new valid CAN message to be received.


                                  |<-- oldest data
                              v
             +------+------+------+------+------+------+------+------+------+------+------+------+------+------+------+------+
g_mmc_tx_buf | byte | byte | byte | byte | byte | byte | byte | byte | byte | byte | byte | byte | byte | byte | byte | byte |
             +------+------+------+------+------+------+------+------+------+------+------+------+------+------+------+------+
                                  ^                                                                       |
                                  |                                                                       |
                                  +----------------- newest data ---------------------------------------+

In the buffer, g_head points to the next available location to write new data, and g_tail points to the oldest data that will be overwritten when the buffer is full.
When new data is added to the buffer, g_head is moved forward, and when the buffer is full, g_tail is moved forward to overwrite the oldest data.
This allows the buffer to be used to store incoming data in a continuous circular manner.
 */
void u54_1(void)
{
    uint8_t p_buff[3 * 512];
    uint8_t g_rx_buff[RX_BUFF_SIZE] = {0};
    uint32_t loop_count = 0;
    uint32_t loop_SD_READ = 0;
    uint32_t sector_number = SECT_NO;
    int8_t status = 0u;

    uint32_t start_sector = 0; // starting sector of BIOS partition



    /**CAN**/
    uint8_t ret_status;
    uint8_t rx_bytes = 0u;
    uint32_t no_of_msgs = 0u;
    uint8_t init_return_value = 0u;
    uint8_t rx_char, count;
    uint8_t loop_count_CAN;
    uint32_t msg_len;
    uint32_t chunk_size;
    uint32_t error_flag;
    uint32_t tx_status = 0u;
    size_t  rx_size;
    uint32_t g_bytes_received = 0;
    uint32_t g_head = 0;
    uint32_t g_tail = 0;
    uint32_t  g_block_count =0u;

    volatile uint32_t erase_address = 0;
    volatile uint32_t erase_count = 0;
    uint16_t mb_offset = 0;

    uint8_t manufacturer_id, device_id;
    /**
     * State Machine
     */

    state_t state = IDLE;

    // Initialize other variables
    bool trigger_received = false;
    bool can0_to_can1_complete = false;
    bool can1_to_emmc_complete = false;

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
    mss_config_clk_rst(MSS_PERIPH_EMMC, (uint8_t) 1, PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_CAN0, (uint8_t) 1, PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_CAN1, (uint8_t) 1, PERIPHERAL_ON);

    PLIC_DisableIRQ(CAN0_PLIC);
    PLIC_DisableIRQ(CAN1_PLIC);




    MSS_UART_init(g_uart,
            MSS_UART_115200_BAUD,
            MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

    MSS_UART_polled_tx_string(g_uart, (const uint8_t*)g_greeting_msg);

    /* Initialize the write and read Buffers */
    for (loop_count = 0; loop_count < (BUFFER_SIZE); loop_count++)
    {
        g_mmc_tx_buff[loop_count] = 0x45u + loop_count;
        g_mmc_rx_buff[loop_count] = 0x00u;
    }

        PLIC_init();
    __enable_irq();
        PLIC_SetPriority(MMC_main_PLIC, 2u);
    PLIC_SetPriority(MMC_wakeup_PLIC, 2u);

    /* DMA init for eMMC */
        loop_count = MSS_MPU_configure(MSS_MPU_MMC,
                                       MSS_MPU_PMP_REGION3,
                                       LIM_BASE_ADDRESS,
                                       LIM_SIZE,
                                       MPU_MODE_READ_ACCESS|MPU_MODE_WRITE_ACCESS|MPU_MODE_EXEC_ACCESS,
                                       MSS_MPU_AM_NAPOT,
                                       0u);

    /*--------------------------------------------------------------------------
         * Performs CAN Initialization and Message Buffer Configuration
         */
        /* ----------------------- CAN - 0 Initialization   ----------------- */
        init_return_value = MSS_CAN_init(g_mss_can_0, CAN_SPEED_8M_1M,
                                         (pmss_can_config_reg)0, 6u, 6u);
        MSS_CAN_set_mode(g_mss_can_0, CANOP_MODE_NORMAL);
        MSS_CAN_start(g_mss_can_0);

        /* ----------------------- CAN - 1 Initialization   ----------------- */
        init_return_value = MSS_CAN_init(g_mss_can_1, CAN_SPEED_8M_1M,
                                         (pmss_can_config_reg)0, 6u, 6u);
        MSS_CAN_set_mode(g_mss_can_1, CANOP_MODE_NORMAL);
        MSS_CAN_start(g_mss_can_1);

        /****************************SPI Initialization************************/
        FLASH_init();
        FLASH_global_unprotect();
        FLASH_read_device_id(&manufacturer_id, &device_id);

        // Erase flash memory
       /*erase_address = 0;
        for(erase_count = 0; erase_count <= 640; erase_count++) {
            FLASH_erase_64k_block(erase_address);
            delay1(500);
            FLASH_read(erase_address, g_read_buf, 32);
            erase_address += 0x10000;
        }
        write_flash_directory();*/
        // Read data from CAN and program it to the flash

        g_flash_address = flash_address[1];
        //g_flash_address = 0x500000;
        //g_flash_address = flash_image;
        uint32_t total_length = 0;
        /* Clear receive buffer */
        for (count = 0u; count < 8u; count++)
        {
            rx_buf.DATA[count] = 0u;
        }

        /* Configure for transmit */
        pmsg.ID = 0x78u;
        pmsg.DATALOW = 0xAAAAAAAAu;
        pmsg.DATAHIGH = 0x55555555u;
    #ifdef CAN_TX_EXTENDED_ID
        pmsg.L =((1<<20) | 0x00080000u); /* Extended ID, 8 bytes of data */
    #else
        pmsg.L = 0x00080000u;          /* Standard ID, 8 bytes of data */
    #endif

        /* Configure for receive */
        /* Initialize the rx mailbox */
        rx_msg.ID = 0x80u;
        rx_msg.DATAHIGH = 0u;
        rx_msg.DATALOW = 0u;
        rx_msg.AMR.L = 0xFFFFFFFFu;
        rx_msg.ACR.L = 0x00000000u;
        rx_msg.AMR_D = 0xFFFFFFFFu;
        rx_msg.ACR_D = 0x00000000u;
        rx_msg.RXB.DLC = 8u;
        rx_msg.RXB.IDE = 0u;

        /* Configure receive buffer For CAN 0 */
        ret_status = MSS_CAN_config_buffer_n(g_mss_can_0, 0, &rx_msg);
        if (CAN_OK != ret_status)
        {
            MSS_UART_polled_tx_string(g_uart,
                   (const uint8_t *)"\n\r CAN 0 Message Buffer configuration Error");
        }

        /* Configure receive buffer For CAN 1 */
        rx_msg.ID = 0x00;
        ret_status = MSS_CAN_config_buffer_n(g_mss_can_1, 1, &rx_msg);
        if (CAN_OK != ret_status)
        {
            MSS_UART_polled_tx_string(g_uart,
                   (const uint8_t *)"\n\r CAN 1 Message Buffer configuration Error");
        }


        //iap service
        /*uint32_t spi_idx = 0;
        uint8_t result = MSS_SYS_execute_iap(IAP_PROGRAM_BY_SPIADDR_CMD, flash_address[2]);
        uint8_t buff[20];
        sprintf((char *)buff, "%d", result);

        MSS_UART_polled_tx(g_uart, (uint8_t *)buff, sizeof(buff));*/

        //iap authentication
         //uint8_t address[sizeof(uint32_t)];
        //memcpy(address, flash_address[0], sizeof(uint32_t));

        //uint32_t spi_directory = 0x00000000;

        //FLASH_program(spi_directory, address, sizeof(address));
        /*uint32_t spi_idx = 0;
        uint16_t result = MSS_SYS_authenticate_iap_image(spi_idx);
        uint8_t buff[20];
        sprintf((char *)buff, "%d", result);

        MSS_UART_polled_tx(g_uart, (uint8_t *)buff, sizeof(buff));*/

        //bitstream authentication
        /*uint16_t result = MSS_SYS_authenticate_bitstream(IAP_IMAGE_SPI_ADDRESS, mb_offset);
        uint8_t buff[20];
        sprintf((char *)buff, "%d", result);

        MSS_UART_polled_tx(g_uart, (uint8_t *)buff, sizeof(buff));*/

        //read the data only
        /*uint8_t output[1024];  // Assuming a maximum size for the output buffer
        int index = 0;

        for(int l = 0; l < 5; ++l){
            FLASH_read(flash_address[1], job_read_buffer, 512);
            g_flash_address += BUFFER_SIZE;
            for (int i = 0; i < sizeof(job_read_buffer); ++i) {
                    // Use sprintf to convert each byte to a string and append it to the output buffer
                    index += sprintf(output + index, "%02X ", job_read_buffer[i]);

                    }
            MSS_UART_polled_tx(g_uart, output, sizeof(output));
        }*/


        //MSS_UART_polled_tx(g_uart, output, sizeof(output));
        /*while (1)
        {
            ret_status = MSS_CAN_get_message_n(g_mss_can_1, 1u, &rx_buf);

            if (CAN_VALID_MSG == ret_status)
                {
                // Copy CAN message data to buffer
                    uint32_t remaining_bytes = BUFFER_SIZE - g_bytes_received;
                    uint32_t bytes_to_copy = rx_buf.DLC < remaining_bytes ? rx_buf.DLC : remaining_bytes;

                    for (uint32_t loop_count = 0u; loop_count < bytes_to_copy; loop_count++)
                    {
                        g_mmc_tx_buff[g_head] = rx_buf.DATA[loop_count];
                        g_head = (g_head + 1) % BUFFER_SIZE;
                        if (g_head == g_tail)
                        {
                            // Buffer is full, move tail pointer to overwrite oldest data
                            g_tail = (g_tail + 1) % BUFFER_SIZE;
                        }
                    }

                    // Increment byte count by bytes copied
                    g_bytes_received += bytes_to_copy;

                    // If the buffer is full, post-process and reset the buffer
                    if (g_bytes_received >= BUFFER_SIZE)
                    {
                        FLASH_program(g_flash_address, g_mmc_tx_buff, 512);
                        //MSS_UART_polled_tx_string(g_uart,
                         //                  (const uint8_t *)"\n\r Written another buffer");
                         FLASH_read(g_flash_address, job_read_buffer, 512);
                        g_flash_address += BUFFER_SIZE;

                        uint8_t output[1024];  // Assuming a maximum size for the output buffer
                        int index = 0;

                        for (int i = 0; i < sizeof(job_read_buffer); ++i) {
                                // Use sprintf to convert each byte to a string and append it to the output buffer
                                index += sprintf(output + index, "%02X ", job_read_buffer[i]);
                                }


                        MSS_UART_polled_tx(g_uart, output, sizeof(output));

                        memset(g_mmc_tx_buff, 0, sizeof(g_mmc_tx_buff));
                        g_block_count++;
                        g_head = 0;
                        g_tail = 0;
                        g_bytes_received = 0;


                    }
                }
            }*/
}


void transfer_complete_handler(uint32_t status)
{
    uint32_t isr_err;

    if (ERROR_INTERRUPT & status)
    {
        isr_err = status >> 16u;
    }
    else if (TRANSFER_COMPLETE & status)
    {
        isr_err = 0u;   /*Transfer complete*/
    }
    else
    {
        ASSERT(0);
    }
}

static void mmc_reset_block(void)
{
    SYSREG->SUBBLK_CLOCK_CR |= (uint32_t)(SUBBLK_CLOCK_CR_MMC_MASK);
    SYSREG->SOFT_RESET_CR |= (uint32_t)(SOFT_RESET_CR_MMC_MASK);
    SYSREG->SOFT_RESET_CR &= ~(uint32_t)(SOFT_RESET_CR_MMC_MASK);
}

static int8_t mmc_init_emmc(void)
{
    mss_mmc_status_t ret_status;

    if (g_mmc_initialized == 1u)
    {
        ret_status = MSS_MMC_INIT_SUCCESS;
    }
    else
    {
        ASSERT(mss_does_xml_ver_support_switch() == true)

        if (switch_mssio_config(EMMC_MSSIO_CONFIGURATION) == false)
        {
            while(1u);
        }
        switch_external_mux(EMMC_MSSIO_CONFIGURATION);
        /* eMMC configuration */
        g_mmc.clk_rate = MSS_MMC_CLOCK_50MHZ;
        g_mmc.card_type = MSS_MMC_CARD_TYPE_MMC;
        g_mmc.bus_speed_mode = MSS_MMC_MODE_SDR;
        g_mmc.data_bus_width = MSS_MMC_DATA_WIDTH_8BIT;
        g_mmc.bus_voltage = MSS_MMC_1_8V_BUS_VOLTAGE;

        mmc_reset_block();

        ret_status = MSS_MMC_init(&g_mmc);
        if (ret_status == MSS_MMC_INIT_SUCCESS)
        {
            g_mmc_initialized = 1u;
        }
    }
    return ret_status;
}

static int8_t mmc_init_sdcard(void)
{
    mss_mmc_status_t ret_status;

    if (g_mmc_initialized == 2u)
    {
        ret_status = MSS_MMC_INIT_SUCCESS;
    }
    else
    {
        ASSERT(mss_does_xml_ver_support_switch() == true)

        if (switch_mssio_config(SD_MSSIO_CONFIGURATION) == false)
        {
            while(1u);
        }
        switch_external_mux(SD_MSSIO_CONFIGURATION);
        /* SD Card configuration */
        g_mmc.card_type = MSS_MMC_CARD_TYPE_SD;
        g_mmc.data_bus_width = MSS_MMC_DATA_WIDTH_4BIT;
        g_mmc.bus_speed_mode = MSS_SDCARD_MODE_HIGH_SPEED;
        g_mmc.clk_rate = MSS_MMC_CLOCK_50MHZ;

        mmc_reset_block();

        ret_status = MSS_MMC_init(&g_mmc);
        if (ret_status == MSS_MMC_INIT_SUCCESS)
        {
            g_mmc_initialized = 2u;
        }
    }
    return ret_status;
}

static int8_t single_block_wr_rd(uint32_t sector_number)
{
    int8_t status = 0u;
    /* Erase single block */
    status = MSS_MMC_erase(sector_number, 1u);
    if (status == MSS_MMC_TRANSFER_SUCCESS)
    {
        /* Single Block - write */
        status = MSS_MMC_single_block_write((uint32_t *)g_mmc_tx_buff, sector_number);
        if (status == MSS_MMC_TRANSFER_SUCCESS)
        {
            /* Single Block - read */
            status = MSS_MMC_single_block_read(sector_number, (uint32_t *)g_mmc_rx_buff);
            if (status == MSS_MMC_TRANSFER_SUCCESS)
            {
                status = verify_write(g_mmc_tx_buff, g_mmc_rx_buff, BLOCK_SIZE); //single block
            }
        }
    }

    return (status);
}

static int8_t multi_block_transfer(uint32_t sector_number)
{
    int8_t status = 0u;
    /* Erase multi block */
    status = MSS_MMC_erase(sector_number, BUFFER_SIZE/BLOCK_SIZE);
    if (status == MSS_MMC_TRANSFER_SUCCESS)
    {
        MSS_MMC_set_handler(transfer_complete_handler);

        status = multi_block_wr_rd(sector_number, USE_SDMA);
        if (status == MSS_MMC_TRANSFER_SUCCESS)
        {
            status = multi_block_wr_rd(sector_number, USE_ADMA2);
        }
    }

    return (status);
}

static int8_t multi_block_wr_rd(uint32_t sector_number, uint8_t dma_type)
{
    int8_t status = 0u;

    if (USE_ADMA2 == dma_type)
    {
        status =  MSS_MMC_adma2_write(g_mmc_tx_buff, sector_number, BUFFER_SIZE);
    }
    else if (USE_SDMA == dma_type)
    {
        status =  MSS_MMC_sdma_write(g_mmc_tx_buff, sector_number, BUFFER_SIZE);
    }
    else
    {
        status = -1;
    }

    if (status == MSS_MMC_TRANSFER_IN_PROGRESS)
    {
        do
        {
            status =  MSS_MMC_get_transfer_status();
        }while (status == MSS_MMC_TRANSFER_IN_PROGRESS);
    }

    if (status == MSS_MMC_TRANSFER_SUCCESS)
    {
        if (USE_ADMA2 == dma_type)
        {
            status =  MSS_MMC_adma2_read(sector_number, g_mmc_rx_buff, BUFFER_SIZE);
        }
        else if (USE_SDMA == dma_type)
        {
            status =  MSS_MMC_sdma_read(sector_number, g_mmc_rx_buff, BUFFER_SIZE);
        }
        else
        {
            status = -1;
        }

        if (status == MSS_MMC_TRANSFER_IN_PROGRESS)
        {
            do
            {
                status = MSS_MMC_get_transfer_status();
            }while (status == MSS_MMC_TRANSFER_IN_PROGRESS);
        }

        if (status == MSS_MMC_TRANSFER_SUCCESS)
        {
            status = verify_write(g_mmc_tx_buff, g_mmc_rx_buff, BUFFER_SIZE);
        }
    }

    return (status);
}

static int8_t verify_write(uint8_t* write_buff, uint8_t* read_buff, uint32_t size)
{
    int8_t error = 0u;
    uint32_t index = 0u;
    uint8_t p_buff[100];

    while (size != 0u)
    {
        sprintf(p_buff,"\r\n write_buff[%d]  = 0x%02x", index, write_buff[index] );
        MSS_UART_polled_tx(g_uart, p_buff, strlen(p_buff));

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

/* hart1 software interrupt handler */
void Software_h1_IRQHandler(void)
{
    uint64_t hart_id = read_csr(mhartid);
    count_sw_ints_h1++;
}

static void report_status(int8_t status)
{
    if(0 == status)
    {
        MSS_UART_polled_tx_string(g_uart,
                (const uint8_t*)"\r\nTransfer successful \r\n");
    }
    else
    {
        MSS_UART_polled_tx_string(g_uart,
                (const uint8_t*)"\r\nTransfer failed \r\n");
    }
}



static int8_t multi_block_read_transfer(uint32_t sector_number) {
    int8_t status = 0u;

    MSS_MMC_set_handler(transfer_complete_handler);

    status = multi_block_read(sector_number, USE_ADMA2);
    if (status == MSS_MMC_TRANSFER_SUCCESS) {
        MSS_UART_polled_tx_string(g_uart,
                      (const uint8_t*)"\r\n USE_ADMA2 Transfer successful \r\n");
        status = multi_block_read(sector_number, USE_SDMA);
        if (status == MSS_MMC_TRANSFER_SUCCESS) {
            MSS_UART_polled_tx_string(g_uart,
                                  (const uint8_t*)"\r\n USE_SDMA Transfer successful \r\n");
               status=0;
           }
    }



    MSS_MMC_set_handler(transfer_complete_handler);


    return status;
}

static int8_t multi_block_write_transfer(uint32_t sector_number) {
    int8_t status = 0u;

    MSS_MMC_set_handler(transfer_complete_handler);


    status = multi_block_write(sector_number, USE_ADMA2);
    if (status == MSS_MMC_TRANSFER_SUCCESS) {
        MSS_UART_polled_tx_string(g_uart,
                      (const uint8_t*)"\r\n USE_ADMA2 Transfer successful \r\n");
        status = multi_block_write(sector_number, USE_SDMA);
        if (status == MSS_MMC_TRANSFER_SUCCESS) {
            MSS_UART_polled_tx_string(g_uart,
                                  (const uint8_t*)"\r\n USE_SDMA Transfer successful \r\n");
               status=0;
           }
    }



    MSS_MMC_set_handler(transfer_complete_handler);


    return status;
}

static int8_t multi_block_read(uint32_t sector_number, uint8_t dma_type) {
    int8_t status = 0u;

    if (USE_ADMA2 == dma_type) {
        status = MSS_MMC_adma2_read(sector_number, g_mmc_rx_buff, BUFFER_SIZE);
    } else if (USE_SDMA == dma_type) {
        status = MSS_MMC_sdma_read(sector_number, g_mmc_rx_buff, BUFFER_SIZE);
    } else {
        status = -1;
    }

    if (status == MSS_MMC_TRANSFER_IN_PROGRESS) {
        do {
            status = MSS_MMC_get_transfer_status();
        } while (status == MSS_MMC_TRANSFER_IN_PROGRESS);
    }

   // verify_write(g_mmc_tx_buff, g_mmc_rx_buff, BUFFER_SIZE);
    /*
    if (status == MSS_MMC_TRANSFER_SUCCESS) {
        status = verify_write(g_mmc_tx_buff, g_mmc_rx_buff, BUFFER_SIZE);
    }*/

    return status;
}
static int8_t multi_block_write(uint32_t sector_number, uint8_t dma_type) {
    int8_t status = 0u;
    uint8_t p_buff[128];



    if (USE_ADMA2 == dma_type) {
        status = MSS_MMC_adma2_write(g_mmc_tx_buff, sector_number, BUFFER_SIZE);
    } else if (USE_SDMA == dma_type) {
        status = MSS_MMC_sdma_write(g_mmc_tx_buff, sector_number, BUFFER_SIZE);
    } else {
        status = -1;
    }

    if (status == MSS_MMC_TRANSFER_IN_PROGRESS) {
        do {
            status = MSS_MMC_get_transfer_status();
        } while (status == MSS_MMC_TRANSFER_IN_PROGRESS);
    }
   /*  verify_write(g_mmc_tx_buff, g_mmc_rx_buff, BUFFER_SIZE);
        if (status == MSS_MMC_TRANSFER_SUCCESS) {
            status = verify_write(g_mmc_tx_buff, g_mmc_rx_buff, BUFFER_SIZE);
        }
    */

    return status;
}


static int8_t single_block_read(uint32_t sector_number) {
    int8_t status = 0u;
  //  status = MSS_MMC_erase(sector_number, 1u);

    /* Single Block - read */

    status = MSS_MMC_single_block_read(sector_number, (uint32_t *)g_mmc_rx_buff);
    if (status == MSS_MMC_TRANSFER_SUCCESS) {

        //MSS_UART_polled_tx_string(g_uart,(const uint8_t*)"\r\SD CARD Single BLOCK PASS\r\n");
        status = 0u;
    }

   // status = verify_write(g_mmc_tx_buff, g_mmc_rx_buff, BLOCK_SIZE); // single block
    return (status);
}

static void display_hex_values
(
    const uint8_t * in_buffer,
    uint32_t byte_length
)
{
    uint8_t display_buffer[128];
    uint32_t inc;

    if (0u == byte_length)
    {
        MSS_UART_polled_tx_string(g_uart, (const uint8_t*)" <No data present>\n\r");
    }
    else
    {
        if (byte_length > 16u)
        {
            MSS_UART_polled_tx_string(g_uart, (const uint8_t*)"\n\r ");
        }

        for (inc = 0u; inc < byte_length; ++inc)
        {
            if ((inc > 1u) && (0u == (inc % 16u)))
            {
                MSS_UART_polled_tx_string(g_uart, (const uint8_t*)"\n\r");
            }

            uint32_t value = in_buffer[inc];

            // swap the byte order of each 4-byte block
            if (inc % 4 == 0)
            {
                value = ((value & 0x000000FF) << 24) |
                        ((value & 0x0000FF00) << 8) |
                        ((value & 0x00FF0000) >> 8) |
                        ((value & 0xFF000000) >> 24);
            }

            snprintf((char *)display_buffer, sizeof(display_buffer), "%02x ",
                    (uint8_t)value);
            MSS_UART_polled_tx_string(g_uart, display_buffer);
        }
    }
}

static void can1_check_rx_buffer(void)
{
    uint8_t loop_count;

    /*----------------------------------------------------------------------
      Read the Data from CAN channel and Transmit Through UART
     */
    if (CAN_VALID_MSG == MSS_CAN_get_message_n(g_mss_can_1, 1u, &rx_buf))
    {

        for (loop_count = 0u; loop_count < rx_buf.DLC; loop_count++)
        {
            if (loop_count < 4u)
            {
                g_can_to_uart[loop_count] = rx_buf.DATA[3u - loop_count];
            }
            else
            {
                g_can_to_uart[loop_count] = rx_buf.DATA[11u - loop_count];
            }
        }

        MSS_UART_polled_tx_string(g_uart, (const uint8_t *)"\n\r Data Received "
                "as CAN 1 Message is ");
        MSS_UART_polled_tx_string(g_uart, (const uint8_t*)"\n\r ");

        /* Send to UART */
        display_hex_values(g_can_to_uart,rx_buf.DLC);
        MSS_UART_polled_tx_string(g_uart, (const uint8_t *)"\n\r Observe the"
                "message sent from the CAN 0 ");
        MSS_UART_polled_tx_string(g_uart, (const uint8_t *)"\n\r It should be "
                "same as message Received on Hyperterminal");

    }
}

void circular_buffer_add(uint8_t* data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
        {
            circular_buffer[write_pos] = data[i];
            write_pos = (write_pos + 1) % BUFFER_SIZE;

            // add the data item to the array
            if (num_items < BUFFER_SIZE)
            {
                data_array[num_items] = data[i];
                num_items++;
            }
        }
    }




