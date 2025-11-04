
/*******************************************************************************
 * Copyright 2019-2021 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * Application code running on U54_2
 *
 * PolarFire SoC MSS GPIO interrupt example project
 */


/// Some snippets taken from here
/// https://github.com/polarfire-soc/polarfire-soc-bare-metal-examples/tree/main/driver-examples/fpga-ip/CoreAXI4DMAController/mpfs-coreaxi4dma-stream
///

#include <stdio.h>
#include <string.h>

#include <mpfs_hal/mss_hal.h>
#include <drivers/mss/mss_mmuart/mss_uart.h>

#include <drivers/fpga_ip/CoreAXI4DMAController/core_axi4dmacontroller.h>

#include <inc/cond_var.h>

/// CoreAXI4 DMA Controller Interrupt Bit Masks
/// For full details see Handbook section 5.3
///
#define DMA_OPERATION_STATUS_BIT_MASK           (0xFu)
#define DMA_DESCRIPTOR_NUM_BIT_MASK             (0x3F0u)
#define IR_STATUS_DESCRIPTOR_NUM_BIT_SHIFT      (0x4u)
#define RETURN_DESCRIPTOR_INTERRUPT_NUMBER(X)   (((X)&DMA_DESCRIPTOR_NUM_BIT_MASK) >> IR_STATUS_DESCRIPTOR_NUM_BIT_SHIFT)

/// Defines to configure the memory peripheral used as the stream destination
///
#define DDR_CACHED      1

/// #define DDR_NCACHED 1

/// Using SRAM requires setting the 'BFM_SIMULATION' flag when generating the Libero design
///
/// #define SRAM        1

/// Turn on to display pixel comparation in the pattern mode
///
/// #define DEBUG_PRINT 1U


/// Change these when the resolution changes
///
#define NUM_COLUMNS     2048
#define NUM_ROWS        2048

/// Defines to configure the number of words / bytes to transfer
/// Pixels are packet as (PIX[i+1] << 16) | pix[i])
/// The first pixel contains the frame counter
/// The second pixel contains the Line number
///
#define STREAM_SIZE         ((NUM_ROWS * NUM_COLUMNS / 2))
#define STREAM_SIZE_BYTES   (STREAM_SIZE * 4)

/// Defining memory addresses
///
#define STREAM_GEN_BASE_ADDR    0x40000000u
#define SRAM_BASE_ADDR          0x61000000u
#define DDR_BASE_ADDR           0x80000000u
#define DDR_NCACHED_BASE_ADDR   0xC0000000u
#define DMA_CTRLR_BASE_ADDR     0x60020000u
#define STREAM_OFFSET           0x1000u

/// Stream configuration bit defines
///
#define STREAM_DEST_OPERAND          (0x0001u << 0u)
#define STREAM_DEST_DATA_READY       (0x0001u << 2u)
#define STREAM_DESCRIPTOR_VALID      (0x0001u << 3u)


/// CoreAXI4DMAController instance at base address 0x60020000u
///
axi4dma_instance_t                  g_dmac;
static      uint8_t                 g_info_string[200] = {0};

typedef enum
{
    STREAM_INCOMPLETE,
    STREAM_TRANSFER_COMPLETE,
    BLOCK_TRANSFER_COMPLETE,
    TRANSFER_ERROR
} transfer_status_t;
static      transfer_status_t       g_stream_status = STREAM_INCOMPLETE;
extern volatile struct cond_var_t   g_cond_var;
axi4dma_stream_desc_t*              g_tdest0_stream_descriptor;
uint64_t                            g_frame_addr;

/// pointer to location in DDR where stream will written to
///
volatile uint32_t *DDR_data_ptr;
volatile uint64_t DDR_data_addr;

typedef enum
{
    NO_ERROR,
    UNKNOWN_DESCRIPTOR_TRANSFER_COMPLETE,
    WRITE_ERROR,
    READ_ERROR,
    INVALID_DESCRIPTOR,
    UNKNOWN_ERROR
} dma_error_status_t;

static volatile dma_error_status_t  g_dma_transfer_error_type = NO_ERROR;

void
report_transfer_error(void)
{
    switch (g_dma_transfer_error_type)
    {
        case UNKNOWN_DESCRIPTOR_TRANSFER_COMPLETE:
            MSS_UART_polled_tx_string(&g_mss_uart1_lo,
                                      "\r\n Error!\r\nTransfer Complete set from unknown"
                                      " descirptor\r\n");
            break;

        case WRITE_ERROR:
            MSS_UART_polled_tx_string(&g_mss_uart1_lo, "\r\n Error!\r\nDMA Write Error\r\n");
            break;

        case READ_ERROR:
            MSS_UART_polled_tx_string(&g_mss_uart1_lo, "\r\n Error!\r\nDMA Read Error\r\n");
            break;

        case INVALID_DESCRIPTOR:
            MSS_UART_polled_tx_string(&g_mss_uart1_lo, "\r\n Error!\r\nDMA Invalid Descirptor\r\n");
            break;

        case UNKNOWN_ERROR:
            MSS_UART_polled_tx_string(&g_mss_uart1_lo, "\r\n Error!\r\nDMA Unknown Error\r\n");
            break;

        default:
            MSS_UART_polled_tx_string(&g_mss_uart1_lo, "\r\n Error!\r\nDMA Unknown Error\r\n");
            break;
    }
    g_dma_transfer_error_type = NO_ERROR;
}

/// The CoreAXI4DMAController Interrupt 0 via F2H interrupt.
///
uint8_t
PLIC_f2m_2_IRQHandler(void)
{
    axi4dma_desc_id_t dma_descriptor_id;
    uint32_t external_descriptor_address;
    uint32_t dma_transfer_status = 0u;
    uint32_t descriptor = 0u;

    uint32_t irq_status = AXI4DMA_transfer_status(&g_dmac,
                                                  IRQ_NUM_0,
                                                  &dma_descriptor_id,
                                                  &external_descriptor_address);

    dma_transfer_status = irq_status & DMA_OPERATION_STATUS_BIT_MASK;
    descriptor = RETURN_DESCRIPTOR_INTERRUPT_NUMBER(irq_status);

    switch (dma_transfer_status)
    {
        case AXI4DMA_OP_COMPLETE_INTR_MASK:
            switch (descriptor)
            {
                case STREAM_DESC_33:
                    g_stream_status = STREAM_TRANSFER_COMPLETE;
                    break;

                case INTRN_DESC_0:
                    g_stream_status = BLOCK_TRANSFER_COMPLETE;
                    break;

                default:
                    g_stream_status = TRANSFER_ERROR;
                    g_dma_transfer_error_type = UNKNOWN_DESCRIPTOR_TRANSFER_COMPLETE;
                    break;
            }
            break;

        case AXI4DMA_WR_ERR_INTR_MASK:
            g_stream_status = TRANSFER_ERROR;
            g_dma_transfer_error_type = WRITE_ERROR;
            break;

        case AXI4DMA_RD_ERR_INTR_MASK:
            g_stream_status = TRANSFER_ERROR;
            g_dma_transfer_error_type = READ_ERROR;
            break;

        case AXI4DMA_INVALID_DESC_INTR_MASK:
            g_stream_status = TRANSFER_ERROR;
            g_dma_transfer_error_type = INVALID_DESCRIPTOR;
            break;

        default:
            g_stream_status = TRANSFER_ERROR;
            g_dma_transfer_error_type = UNKNOWN_ERROR;
            break;
    }

    uint8_t error_message[50] = {0};
    sprintf(error_message, " > Transfer Status: %i\r\n", dma_transfer_status);
    MSS_UART_polled_tx_string(&g_mss_uart1_lo, error_message);

    AXI4DMA_clear_irq(&g_dmac,
                      IRQ_NUM_0,
                      AXI4DMA_OP_COMPLETE_INTR_MASK | AXI4DMA_WR_ERR_INTR_MASK |
                      AXI4DMA_RD_ERR_INTR_MASK | AXI4DMA_INVALID_DESC_INTR_MASK);

    /// Set the stream transfer to destination memory address
    ///
    if (g_stream_status == STREAM_TRANSFER_COMPLETE)
    AXI4DMA_configure_stream(&g_dmac,
                              g_tdest0_stream_descriptor,
                              TDEST_0,
                              STREAM_DEST_OPERAND | STREAM_DEST_DATA_READY | STREAM_DESCRIPTOR_VALID,
                              STREAM_SIZE_BYTES,
                              g_frame_addr);


    return EXT_IRQ_KEEP_ENABLED;
}

void u54_1(void)
{
    uint32_t *stream_ctrl;      /// pointer to stream controller
    uint32_t *stream_ctrl_C;    /// exposure register

#ifdef DDR_CACHED
    ///  use cached DDR as the stream memory destination
    ///
    uint64_t ddr_loc = DDR_BASE_ADDR;
#elif defined(DDR_NCACHED)
    /// use non-cached DDR as the stream memory destination
    ///
    uint32_t ddr_loc = DDR_NCACHED_BASE_ADDRu;

#elif defined(SRAM)
    /// use LSRAM as the stream memory destination
    ///
    uint32_t ddr_loc = SRAM_BASE_ADDR;

#else
    /// use cached DDR as the default memory destination
    ///
    uint32_t ddr_loc = DDR_BASE_ADDR;
#endif

    g_frame_addr = ddr_loc + STREAM_OFFSET;

    cond_var_init(&g_cond_var);

    g_tdest0_stream_descriptor = (axi4dma_stream_desc_t*)(uint64_t)ddr_loc;


    /* Clear pending software interrupt in case there was any.
     * Enable only the software interrupt so that the E51 core can bring this
     * core out of WFI by raising a software interrupt In case of external,
     * bootloader not present
     */

    clear_soft_interrupt();
    set_csr(mie, MIP_MSIP);

#if (IMAGE_LOADED_BY_BOOTLOADER == 0)

    /*Put this hart into WFI.*/

    do
    {
        __asm("wfi");
    }while(0 == (read_csr(mip) & MIP_MSIP));

    /* The hart is out of WFI, clear the SW interrupt. Hear onwards Application
     * can enable and use any interrupts as required */
    clear_soft_interrupt();
#endif

    PLIC_init();
    __enable_irq();

    /* Reset the peripherals turn on the clocks */

    (void)mss_config_clk_rst(MSS_PERIPH_MMUART1, (uint8_t) MPFS_HAL_FIRST_HART, PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_GPIO2, (uint8_t) MPFS_HAL_FIRST_HART, PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_FIC0, (uint8_t) MPFS_HAL_FIRST_HART, PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_FIC3, (uint8_t) MPFS_HAL_FIRST_HART, PERIPHERAL_ON);

    mss_enable_fabric();

    MSS_UART_init( &g_mss_uart1_lo,
            MSS_UART_115200_BAUD,
            MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);


    PLIC_SetPriority(FABRIC_F2H_2_PLIC, 2);
    PLIC_EnableIRQ(FABRIC_F2H_2_PLIC);

    /// Reset the module
    ///
    /// stream_ctrl = (void *)  (STREAM_GEN_BASE_ADDR + 0x8);
    /// *stream_ctrl = 0x1u;

    /// configure stream size
    ///
    stream_ctrl = (void *) STREAM_GEN_BASE_ADDR; /// address of size register
    *stream_ctrl = STREAM_SIZE; /// set size based on define


    /// CoreAXI4DMAController IP base address in libero design = 0x60020000u
    ///
    AXI4DMA_init(&g_dmac, DMA_CTRLR_BASE_ADDR);

    /// Set the stream transfer to destination memory address
    ///
    AXI4DMA_configure_stream(&g_dmac,
                              g_tdest0_stream_descriptor,
                              TDEST_0,
                              STREAM_DEST_OPERAND | STREAM_DEST_DATA_READY | STREAM_DESCRIPTOR_VALID,
                              STREAM_SIZE_BYTES,
                              g_frame_addr);

    /// The stream descriptor is associated with IRQ0 in the IP
    ///
    AXI4DMA_enable_irq(&g_dmac, IRQ_NUM_0, AXI4DMA_OP_COMPLETE_INTR_MASK |
                                           AXI4DMA_WR_ERR_INTR_MASK |
                                           AXI4DMA_RD_ERR_INTR_MASK  |
                                           AXI4DMA_INVALID_DESC_INTR_MASK );

    /// Initiating stream by writing to the  register on the generator module
    ///
    stream_ctrl = (void *)  (STREAM_GEN_BASE_ADDR + 0x4);

    /// Start streaming and set the exposure
    /// bit 0 - Start
    /// bit 1 - Pattern
    /// bit 2 - Pattern thru sensor lane
    /// bit 5:4 - High gain 00 Low Gain 01 HDR 10
    ///
    *stream_ctrl = 0x1 | (0x01 << 4);

    /// wait for stream to complete
    ///
    if (g_stream_status == STREAM_INCOMPLETE) {
        MSS_UART_polled_tx_string(&g_mss_uart1_lo, "\r\n > Waiting for transfer to complete");
    }


    /// Wait for the interrupt to raise it
    ///
    while (STREAM_INCOMPLETE == g_stream_status)
    {
       asm volatile("nop");
    }

    /// Exposure
    ///
    stream_ctrl_C = (void *)  (STREAM_GEN_BASE_ADDR + 0xC);
    *stream_ctrl_C = 10;


    /// verify data written matches expected
    ///
    volatile uint8_t fail = 0;

    MSS_UART_polled_tx_string(&g_mss_uart1_lo, "\r\n > Verifying DATA\n\r\n\r");


    uint32_t pixel_pattern = 0u;
    uint16_t pixel_counter = 0u;
    uint16_t frame_counter = 0u;
    uint8_t  first_frame = 1u;
    uint16_t our_frame_counter = 0u;
    uint64_t pattern_count = 0u;

    DDR_data_addr =  ddr_loc + STREAM_OFFSET;
    DDR_data_ptr = (void*)(ddr_loc + STREAM_OFFSET); /// Stream destination address
    while (1)
    {
        fail = 0u;

        DDR_data_ptr = (void*)(ddr_loc + STREAM_OFFSET); /// Stream destination address
        DDR_data_addr = (uint64_t)(DDR_data_ptr);

        cond_var_signal(&g_cond_var);

        /// The sent frame counter is encoded in the 2nd pixel
        /// which is part of the word received
        /// At the beginning of each line
        ///
        volatile uint16_t received_frame_counter = *(DDR_data_ptr + 0x0) & 0xFFFF;

        sprintf(g_info_string, " > Fm %4d | Rcvd Fm %4d | WORD[0] 0x%08X\r\n", frame_counter, received_frame_counter, *DDR_data_ptr);
        MSS_UART_polled_tx_string(&g_mss_uart1_lo, g_info_string);

        if (first_frame == 1u)
        {
            frame_counter = received_frame_counter;
            first_frame = 0u;
        }
        else
            ++frame_counter;

        *stream_ctrl_C = (++our_frame_counter * 50) % 200; /// for testing
        our_frame_counter = our_frame_counter % 4;

        /// Line number starts from 0
        ///
        for(volatile uint32_t y = 0; y < NUM_ROWS; ++y)
        {

            pixel_counter = 2u;
            for(volatile uint32_t x = 0; x < NUM_COLUMNS/2; ++x)
            {
                volatile uint32_t texel = *DDR_data_ptr;

                if (pattern_count++ >= STREAM_SIZE)
                {
                    pattern_count = 0u;
                }

                /// The second pixel of each line beginning is the Line number
                ///
                if (x == 0)
                    pixel_pattern = (y << 16) | (frame_counter);
                else
                    pixel_pattern = (pixel_counter++) | ((pixel_counter++) << 16);

                if (pixel_pattern != texel)
                {
#ifdef DEBUG_PRINT
                    sprintf(g_info_string, " > Ln %4d | Fm %4d | exp 0x%08X rd 0x%08X (match failed)    \r\n", y, frame_counter, pixel_pattern, texel);
                    MSS_UART_polled_tx_string(&g_mss_uart1_lo, g_info_string);
#endif
                    fail = 1;
                } else
                {
#ifdef DEBUG_PRINT
                    sprintf(g_info_string, " > Ln %4d | Fm %4d | exp 0x%08X rd 0x%08X (match successful)\r\n", y, frame_counter, pixel_pattern, texel);
                    MSS_UART_polled_tx_string(&g_mss_uart1_lo, g_info_string);
#endif
                }

                DDR_data_ptr +=1;
            }
        }

        if (fail == 0)
        {
            sprintf(g_info_string, " >> Fm %08d | Stream data match is successful       \r\n", frame_counter);
            MSS_UART_polled_tx_string(&g_mss_uart1_lo, g_info_string);
        }
        else
        {
            if (received_frame_counter != frame_counter)
            {
                /// first_frame = 1u;   /// Reset over after reporting the error
                sprintf(g_info_string, " >> Fm %08d | Rcvd Fm %08d match failed             \r\n", frame_counter, received_frame_counter);
                MSS_UART_polled_tx_string(&g_mss_uart1_lo, g_info_string);
            }
            sprintf(g_info_string, " >> Fm %08d | Stream data match failed                  \r\n", frame_counter);
            MSS_UART_polled_tx_string(&g_mss_uart1_lo, g_info_string);
        }

        cond_var_wait(&g_cond_var);

     }

    while (1u)
    {
        asm volatile ("nop");
    }
}

