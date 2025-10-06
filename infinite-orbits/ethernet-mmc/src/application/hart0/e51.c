/*******************************************************************************
 * @file    e51.c
 * @brief   Ethernet-to-eMMC receiver for Icicle Kit (PolarFire SoC)
 *
 *          - Initializes Ethernet MAC (RX only)
 *          - Receives fixed-size Ethernet frames (payload = 512B + 4B sequence)
 *          - Writes payloads sequentially to eMMC
 *          - Detects packet losses based on sequence counter
 *
 * @note    Requires: MSS_MAC, MSS_MMC, MSS_UART drivers.
 ******************************************************************************/

 #include <stdio.h>
 #include <string.h>
 #include "mpfs_hal/mss_hal.h"
 #include "mpfs_hal/common/nwc/mss_nwc_init.h"
 
 #include "drivers/mss/mss_gpio/mss_gpio.h"
 #include "drivers/mss/mss_mmuart/mss_uart.h"
 #include "drivers/mss/mss_mmc/mss_mmc.h"
 #include "../platform/mpfs_hal/common/nwc/mss_io_config.h"
 
 #include "drivers/mss/mss_ethernet_mac/mss_ethernet_registers.h"
 #include "drivers/mss/mss_ethernet_mac/mss_ethernet_mac_sw_cfg.h"
 #include "drivers/mss/mss_ethernet_mac/mss_ethernet_mac_regs.h"
 #include "drivers/mss/mss_ethernet_mac/mss_ethernet_mac.h"
 #include "drivers/mss/mss_ethernet_mac/phy.h"
 #include "inc/common.h"
 
 /* -------------------------------------------------------------------------
  *  UART and utility macros
  * ------------------------------------------------------------------------- */
 #define DEMO_UART       &g_mss_uart0_lo
 #define PRINT_STRING(x) MSS_UART_polled_tx_string(DEMO_UART, (uint8_t *)(x))
 
 /* -------------------------------------------------------------------------
  *  Memory, MMC, and system configuration
  * ------------------------------------------------------------------------- */
 #define LIM_BASE_ADDRESS        0x08000000u
 #define LIM_SIZE                0x200000u
 #define ERROR_INTERRUPT         0x8000u
 #define TRANSFER_COMPLETE       0x1u
 #define USE_SDMA                0u
 #define USE_ADMA2               1u
 #define BLOCK_SIZE_BYTES        512
 #define SLOT_SIZE_BYTES         (100 * 1024 * 1024)  /* 100 MB */
 #define STREAM_GEN_BASE_ADDR    0x4A000000u
 #define START_ADDRESS           0u
 
 /* -------------------------------------------------------------------------
  *  Global buffers and state
  * ------------------------------------------------------------------------- */
 mss_mmc_cfg_t g_mmc;
 uint8_t g_mmc_initialized = 0u;
 uint8_t g_mmc_rx_buff[BLOCK_SIZE_BYTES] = {0};
 uint8_t g_mmc_tx_buff[BLOCK_SIZE_BYTES];
 uint32_t g_block_count = 0u;
 char message[128];
 
 /* Ethernet RX buffers */
 static uint8_t g_mac_rx_buffer[MSS_MAC_RX_RING_SIZE][MSS_MAC_MAX_RX_BUF_SIZE]
     __attribute__((aligned(16)));
 
 /* Global MAC configuration */
 mss_mac_cfg_t g_mac_config;
 mss_mac_instance_t *g_mac = &g_mac0;
 
 /* TX buffer (unused in RX mode) */
 uint8_t tx_buffer[1500];
 uint32_t length;
 
 /* -------------------------------------------------------------------------
  *  Function prototypes
  * ------------------------------------------------------------------------- */
 static void mac_rx_callback(void *this_mac, uint32_t queue_no,
                             uint8_t *p_rx_packet, uint32_t pckt_length,
                             mss_mac_rx_desc_t *cdesc, void *caller_info);
 
 static void write_payload_to_emmc(const uint8_t *payload, uint32_t payload_len);
 static mss_mmc_status_t mmc_init_emmc(void);
 static void process_ethernet_payload(uint8_t *packet, uint32_t packet_len);
 static void ethernet_init(void);
 static void mmc_reset_block(void);
 
 static mss_mmc_status_t multi_block_write_transfer(uint32_t sector_number);
 static mss_mmc_status_t multi_block_write(uint32_t sector_number, uint8_t dma_type);
 void transfer_complete_handler(uint32_t status);
 /*******************************************************************************
  * @brief  TX complete callback (currently unused)
  ******************************************************************************/
 static void packet_tx_complete_handler(void *this_mac,
                                        uint32_t queue_no,
                                        mss_mac_tx_desc_t *cdesc,
                                        void *caller_info)
 {
     (void)this_mac;
     (void)queue_no;
     (void)cdesc;
     (void)caller_info;
 }
 
 /*******************************************************************************
  * @brief  Initialize Ethernet MAC for reception
  ******************************************************************************/
 static void ethernet_init(void)
 {
     MSS_MAC_cfg_struct_def_init(&g_mac_config);
 
     /* Basic MAC configuration */
     g_mac_config.speed_duplex_select = MSS_MAC_ANEG_ALL_SPEEDS;
     g_mac_config.mac_addr[0] = 0x00;
     g_mac_config.mac_addr[1] = 0xFC;
     g_mac_config.mac_addr[2] = 0x00;
     g_mac_config.mac_addr[3] = 0x12;
     g_mac_config.mac_addr[4] = 0x34;
     g_mac_config.mac_addr[5] = 0x56;
 
     /* PHY configuration */
     g_mac_config.phy_addr              = PHY_VSC8662_0_MDIO_ADDR;
     g_mac_config.phy_type              = MSS_MAC_DEV_PHY_VSC8662;
     g_mac_config.interface_type        = TBI;
     g_mac_config.phy_autonegotiate     = MSS_MAC_VSC8662_phy_autonegotiate;
     g_mac_config.phy_mac_autonegotiate = MSS_MAC_VSC8662_mac_autonegotiate;
     g_mac_config.phy_get_link_status   = MSS_MAC_VSC8662_phy_get_link_status;
     g_mac_config.phy_init              = MSS_MAC_VSC8662_phy_init;
     g_mac_config.phy_set_link_speed    = MSS_MAC_VSC8662_phy_set_link_speed;
 
     MSS_MAC_init(g_mac, &g_mac_config);
     MSS_MAC_set_tx_callback(g_mac, 0, packet_tx_complete_handler);
     MSS_MAC_set_rx_callback(g_mac, 0, mac_rx_callback);
 
     /* Register RX buffers */
     for (uint32_t i = 0; i < MSS_MAC_RX_RING_SIZE; i++)
     {
         MSS_MAC_receive_pkt(g_mac, 0, g_mac_rx_buffer[i], 0,
             (i == MSS_MAC_RX_RING_SIZE - 1) ? -1 : 0);
     }
 
     /* Start PHY autonegotiation */
     g_mac->phy_mac_autonegotiate((const void *)g_mac);
 }
 
 /*******************************************************************************
  * @brief  Process Ethernet payload (extract sequence + 512B data)
  ******************************************************************************/
 static void process_ethernet_payload(uint8_t *packet, uint32_t packet_len)
 {
     if (packet_len < 14u)
         return;
 
     /* Skip 14-byte Ethernet header (6 dst + 6 src + 2 type) */
     uint8_t *payload = packet + 14u;
     uint32_t payload_len = packet_len - 14u;
 
     if (payload_len < 4u)
         return;
 
     /* Extract 4-byte big-endian sequence ID */
     uint32_t seq_id = ((uint32_t)payload[0] << 24) |
                       ((uint32_t)payload[1] << 16) |
                       ((uint32_t)payload[2] << 8)  |
                        (uint32_t)payload[3];
 
     uint8_t *data = payload + 4u;
     uint32_t data_len = payload_len - 4u;
 
     /* Sequence tracking */
     static uint32_t expected_seq = 0u;
     static uint8_t first_packet = 1u;
 
     if (first_packet)
     {
         expected_seq = seq_id;
         first_packet = 0u;
         PRINT_STRING("First frame received\n\r");
     }
     else if (seq_id != expected_seq)
     {
         char warn[128];
         snprintf(warn, sizeof(warn),
                  "[WARN] Sequence mismatch: expected %lu, got %lu (lost %ld)\n\r",
                  (unsigned long)expected_seq,
                  (unsigned long)seq_id,
                  (long)(seq_id - expected_seq));
         PRINT_STRING(warn);
         expected_seq = seq_id;
     }
 
     expected_seq++;
 
     if (data_len != 512u)
     {
         char info[64];
         snprintf(info, sizeof(info),
                  "Data length = %lu (expected 512)\n\r", (unsigned long)data_len);
         PRINT_STRING(info);
     }
 
     /* Write data block to eMMC */
     write_payload_to_emmc(data, data_len);
 }
 
 /*******************************************************************************
  * @brief  Write a 512-byte data block to eMMC
  ******************************************************************************/
 static void write_payload_to_emmc(const uint8_t *payload, uint32_t payload_len)
 {
     memcpy(g_mmc_tx_buff, payload, payload_len);

     snprintf(message, sizeof(message), "[INFO] Escribiendo bloque %lu en eMMC...\n\r", (unsigned long)g_block_count);
     PRINT_STRING(message);
 
     mss_mmc_status_t status = multi_block_write_transfer(START_ADDRESS+ g_block_count);

     //mss_mmc_status_t status = MSS_MMC_single_block_write(
      //   (uint32_t *)g_mmc_tx_buff,
       //  START_ADDRESS + g_block_count);
 
     if (status != MSS_MMC_TRANSFER_SUCCESS)
     {
         snprintf(message, sizeof(message),
                  "[ERROR] eMMC block %lu write failed\n\r",
                  (unsigned long)g_block_count);
         PRINT_STRING(message);
     }
 
     g_block_count++;
 }
 
 /*******************************************************************************
  * @brief  Ethernet RX callback (called by MAC driver)
  ******************************************************************************/
 static void mac_rx_callback(void *this_mac,
                             uint32_t queue_no,
                             uint8_t *p_rx_packet,
                             uint32_t pckt_length,
                             mss_mac_rx_desc_t *cdesc,
                             void *caller_info)
 {
     (void)this_mac;
     (void)queue_no;
     (void)cdesc;
     (void)caller_info;
 
     /* Process received frame payload */
     process_ethernet_payload(p_rx_packet, pckt_length);
 
     /* Release RX buffer back to MAC driver */
     MSS_MAC_receive_pkt((mss_mac_instance_t *)this_mac, 0, p_rx_packet, 0, 1);
 }
 
 /*******************************************************************************
  * @brief  Main entry point (executed by hart0 / E51)
  ******************************************************************************/
 void e51(void)
 {
     /* System init */
     PLIC_init();
     SYSREG->SOFT_RESET_CR = 0U;
     SYSREG->SUBBLK_CLOCK_CR = 0xFFFFFFFFUL;
     SysTick_Config();
 
     /* Enable Ethernet IRQs */
     PLIC_EnableIRQ(PLIC_MAC0_INT_INT_OFFSET);
     PLIC_EnableIRQ(PLIC_MAC0_QUEUE1_INT_OFFSET);
     PLIC_EnableIRQ(PLIC_MAC0_QUEUE2_INT_OFFSET);
     PLIC_EnableIRQ(PLIC_MAC0_QUEUE3_INT_OFFSET);
     PLIC_EnableIRQ(PLIC_MAC0_EMAC_INT_OFFSET);
     PLIC_EnableIRQ(PLIC_MAC0_MMSL_INT_OFFSET);
 
     /* UART init */
     MSS_UART_init(DEMO_UART, MSS_UART_115200_BAUD,
                   MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY |
                   MSS_UART_ONE_STOP_BIT);
 
     /* Peripheral init */
     ethernet_init();
     mss_mmc_status_t emmc_status = mmc_init_emmc();
     sprintf(message, "\n\r eMMC init status: %d\n\r", emmc_status);
     PRINT_STRING(message);
 
     /* Idle loop */
     while (1)
     {
         for (volatile uint32_t d = 0; d < 100000000; d++) { __asm__("nop"); }
     }
 }
 
 /*******************************************************************************
  * @brief  Reset the MMC peripheral block
  ******************************************************************************/
 static void mmc_reset_block(void)
 {
     SYSREG->SUBBLK_CLOCK_CR |= (uint32_t)(SUBBLK_CLOCK_CR_MMC_MASK);
     SYSREG->SOFT_RESET_CR   |= (uint32_t)(SOFT_RESET_CR_MMC_MASK);
     SYSREG->SOFT_RESET_CR   &= ~(uint32_t)(SOFT_RESET_CR_MMC_MASK);
 }
 
 /*******************************************************************************
  * @brief  Initialize eMMC (50 MHz, 8-bit bus)
  ******************************************************************************/
 static mss_mmc_status_t mmc_init_emmc(void)
 {
     mss_mmc_status_t ret_status;
 
     mss_config_clk_rst(MSS_PERIPH_EMMC, (uint8_t)1, PERIPHERAL_ON);

     __enable_irq();
     PLIC_SetPriority(MMC_main_PLIC, 2u);
     PLIC_SetPriority(MMC_wakeup_PLIC, 2u);
 
     ret_status = MSS_MPU_configure(MSS_MPU_MMC, MSS_MPU_PMP_REGION3,
                                    LIM_BASE_ADDRESS, LIM_SIZE,
                                    MPU_MODE_READ_ACCESS |
                                    MPU_MODE_WRITE_ACCESS |
                                    MPU_MODE_EXEC_ACCESS,
                                    MSS_MPU_AM_NAPOT, 0u);
 
     if (g_mmc_initialized == 1u)
     {
         ret_status = MSS_MMC_INIT_SUCCESS;
     }
     else
     {
         ASSERT(mss_does_xml_ver_support_switch() == true);
 
         if (!switch_mssio_config(EMMC_MSSIO_CONFIGURATION))
         {
             while (1u);
         }
 
         /* eMMC configuration parameters */
         g_mmc.clk_rate       = MSS_MMC_CLOCK_50MHZ;
         g_mmc.card_type      = MSS_MMC_CARD_TYPE_MMC;
         g_mmc.bus_speed_mode = MSS_MMC_MODE_SDR;
         g_mmc.data_bus_width = MSS_MMC_DATA_WIDTH_8BIT;
         g_mmc.bus_voltage    = MSS_MMC_1_8V_BUS_VOLTAGE;
 
         mmc_reset_block();
         ret_status = MSS_MMC_init(&g_mmc);
 
         if (ret_status == MSS_MMC_INIT_SUCCESS)
         {
             g_mmc_initialized = 1u;
         }
     }
 
 #if 1 /* Optional: erase first 500 sectors for clean start */
     for (uint16_t sector = 0; sector < 500; sector++)
     {
         ret_status = MSS_MMC_erase(sector, 1u);
     }
 #endif
 
     return ret_status;
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

 static mss_mmc_status_t multi_block_write_transfer(uint32_t sector_number) {
    mss_mmc_status_t status = 0u;

    MSS_MMC_set_handler(transfer_complete_handler);

    //status = multi_block_write(sector_number, USE_ADMA2);
    if (status == MSS_MMC_TRANSFER_SUCCESS) {
        status = multi_block_write(sector_number, USE_SDMA);
    }

    return status;
}

static mss_mmc_status_t multi_block_write(uint32_t sector_number, uint8_t dma_type) {
    mss_mmc_status_t status = 0u;

    if (USE_ADMA2 == dma_type) {
        status = MSS_MMC_adma2_write(g_mmc_tx_buff, sector_number, BLOCK_SIZE_BYTES);
    } else if (USE_SDMA == dma_type) {
        status = MSS_MMC_sdma_write(g_mmc_tx_buff, sector_number, BLOCK_SIZE_BYTES);
    } else {
        status = -1;
    }

    if (status == MSS_MMC_TRANSFER_IN_PROGRESS) {
        do {
            status = MSS_MMC_get_transfer_status();
        } while (status == MSS_MMC_TRANSFER_IN_PROGRESS);
    }

    return status;
}
 
