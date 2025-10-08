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
#include "RFIM_Phy.h"


 /* -------------------------------------------------------------------------
  *  UART and utility macros
  * ------------------------------------------------------------------------- */
 #define DEMO_UART       &g_mss_uart0_lo
 #define PRINT_STRING(x) MSS_UART_polled_tx_string(DEMO_UART, (uint8_t *)(x))

 /* -------------------------------------------------------------------------
  *  Memory, MMC, and system configuration
  * ------------------------------------------------------------------------- */
#define ICICLE_KIT              0
#define ETHERTYPE_DATA          0x88B5
#define LIM_BASE_ADDRESS        0x81000000UL
#define LIM_SIZE                0x200000u
#define ERROR_INTERRUPT         0x8000u
#define TRANSFER_COMPLETE       0x1u
#define USE_SDMA                0u
#define USE_ADMA2               1u
#define BLOCK_SIZE_BYTES        512
#define START_ADDRESS           0
#define BUFFER_SIZE             1 * 1024 * 1024 // in MB
#define BLOCK_SIZE              512u
#define RECIVED_PAQUET_SIZE     1024
#define PACKET_IDLE             0
#define PACKET_ARMED            1
#define PACKET_DONE             2
#define PACKET_MAX              16384U

 /* -------------------------------------------------------------------------
  *  Global buffers and state
  * ------------------------------------------------------------------------- */
#define DDR_BASE_ADDR   BASE_ADDRESS_NON_CACHED_32_DDR
//uint8_t *g_mmc_tx_buff = (uint8_t *)DDR_BASE_ADDR;
uint8_t g_mmc_tx_buff[BUFFER_SIZE];
mss_mmc_cfg_t g_mmc;
uint8_t g_mmc_initialized = 0u;
static volatile uint32_t g_block_count    = 0;   /* number of 512B pages written so far */
char message[128];
volatile uint64_t   g_tick_counter = 0;
volatile uint32_t   g_packet_length = 0;
volatile int        g_reload = PACKET_IDLE;
mss_mac_instance_t* g_mac = &g_mac0;
volatile int    g_capture = PACKET_IDLE;
uint8_t         g_packet_data[PACKET_MAX];
mss_mac_cfg_t       g_mac_config;

static uint8_t g_mac_rx_buffer[MSS_MAC_RX_RING_SIZE][MSS_MAC_MAX_RX_BUF_SIZE]
    __attribute__((aligned(16)));

 /* -------------------------------------------------------------------------
  *  Function prototypes
  * ------------------------------------------------------------------------- */
static void low_level_init(void);
static mss_mmc_status_t mmc_init_emmc(void);
void print_buffer_hex(const uint8_t *buf, uint32_t len);

static void send_confirmation_frame(void)
{
    const uint8_t CMD_SAVE_COMPLETED = 0x04u;
    const char msg_text[] = "SAVE_COMPLETED";
    const uint32_t seq_id = 0u;   /* Optional sequence (not relevant here) */
    uint8_t dest_mac[6] = {0x00, 0x11, 0x22, 0x68, 0x46, 0xb7}; // MAC of your PC
    uint8_t src_mac[6]  = {0x00, 0xfc, 0x00, 0x12, 0x34, 0x56}; // MAC of the board

    /* Build Ethernet payload: [CMD][SEQ(4B)][DATA...] */
    uint8_t tx_payload[1 + 4 + sizeof(msg_text)];
    tx_payload[0] = CMD_SAVE_COMPLETED;
    tx_payload[1] = (uint8_t)(seq_id >> 24) & 0xFF;
    tx_payload[2] = (uint8_t)(seq_id >> 16) & 0xFF;
    tx_payload[3] = (uint8_t)(seq_id >> 8)  & 0xFF;
    tx_payload[4] = (uint8_t)seq_id & 0xFF;
    memcpy(&tx_payload[5], msg_text, sizeof(msg_text));

    /* Build Ethernet frame: dst + src + type + payload */
    uint8_t p_rx_packet[6 + 6 + 2 + sizeof(tx_payload)];
    memcpy(p_rx_packet, dest_mac, 6u);
    memcpy(&p_rx_packet[6], src_mac, 6u);
    p_rx_packet[12] = (ETHERTYPE_DATA >> 8) & 0xFF;
    p_rx_packet[13] = ETHERTYPE_DATA & 0xFF;
    memcpy(&p_rx_packet[14], tx_payload, sizeof(tx_payload));

    uint32_t pckt_length = sizeof(p_rx_packet);

    /* Send the packet */
    int32_t tx_status = MSS_MAC_send_pkt(g_mac, 0, p_rx_packet, pckt_length, (void *)0);

    if (tx_status != MSS_MAC_SUCCESS)
    {
        //PRINT_STRING("[WARN] Failed to send 'Save completed' notification\n\r");
    }
    else
    {
        //PRINT_STRING("[INFO] Sent 'Save completed' notification\n\r");
    }
}

void print_buffer_hex(const uint8_t *buf, uint32_t len)
{

    PRINT_STRING("\n\r[INFO] Dumping buffer (hex view):\n\r");

    for (uint32_t i = 0; i < len; i++)
    {
        /* Al inicio de cada línea (16 bytes) imprimimos el offset */
        if (i % 16 == 0)
        {
            sprintf(message, "\n\r%04lu: ", (unsigned long)i);
            PRINT_STRING(message);
        }

        /* Imprimir cada byte en hexadecimal */
        sprintf(message, "%02X ", buf[i]);
        PRINT_STRING(message);
    }

    PRINT_STRING("\n\r[INFO] End of buffer dump\n\r");
}

/*******************************************************************************
 * @brief  Write a 512-byte data block to eMMC
 ******************************************************************************/
static void write_payload_to_emmc(void)
{

    //print_buffer_hex(g_mmc_tx_buff, 100);

    mss_mmc_status_t status =  MSS_MMC_sdma_write( g_mmc_tx_buff,
                        START_ADDRESS + g_block_count,
                        BUFFER_SIZE);
    do {
        status =  MSS_MMC_get_transfer_status();
    }while (status == MSS_MMC_TRANSFER_IN_PROGRESS);

    g_block_count+=(BUFFER_SIZE/BLOCK_SIZE_BYTES);

    send_confirmation_frame();
}

/*******************************************************************************
 * @brief  Process Ethernet payload (extract CMD + sequence + data)
 ******************************************************************************/
 static void process_ethernet_payload(uint8_t *packet, uint32_t packet_len)
 {
     /* Extract EtherType (bytes 12-13, big-endian) */
     uint16_t ether_type = (uint16_t)(((uint16_t)packet[12] << 8) | (uint16_t)packet[13]);
     if (ether_type != ETHERTYPE_DATA)
     {
         /* Not our application frame */
         //PRINT_STRING("Ignoring non-application frame\n\r");
         return;
     }
 
     /* Skip 14-byte Ethernet header (6 dst + 6 src + 2 type) */
     uint8_t *payload = packet + 14u;
     uint32_t payload_len = packet_len - 14u;
 
     if (payload_len < 5u) /* CMD (1B) + SEQ (4B) */
         return;
 
     /* Extract command byte */
     uint8_t cmd = payload[0u];
 
     /* Extract 4-byte big-endian sequence ID */
     uint32_t seq_id = ((uint32_t)payload[1] << 24) |
                       ((uint32_t)payload[2] << 16) |
                       ((uint32_t)payload[3] << 8)  |
                        (uint32_t)payload[4];
 
     /* Data begins after CMD + SEQ */
     uint8_t *data = payload + 5u;
     uint32_t data_len = payload_len - 5u;
 
     /* Persistent state across calls */
     static uint32_t expected_seq = 0u;
     static uint8_t first_packet = 1u;
     static uint32_t accumulated_len = 0u;
     static uint32_t block_count = 0u;


     switch (cmd)
     {
         case 0x01: /* CMD_START */
             PRINT_STRING("[INFO] Start of transmission received\n\r");
             expected_seq = seq_id;
             first_packet = 0u;
             accumulated_len = 0u;
             block_count = 0u;
             break;
 
         case 0x02: /* CMD_DATA */
         {
             /* Sequence check */
             if (!first_packet && seq_id != expected_seq)
             {
                 PRINT_STRING("[WARN] Sequence mismatch\n\r");
                 expected_seq = seq_id; /* Resync */
             }
 
             expected_seq++;
 
             /* Copy the chunk into the main buffer */
             memcpy(&g_mmc_tx_buff[accumulated_len], data, data_len);
             accumulated_len += data_len;
             block_count += (data_len / BLOCK_SIZE_BYTES);
 
             /* If buffer full → write to eMMC */
             if (accumulated_len >= BUFFER_SIZE)
             {
                 //PRINT_STRING("[INFO] Writing full buffer to eMMC...\n\r");
                 write_payload_to_emmc();
                 accumulated_len = 0u;
                 block_count = 0u;
             }
             break;
         }
 
         case 0x03: /* CMD_END */
         {
             PRINT_STRING("[INFO] End of transmission received\n\r");
 
             /* If data still pending in buffer, write it */
             if (accumulated_len > 0u)
             {
                 PRINT_STRING("[INFO] Writing remaining buffer to eMMC...\n\r");
                 write_payload_to_emmc();
                 accumulated_len = 0u;
                 block_count = 0u;
             }
 
             PRINT_STRING("[INFO] Transmission complete\n\r");
             break;
         }
 
         default:
             PRINT_STRING("[WARN] Unknown CMD received\n\r");
             break;
     }
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

    if (PACKET_ARMED == g_capture) /* Looking for packet so grab a copy */
    {
        if (pckt_length > PACKET_MAX) {
            pckt_length = PACKET_MAX;
        }

        memcpy(g_packet_data, p_rx_packet, pckt_length);

        g_packet_length = pckt_length;
        g_capture = PACKET_DONE; /* and say we go it */
    }

    /* Release RX buffer back to MAC driver */
    MSS_MAC_receive_pkt((mss_mac_instance_t *)this_mac, 0, p_rx_packet, 0, 1);
}

/*******************************************************************************
 * @brief  Initialize Ethernet MAC for reception
 ******************************************************************************/
 static int ethernet_init(void)
 {
     int ret_status = 0;
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
     //MSS_MAC_set_tx_callback(g_mac, 0, packet_tx_complete_handler);
     MSS_MAC_set_rx_callback(g_mac, 0, mac_rx_callback);
 
     /* Register RX buffers */
     for (uint32_t i = 0; i < MSS_MAC_RX_RING_SIZE; i++)
     {
         MSS_MAC_receive_pkt(g_mac, 0, g_mac_rx_buffer[i], 0,
             (i == MSS_MAC_RX_RING_SIZE - 1) ? -1 : 0);
     }
 
     /* Start PHY autonegotiation */
     g_mac->phy_mac_autonegotiate((const void *)g_mac);

     return ret_status;
 }

 /*******************************************************************************
  * @brief  Main entry point (executed by hart0 / E51)
  ******************************************************************************/
void e51(void)
{
    write_csr(mscratch, 0);
    write_csr(mcause, 0);
    write_csr(mepc, 0);

    (void)mss_config_clk_rst(MSS_PERIPH_MAC0, (uint8_t) MPFS_HAL_FIRST_HART, PERIPHERAL_ON);
    (void)mss_config_clk_rst(MSS_PERIPH_MAC1, (uint8_t) MPFS_HAL_FIRST_HART, PERIPHERAL_ON);

    PLIC_init();

    SYSREG->SOFT_RESET_CR = 0U;
    SYSREG->SUBBLK_CLOCK_CR = 0xFFFFFFFFUL;
    SYSREG->SOFT_RESET_CR   &= (uint32_t)~(SOFT_RESET_CR_FPGA_MASK);
    mss_config_clk_rst(MSS_PERIPH_EMMC, (uint8_t) 1, PERIPHERAL_ON);
     PLIC_init();
    __disable_local_irq((int8_t)MMUART0_E51_INT);

    SysTick_Config();

    PLIC_EnableIRQ(PLIC_MAC0_INT_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_QUEUE1_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_QUEUE2_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_QUEUE3_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_EMAC_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_MMSL_INT_OFFSET);

    /* UART init */
    MSS_UART_init(DEMO_UART,
                  MSS_UART_115200_BAUD,
                  MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);


    __enable_irq();
    PLIC_SetPriority(MMC_main_PLIC, 2u);
    PLIC_SetPriority(MMC_wakeup_PLIC, 2u);

    /* Ethernet init */
    #if ICICLE_KIT
    int ethernet_status = ethernet_init(); 
    #else
    low_level_init();
    #endif
    sprintf(message, "\n\r ethernet Init status: %d ", ethernet_status);
    PRINT_STRING(message);

    /* eMMC init */
    mss_mmc_status_t emmc_status = mmc_init_emmc();
    sprintf(message, "\n\r eMMC Init status: %d ", emmc_status);
    PRINT_STRING(message);

    for (uint16_t sector_number = 0; sector_number < 500; sector_number++){
        emmc_status = MSS_MMC_erase(sector_number, 1u);
    }
    sprintf(message, "\n\r eMMC erase status: %d \n\r", emmc_status);
    PRINT_STRING(message);

    g_capture = PACKET_ARMED;

    while(1){

        if (PACKET_DONE == g_capture) {
            process_ethernet_payload(g_packet_data, g_packet_length);
            g_capture = PACKET_ARMED;
        }
    }

}

 /*******************************************************************************
  * @brief  Reset the MMC peripheral block
  ******************************************************************************/
static void mmc_reset_block(void)
{
    SYSREG->SUBBLK_CLOCK_CR |= (uint32_t)(SUBBLK_CLOCK_CR_MMC_MASK);
    SYSREG->SOFT_RESET_CR |= (uint32_t)(SOFT_RESET_CR_MMC_MASK);
    SYSREG->SOFT_RESET_CR &= ~(uint32_t)(SOFT_RESET_CR_MMC_MASK);
}

 /*******************************************************************************
  * @brief  Initialize eMMC (50 MHz, 8-bit bus)
  ******************************************************************************/
static mss_mmc_status_t mmc_init_emmc(void)
{
    mss_mmc_status_t ret_status;

    /* DMA init for eMMC */
    MSS_MPU_configure(MSS_MPU_MMC,
                     MSS_MPU_PMP_REGION3,
                     LIM_BASE_ADDRESS,
                     LIM_SIZE,
                     MPU_MODE_READ_ACCESS|MPU_MODE_WRITE_ACCESS|MPU_MODE_EXEC_ACCESS,
                     MSS_MPU_AM_NAPOT,
                     0u);

    if (g_mmc_initialized == 1u)
    {
        ret_status = MSS_MMC_INIT_SUCCESS;
    }
    else
    {
#if ICICLE_KIT
        ASSERT(mss_does_xml_ver_support_switch() == true);

        if (!switch_mssio_config(EMMC_MSSIO_CONFIGURATION))
        {
            while (1u);
        }
#endif
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

static void low_level_init(void)
{
    uint32_t count;

#if defined(MSS_MAC_USE_DDR)
    g_mac_rx_buffer = g_mss_mac_ddr_ptr;
    g_mss_mac_ddr_ptr += MSS_MAC_RX_RING_SIZE * MSS_MAC_MAX_RX_BUF_SIZE;
#endif
    /*-------------------------- Initialize the MAC --------------------------*/
    /*
     * Get the default configuration for the Ethernet MAC and change settings
     * to match the system/application. The default values typically changed
     * are:
     *  - interface:
     *      Specifies the interface used to connect the Ethernet MAC to the PHY.
     *      Example choice are MII, GMII, TBI.
     *  - phy_addr:
     *      Specifies the MII management interface address of the external PHY.
     *  - mac_addr:
     *      Specifies the MAC address of the device. This number should be
     *      unique on the network the device is connected to.
     *  - speed_duplex_select:
     *      Specifies the allowed speed and duplex mode for setting up a link.
     *      This can be used to specify the set of allowed speeds and duplex
     *      modes used during auto-negotiation or force the link speed to a
     *      specific speed and duplex mode.
     */
    MSS_MAC_cfg_struct_def_init(&g_mac_config);

    g_mac_config.speed_duplex_select = MSS_MAC_ANEG_ALL_SPEEDS;
    g_mac_config.mac_addr[0] = 0x00;
    g_mac_config.mac_addr[1] = 0xFC;
    g_mac_config.mac_addr[2] = 0x00;
    g_mac_config.mac_addr[3] = 0x12;
    g_mac_config.mac_addr[4] = 0x34;
    g_mac_config.mac_addr[5] = 0x56;
    g_mac_config.tsu_clock_select = 0U;
    g_mac_config.speed_mode = MSS_MAC_100_FDX;
    g_mac_config.phy_addr = PHY_VSC8541_MDIO_ADDR;
    g_mac_config.phy_type = MSS_MAC_DEV_PHY_VSC8541;
    g_mac_config.pcs_phy_addr = PHY_NULL_MDIO_ADDR;
    g_mac_config.interface_type = GMII;
    g_mac_config.phy_autonegotiate = MSS_MAC_VSC8541_phy_autonegotiate;
    g_mac_config.phy_mac_autonegotiate = MSS_MAC_VSC8541_mac_autonegotiate;
    g_mac_config.phy_get_link_status = MSS_MAC_VSC8541_phy_get_link_status;
    //g_mac_config.phy_init = MSS_MAC_VSC8541_phy_init;
    g_mac_config.phy_set_link_speed = MSS_MAC_VSC8541_phy_set_link_speed;
    /* Init GEM1 for MDIO interface only */
    g_mac_config.mac_addr[5] = 0x57;
    MSS_MAC_init(&g_mac1, &g_mac_config);

    g_mac_config.mac_addr[5] = 0x56;
    g_mac = &g_mac0;
    g_mac_config.phy_addr = PHY_VSC8541_MDIO_ADDR;
    //g_mac_config.phy_controller = &g_mac1;

    MSS_MAC_init(g_mac, &g_mac_config);

    RFIM_MAC0_phy_init();
    RFIM_MAC1_phy_init();

    PRINT_STRING("\n\r");

    RFIM_MAC0_phy_autonegotiate();
    RFIM_MAC1_phy_autonegotiate();

    RFIM_MAC0_phy_dump_id(DEMO_UART);
    RFIM_MAC1_phy_dump_id(DEMO_UART);

    PRINT_STRING("\n\r");

    RFIM_MAC0_phy_dump_link_status(DEMO_UART);
    RFIM_MAC1_phy_dump_link_status(DEMO_UART);

    PRINT_STRING("\n\r");
    PRINT_STRING("Setting speed to 100 Mbps Full Duplex\n\r");

    RFIM_MAC0_phy_set_speed(PHY_MAC100MBPS);
    RFIM_MAC1_phy_set_speed(PHY_MAC100MBPS);

    PRINT_STRING("\n\r");

    RFIM_MAC0_phy_dump_link_status(DEMO_UART);
    RFIM_MAC1_phy_dump_link_status(DEMO_UART);

    PRINT_STRING("\n\r");

    /*
     * Not doing the tx disable/enable sequence here around the queue allocation
     * adjustment results in tx_go being set which causes the new tx code to
     * hang waiting for the last tx to complete...
     */
    g_mac->mac_base->NETWORK_CONTROL &= ~GEM_ENABLE_TRANSMIT;

    /* Allocate all 4 segments to queue 0 as this is our only one... */
    g_mac->mac_base->TX_Q_SEG_ALLOC_Q0TO3 = 2;

    g_mac->mac_base->NETWORK_CONTROL |= GEM_ENABLE_TRANSMIT;

    /*
     * Register MAC interrupt handler listener functions. These functions will
     * be called  by the MAC driver when a packet has been sent or received.
     * These callback functions are intended to help managing transmit and
     * receive buffers by indicating when a transmit buffer can be released or
     * a receive buffer has been filled with an rx packet.
     */

    //MSS_MAC_set_tx_callback(g_mac, 0, packet_tx_complete_handler);
    MSS_MAC_set_rx_callback(g_mac, 0, mac_rx_callback);

  //  MSS_MAC_set_tx_callback(&g_mac1, 0, packet_tx_complete_handler);
 //  MSS_MAC_set_rx_callback(&g_mac1, 0, mac_rx_callback);

    /*
     * Allocate receive buffers.
     *
     * We prime the pump with a full set of packet buffers and then re use them
     * as each packet is handled.
     *
     * This function will need to be called each time a packet is received to
     * hand back the receive buffer to the MAC driver.
     */
    for (count = 0; count < MSS_MAC_RX_RING_SIZE; ++count)
    {
        /*
         * We allocate the buffers with the Ethernet MAC interrupt disabled
         * until we get to the last one. For the last one we ensure the Ethernet
         * MAC interrupt is enabled on return from MSS_MAC_receive_pkt().
         */
#if defined(MSS_MAC_USE_DDR)
        if (count != (MSS_MAC_RX_RING_SIZE - 1))
        {
            MSS_MAC_receive_pkt(g_mac,
                                0,
                                g_mac_rx_buffer + count * MSS_MAC_MAX_RX_BUF_SIZE,
                                0,
                                0);
        }
        else
        {
            MSS_MAC_receive_pkt(g_mac,
                                0,
                                g_mac_rx_buffer + count * MSS_MAC_MAX_RX_BUF_SIZE,
                                0,
                                -1);
        }
#else
        if (count != (MSS_MAC_RX_RING_SIZE - 1))
        {
            MSS_MAC_receive_pkt(g_mac, 0, g_mac_rx_buffer[count], 0, 0);
        }
        else
        {
            MSS_MAC_receive_pkt(g_mac, 0, g_mac_rx_buffer[count], 0, -1);
        }
#endif
    }

    g_mac = &g_mac1;

    g_mac->mac_base->NETWORK_CONTROL &= ~GEM_ENABLE_TRANSMIT;

        /* Allocate all 4 segments to queue 0 as this is our only one... */
        g_mac->mac_base->TX_Q_SEG_ALLOC_Q0TO3 = 2;

        g_mac->mac_base->NETWORK_CONTROL |= GEM_ENABLE_TRANSMIT;

        /*
         * Register MAC interrupt handler listener functions. These functions will
         * be called  by the MAC driver when a packet has been sent or received.
         * These callback functions are intended to help managing transmit and
         * receive buffers by indicating when a transmit buffer can be released or
         * a receive buffer has been filled with an rx packet.
         */

        //MSS_MAC_set_tx_callback(g_mac, 0, packet_tx_complete_handler);
        MSS_MAC_set_rx_callback(g_mac, 0, mac_rx_callback);

    for (count = 0; count < MSS_MAC_RX_RING_SIZE; ++count)
        {
            /*
             * We allocate the buffers with the Ethernet MAC interrupt disabled
             * until we get to the last one. For the last one we ensure the Ethernet
             * MAC interrupt is enabled on return from MSS_MAC_receive_pkt().
             */
    #if defined(MSS_MAC_USE_DDR)
            if (count != (MSS_MAC_RX_RING_SIZE - 1))
            {
                MSS_MAC_receive_pkt(g_mac,
                                    0,
                                    g_mac_rx_buffer + count * MSS_MAC_MAX_RX_BUF_SIZE,
                                    0,
                                    0);
            }
            else
            {
                MSS_MAC_receive_pkt(g_mac,
                                    0,
                                    g_mac_rx_buffer + count * MSS_MAC_MAX_RX_BUF_SIZE,
                                    0,
                                    -1);
            }
    #else
            if (count != (MSS_MAC_RX_RING_SIZE - 1))
            {
                MSS_MAC_receive_pkt(g_mac, 0, g_mac_rx_buffer[count], 0, 0);
            }
            else
            {
                MSS_MAC_receive_pkt(g_mac, 0, g_mac_rx_buffer[count], 0, -1);
            }
    #endif
        }

    delay((DELAY_CYCLES_100MS * 20));
    g_mac = &g_mac0;
}
