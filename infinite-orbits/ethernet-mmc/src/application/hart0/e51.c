#if 1


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

#define DEMO_UART       &g_mss_uart0_lo
#define PRINT_STRING(x) MSS_UART_polled_tx_string(DEMO_UART, (uint8_t *)x);
#define LIM_BASE_ADDRESS        0x08000000u
#define LIM_SIZE                0x200000u
#define ERROR_INTERRUPT         0x8000u
#define TRANSFER_COMPLETE       0x1u
#define USE_SDMA                0u
#define USE_ADMA2               1u
#define BLOCK_SIZE_BYTES        512
#define SLOT_SIZE_BYTES         (100 * 1024 * 1024)  // 100 MB in bytes
#define STREAM_GEN_BASE_ADDR    0x4A000000u
#define START_ADDRESS           0

// Variables globales de buffer circular
mss_mmc_cfg_t g_mmc;
uint8_t g_mmc_initialized = 0u;
uint8_t g_mmc_rx_buff[BLOCK_SIZE_BYTES] = {0};
uint8_t  g_mmc_tx_buff[BLOCK_SIZE_BYTES];
uint32_t g_bytes_received = 0;
uint32_t g_head = 0;
uint32_t g_tail = 0;
uint32_t g_block_count = 0u;
char message[128];


static uint8_t g_mac_rx_buffer[MSS_MAC_RX_RING_SIZE][MSS_MAC_MAX_RX_BUF_SIZE]
    __attribute__((aligned(16)));

mss_mac_cfg_t g_mac_config;
mss_mac_instance_t *g_mac = &g_mac0;



uint8_t tx_buffer[1500];
uint32_t length;

static void mac_rx_callback(
    void *this_mac,
    uint32_t queue_no,
    uint8_t *p_rx_packet,
    uint32_t pckt_length,
    mss_mac_rx_desc_t *cdesc,
    void *caller_info);
static void write_payload_to_emmc(const uint8_t *payload, uint32_t payload_len);
static int8_t multi_block_write_transfer(uint32_t sector_number);
static int8_t multi_block_write(uint32_t sector_number, uint8_t dma_type);
static mss_mmc_status_t mmc_init_emmc(void);
void build_frame(void);
void transfer_complete_handler(uint32_t status);

static void packet_tx_complete_handler(void *this_mac,
    uint32_t queue_no,
    mss_mac_tx_desc_t *cdesc,
    void *caller_info)
{
(void)this_mac;
(void)queue_no;
(void)cdesc;
(void)caller_info;
// Aquí podrías poner un contador de paquetes enviados si quieres
}

static void ethernet_init(void)
{
    MSS_MAC_cfg_struct_def_init(&g_mac_config);

    g_mac_config.speed_duplex_select = MSS_MAC_ANEG_ALL_SPEEDS;
    g_mac_config.mac_addr[0] = 0x00;
    g_mac_config.mac_addr[1] = 0xFC;
    g_mac_config.mac_addr[2] = 0x00;
    g_mac_config.mac_addr[3] = 0x12;
    g_mac_config.mac_addr[4] = 0x34;
    g_mac_config.mac_addr[5] = 0x56;

    g_mac_config.phy_addr = PHY_VSC8662_0_MDIO_ADDR;
    g_mac_config.phy_type = MSS_MAC_DEV_PHY_VSC8662;
    g_mac_config.interface_type = TBI;
    g_mac_config.phy_autonegotiate     = MSS_MAC_VSC8662_phy_autonegotiate;
    g_mac_config.phy_mac_autonegotiate = MSS_MAC_VSC8662_mac_autonegotiate;
    g_mac_config.phy_get_link_status   = MSS_MAC_VSC8662_phy_get_link_status;
    g_mac_config.phy_init              = MSS_MAC_VSC8662_phy_init;
    g_mac_config.phy_set_link_speed    = MSS_MAC_VSC8662_phy_set_link_speed;

    MSS_MAC_init(g_mac, &g_mac_config);

    MSS_MAC_set_tx_callback(g_mac, 0, packet_tx_complete_handler);
    MSS_MAC_set_rx_callback(g_mac, 0, mac_rx_callback);

    // Registrar buffers RX
    for (uint32_t i = 0; i < MSS_MAC_RX_RING_SIZE; i++) {
        MSS_MAC_receive_pkt(g_mac, 0, g_mac_rx_buffer[i], 0, (i == MSS_MAC_RX_RING_SIZE - 1) ? -1 : 0);
    }

    g_mac->phy_mac_autonegotiate(/* mss_mac_instance_t*/ (const void *)g_mac);
}


void build_frame(void) {
    uint8_t dest_mac[6] = {0xd8, 0x43, 0xae, 0xbb, 0xaa, 0x4f}; // MAC de tu PC
    uint8_t src_mac[6]  = {0x00, 0xfc, 0x00, 0x12, 0x34, 0x56}; // MAC Icicle
    uint16_t eth_type   = 0x88B5;  // EtherType personalizado (ejemplo)

    uint8_t payload[]   = "Hola desde Icicle!";

    // Construir cabecera
    memcpy(&tx_buffer[0],  dest_mac, 6);
    memcpy(&tx_buffer[6],  src_mac,  6);
    tx_buffer[12] = (uint8_t)(eth_type >> 8) & 0xFF;
    tx_buffer[13] = (uint8_t)(eth_type >> 0) & 0xFF;

    // Copiar payload
    memcpy(&tx_buffer[14], payload, sizeof(payload)-1);

    length = 14 + sizeof(payload)-1; // cabecera + datos
}

/*******************************************************************************
 * @brief  Procesa el contenido útil (payload) de una trama Ethernet recibida.
 *         Extrae los 512 bytes de datos después de la cabecera Ethernet.
 *
 * @param[in] packet       Puntero al buffer recibido.
 * @param[in] packet_len   Longitud total del paquete (bytes).
 *******************************************************************************/
static void process_ethernet_payload(uint8_t *packet, uint32_t packet_len)
{
    if (packet_len < 14) {
        PRINT_STRING("Frame too short\n\r");
        return;
    }

    // Skip Ethernet header (6 dst + 6 src + 2 ethertype)
    uint8_t *payload = packet + 14;
    uint32_t payload_len = packet_len - 14;

    // Extract 4-byte sequence counter (big endian)
    if (payload_len < 4) {
        PRINT_STRING("Payload too small for sequence header\n\r");
        return;
    }

    uint32_t seq_id = (payload[0] << 24) | (payload[1] << 16) |
                      (payload[2] << 8)  | (payload[3]);
    uint8_t *data = payload + 4;
    uint32_t data_len = payload_len - 4;

    // --- Sequence tracking ---
    static uint32_t expected_seq = 0;
    static uint8_t first_packet = 1;

    if (first_packet) {
        expected_seq = seq_id;
        first_packet = 0;
        PRINT_STRING("First frame received\n\r");
    } else {
        if (seq_id != expected_seq) {
            char warn[128];
            snprintf(warn, sizeof(warn),
                     "[WARN] Sequence mismatch: expected %lu, got %lu (lost %ld packets)\n\r",
                     (unsigned long)expected_seq,
                     (unsigned long)seq_id,
                     (long)(seq_id - expected_seq));
            PRINT_STRING(warn);

            // Recover: set new expected sequence
            expected_seq = seq_id;
        }
    }

    expected_seq++; // Next expected

    // --- Check data length (512 expected) ---
    if (data_len != 512) {
        char info[64];
        snprintf(info, sizeof(info),
                 "Data len = %lu (expected 512)\n\r", (unsigned long)data_len);
        PRINT_STRING(info);
    }

#if 0
    // Optional: print first bytes for debug
    PRINT_STRING("Data preview (first 512 bytes): ");
    for (uint32_t i = 0; i < data_len && i < 512; i++) {
        char byte_str[4];
        snprintf(byte_str, sizeof(byte_str), "%02X ", data[i]);
        PRINT_STRING(byte_str);
    }
    PRINT_STRING("\n\r");
#endif

    // Write the 512-byte data block to eMMC
    write_payload_to_emmc(data, data_len);
}


/*******************************************************************************
 * @brief  Copia datos recibidos en el buffer circular y escribe en eMMC cuando
 *         el bloque está completo (512 bytes).
 *******************************************************************************/
static void write_payload_to_emmc(const uint8_t *payload, uint32_t payload_len)
{
   memcpy(g_mmc_tx_buff, payload, payload_len);
        
  // snprintf(message, sizeof(message),
  // "[INFO] Escribiendo bloque %lu en eMMC...\n\r",
  // (unsigned long)g_block_count);
//MSS_UART_polled_tx_string(DEMO_UART, (const uint8_t *)message);


//mss_mmc_status_t status = MSS_MMC_sdma_write(g_mmc_tx_buff, START_ADDRESS + g_block_count, BLOCK_SIZE_BYTES);


    //multi_block_write_transfer(START_ADDRESS + g_block_count);
    mss_mmc_status_t status = MSS_MMC_single_block_write(
            (uint32_t *)g_mmc_tx_buff,
            START_ADDRESS + g_block_count);
    g_block_count++;
}

static void mac_rx_callback(
    void *this_mac,
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

    // Llamar a la función de procesamiento del payload
    process_ethernet_payload(p_rx_packet, pckt_length);

    // Indicar al driver que el buffer RX ya se puede reutilizar
    MSS_MAC_receive_pkt((mss_mac_instance_t *)this_mac, 0, p_rx_packet, 0, 1);
}




void e51(void)
{
    PLIC_init();

    SYSREG->SOFT_RESET_CR = 0U;
    SYSREG->SUBBLK_CLOCK_CR = 0xFFFFFFFFUL;

    SysTick_Config();

    PLIC_EnableIRQ(PLIC_MAC0_INT_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_QUEUE1_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_QUEUE2_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_QUEUE3_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_EMAC_INT_OFFSET);
    PLIC_EnableIRQ(PLIC_MAC0_MMSL_INT_OFFSET);

    MSS_UART_init(DEMO_UART,
        MSS_UART_115200_BAUD,
        MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT);

    ethernet_init();

    
   
    uint16_t sector_number = 0;
    mss_config_clk_rst(MSS_PERIPH_EMMC, (uint8_t) 1, PERIPHERAL_ON);

    uint8_t ret_status = MSS_MPU_configure(MSS_MPU_MMC,
        MSS_MPU_PMP_REGION3,
        LIM_BASE_ADDRESS,
        LIM_SIZE,
        MPU_MODE_READ_ACCESS|MPU_MODE_WRITE_ACCESS|MPU_MODE_EXEC_ACCESS,
        MSS_MPU_AM_NAPOT,
        0u);

    MSS_UART_polled_tx_string(DEMO_UART,
        (const uint8_t *)"\n\r Init start");
    mss_mmc_status_t status = mmc_init_emmc();
 //////////////START READ BLOCK 0 ////////////////

    MSS_UART_polled_tx_string(DEMO_UART,
        (const uint8_t *)"\r\n[INFO] Leyendo primer bloque...");
    
    status = MSS_MMC_single_block_read(0, (uint32_t *)g_mmc_rx_buff);
    
    if (status == MSS_MMC_TRANSFER_SUCCESS)
    {
        char message[128];
        MSS_UART_polled_tx_string(DEMO_UART,
            (const uint8_t *)"\r\n[OK] Bloque leído correctamente.\r\n");
    
        for (size_t i = 0; i < 512; i++)
        {
            // Imprime 16 bytes por línea
            if ((i % 16) == 0) {
                snprintf(message, sizeof(message), "\r\n[%03u] ", (unsigned int)i);
                MSS_UART_polled_tx_string(DEMO_UART, (const uint8_t *)message);
            }
    
            snprintf(message, sizeof(message), "%02X ", g_mmc_rx_buff[i]);
            MSS_UART_polled_tx_string(DEMO_UART, (const uint8_t *)message);
        }
    
        MSS_UART_polled_tx_string(DEMO_UART, (const uint8_t *)"\r\n[END]\r\n");
    }
    else
    {
        char msg[64];
        snprintf(msg, sizeof(msg),
                 "\r\n[ERROR] Fallo al leer bloque 0 (status=%d)\r\n", status);
        MSS_UART_polled_tx_string(DEMO_UART, (const uint8_t *)msg);
    }

    
    //////////////END READ BLOCK 0 ////////////////
    for (sector_number = 0; sector_number < 500; sector_number++)
    {
    ret_status = MSS_MMC_erase(sector_number, 1u);
    }

    sprintf(message, "\n\r - erase status: %d ", ret_status);
    MSS_UART_polled_tx_string(DEMO_UART, message);

    for (sector_number = 0; sector_number < 500; sector_number++)
    {
        ret_status = MSS_MMC_single_block_read(sector_number, (uint32_t *)g_mmc_rx_buff);
        for (uint16_t loop_count = 0u; loop_count < BLOCK_SIZE_BYTES; loop_count++)
        {
            if(g_mmc_rx_buff[loop_count] != 0)
        {
            sprintf(message, "\n \rsector %d: g_mmc_rx_buff[%d] = %02X ", sector_number, loop_count, g_mmc_rx_buff[loop_count]);
            MSS_UART_polled_tx_string(DEMO_UART, message);
        }
        }
    }

    sprintf(message, "\n\r - read status: %d ", ret_status);
    MSS_UART_polled_tx_string(DEMO_UART, message);

    MSS_UART_polled_tx_string(DEMO_UART,
        (const uint8_t *)"\n\r Init Done");



    while (1)
    {
        /*build_frame();

        int32_t tx_status;
        do {
            tx_status = MSS_MAC_send_pkt(&g_mac0, 0, tx_buffer, length, (void *)0);
        } while (tx_status != MSS_MAC_SUCCESS);*/

        // Delay ~1s
        for (volatile uint32_t d = 0; d < 100000000; d++) { __asm__("nop"); }
    }
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

static mss_mmc_status_t mmc_init_emmc(void)
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
        //switch_external_mux(EMMC_MSSIO_CONFIGURATION);
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

static int8_t multi_block_write_transfer(uint32_t sector_number) {
    int8_t status = 0u;

    MSS_MMC_set_handler(transfer_complete_handler);

    status = multi_block_write(sector_number, USE_ADMA2);
    if (status == MSS_MMC_TRANSFER_SUCCESS) {
        status = multi_block_write(sector_number, USE_SDMA);
    }

    return status;
}

static int8_t multi_block_write(uint32_t sector_number, uint8_t dma_type) {
    int8_t status = 0u;
    uint8_t p_buff[128];

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



#endif
