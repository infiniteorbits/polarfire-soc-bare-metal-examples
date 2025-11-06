/*
 * protocol.c
 *
 *  Created on: 29 Oct 2025
 *      Author: TRAJCE Nikolov
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include "protocol.h"

void set_packet_data(struct packet_t* pckt, uint16_t w, uint16_t h, uint8_t* data, uint32_t data_size)
{
    pckt->width = w;
    pckt->height = h;
    pckt->length += data_size; /// Assumes calling init_packet in advance
    pckt->buffer_size = data_size;

    memcpy(pckt->buffer, (uint8_t*)data, CHUNK_SIZE);
}

void protocol_clear(void)
{
}

#ifdef __cplusplus
}
#endif


