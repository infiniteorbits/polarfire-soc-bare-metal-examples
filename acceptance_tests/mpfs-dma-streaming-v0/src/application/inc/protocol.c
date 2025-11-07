///
/// File            | protocol.c
/// Description     | Simple protocol for streaming images from aqusition
///                 | into chunks over ethernet
///
/// Author          | Trajce Nikolov    | trajce.nikolov.nick@gmail.com
///                                     | trajce.nikolov.nick@outlook.com
/// Date            | October 2025
///
/// Copyright 2025  | RFIM Space
///

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


