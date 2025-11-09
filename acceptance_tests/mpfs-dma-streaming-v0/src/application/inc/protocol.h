///
/// File            | protocol.h
/// Description     | Simple protocol for streaming images from aqusition
///                 | into chunks over ethernet
///
/// Author          | Trajce Nikolov    | trajce.nikolov.nick@gmail.com
///                                     | trajce.nikolov.nick@outlook.com
/// Date            | October  2025
///
/// Copyright 2025  | RFIM Space
///

#ifndef APPLICATION_INC_PROTOCOL_H_
#define APPLICATION_INC_PROTOCOL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RESOLUTION_X    2048
#define RESOLUTION_Y    2048

#define CHUNK_SIZE      1024

struct packet_t
{
    uint8_t     opcode;
    uint32_t    length;
    uint32_t    magic_code;
    uint16_t    width;
    uint16_t    height;
    uint32_t    crc;
    uint32_t    chunk;
    uint32_t    num_chunks;
    uint32_t    buffer_size;
    uint8_t     buffer[CHUNK_SIZE + 24] __attribute__ ((aligned (4)));;
};

struct eth_frame_t
{
    uint8_t dst_mac[6];
    uint8_t src_mac[6];
    uint16_t eth_type;
    uint8_t payload[sizeof(struct packet_t)] __attribute__ ((aligned (4)));;
};

inline void init_packet(struct packet_t* pckt)
{
    pckt->num_chunks = 0;
    pckt->chunk = 0;
    pckt->opcode = 0u;
    pckt->magic_code = 0xb007c0deu;
    pckt->width = RESOLUTION_X;
    pckt->height = RESOLUTION_Y;
    pckt->crc = 0u;
    pckt->buffer_size = 0u;
    pckt->length = sizeof(struct packet_t);
}


/// Call init_packet before
///
extern void set_packet_data(struct packet_t* pckt, uint16_t w, uint16_t h, uint8_t* data, uint32_t data_size);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_INC_PROTOCOL_H_ */
