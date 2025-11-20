// frame_receiver.cpp : Linux version cleaned and adapted
// Compile with:
// g++ -std=c++17 frame_receiver.cpp -o frame_receiver -lsfml-graphics -lsfml-window -lsfml-system -lpcap -lpthread -ltiff

#include <iostream>
#include <thread>
#include <cstring>
#include <cstdint>
#include <pcap.h>
#include <tiffio.h>

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <SFML/Network.hpp>
#include <SFML/OpenGL.hpp>

using BYTE   = uint8_t;
using USHORT = uint16_t;
using DWORD  = uint32_t;

sf::Texture texture;

// -----------------------------------------------------------------------------
// Global RAW and TEXELS buffers
// -----------------------------------------------------------------------------
static uint8_t* RAW_BUFFER = nullptr;    // 8 MB raw buffer
static uint8_t* TEXELS     = nullptr;    // SFML preview buffer

#define CHUNK_SIZE 1024

// -----------------------------------------------------------------------------
// Packet structures
// -----------------------------------------------------------------------------
struct packet_t
{
    uint8_t  opcode;
    uint32_t length;
    uint32_t magic_code;
    uint16_t width;
    uint16_t height;
    uint32_t crc;
    uint32_t chunk;
    uint32_t num_chunks;
    uint32_t buffer_size;
    uint8_t  buffer[1048];
};

struct eth_frame_t
{
    uint8_t dst_mac[6];
    uint8_t src_mac[6];
    uint16_t eth_type;
    uint8_t payload[sizeof(packet_t)];
};

// -----------------------------------------------------------------------------
// Save TIFF (16-bit grayscale)
// -----------------------------------------------------------------------------
void save_tiff_16bit(const char* filename,
                     const uint8_t* raw,
                     int width,
                     int height)
{
    TIFF* tif = TIFFOpen(filename, "w");
    if (!tif)
    {
        printf("ERROR opening TIFF file: %s\n", filename);
        return;
    }

    TIFFSetField(tif, TIFFTAG_IMAGEWIDTH,  width);
    TIFFSetField(tif, TIFFTAG_IMAGELENGTH, height);
    TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 16);
    TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
    TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
    TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
    TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, height);
    TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);

    std::vector<uint16_t> row16(width);

    for (int row = 0; row < height; row++)
    {
        const uint16_t* src = (const uint16_t*)(raw + row * width * 2);

        for (int x = 0; x < width; x++)
        {
            uint16_t v12 = src[x] & 0x0FFF;
            uint16_t v16 = (v12 * 65535) / 4095;
            row16[x] = v16;
        }

        TIFFWriteScanline(tif, row16.data(), row, 0);
    }

    TIFFClose(tif);
    printf("TIFF 16-bit saved: %s\n", filename);
}


// -----------------------------------------------------------------------------
// Packet handler
// -----------------------------------------------------------------------------
void packet_handler(u_char*, const struct pcap_pkthdr*, const u_char* pkt_data)
{
    const packet_t* pckt = reinterpret_cast<const packet_t*>(pkt_data + 16);

    if (pckt->magic_code != 0xb007c0deu)
        return;

    // Allocate RAW buffer once (8MB)
    if (!RAW_BUFFER)
        RAW_BUFFER = new uint8_t[pckt->width * pckt->height * 2];

    // Copy raw chunk (1024 bytes)
    size_t offset = pckt->chunk * CHUNK_SIZE;
    memcpy(RAW_BUFFER + offset, pckt->buffer, pckt->buffer_size);

    // Create texel buffer for display (RGBA)
    if (!TEXELS)
        TEXELS = new uint8_t[pckt->width * pckt->height * 4];

    // Update preview (convert 12bit → 8bit)
    size_t tex_index = (pckt->chunk * (CHUNK_SIZE / 2)) * 4;
    int j = 0;
    for (int i = 0; i < CHUNK_SIZE; i += 2)
    {
        uint8_t lo = pckt->buffer[j++];
        uint8_t hi = pckt->buffer[j++];

        uint16_t color12 = ((uint16_t)hi << 8) | lo;
        color12 &= 0x0FFF;

        uint8_t gray8 = (color12 >> 4);

        TEXELS[tex_index++] = gray8;
        TEXELS[tex_index++] = gray8;
        TEXELS[tex_index++] = gray8;
        TEXELS[tex_index++] = 255;
    }

    texture.update(TEXELS);

    // Last chunk → save TIFF
    if (pckt->chunk == pckt->num_chunks - 1)
    {
        time_t now = time(nullptr);
        struct tm tm_now;
        localtime_r(&now, &tm_now);
        char filename[64];
        strftime(filename, sizeof(filename), "capture_%Y%m%d_%H%M%S.tiff", &tm_now);
        save_tiff_16bit(filename, RAW_BUFFER, pckt->width, pckt->height);
    }
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main()
{
    pcap_if_t* alldevs;
    pcap_if_t* d;
    int inum;
    int i = 0;
    pcap_t* adhandle;
    char errbuf[PCAP_ERRBUF_SIZE];

    // Retrieve the device list
    if (pcap_findalldevs(&alldevs, errbuf) == -1)
    {
        std::cerr << "Error in pcap_findalldevs: " << errbuf << "\n";
        return 1;
    }

    for (d = alldevs; d; d = d->next)
    {
        std::cout << ++i << ". " << d->name;
        if (d->description)
            std::cout << " (" << d->description << ")\n";
        else
            std::cout << " (No description available)\n";
    }

    std::cout << "Enter interface number (1-" << i << "): ";
    std::cin >> inum;

    for (d = alldevs, i = 0; i < inum - 1; d = d->next, i++);

    adhandle = pcap_open_live(d->name, 65536, 1, 1000, errbuf);
    if (!adhandle)
    {
        std::cerr << "Unable to open the adapter: " << d->name << "\n";
        return 1;
    }

    pcap_freealldevs(alldevs);

    std::thread([adhandle]() {
        pcap_loop(adhandle, 0, packet_handler, nullptr);
    }).detach();

    // ---------------- SFML WINDOW ----------------

    unsigned int windowSize = 960;

    sf::RenderWindow window(sf::VideoMode(windowSize, windowSize), "RFIM Image Display");
    sf::View view(sf::FloatRect(0.f, 0.f, 2048.f, 2048.f));
    window.setView(view);

    if (!texture.create(2048, 2048))
    {
        std::cerr << "Could not create texture\n";
        return -1;
    }

    sf::Sprite sprite(texture);

    sprite.setScale(
        float(window.getSize().x) / 2048.f,
        float(window.getSize().y) / 2048.f
    );

    // ---------------- MAIN LOOP ----------------

    while (window.isOpen())
    {
        sf::Event e;
        while (window.pollEvent(e))
        {
            if (e.type == sf::Event::Closed)
                window.close();

            if (e.type == sf::Event::Resized)
            {
                // Adjust the window size
                view.setSize(e.size.width, e.size.height);
                view.setCenter(e.size.width / 2.f, e.size.height / 2.f);
                window.setView(view);

                // Sclate the window
                float sx = float(e.size.width)  / 2048.f;
                float sy = float(e.size.height) / 2048.f;
                sprite.setScale(sx, sy);
            }
        }

        window.clear();
        window.draw(sprite);
        window.display();
    }

    pcap_close(adhandle);
    return 0;
}
