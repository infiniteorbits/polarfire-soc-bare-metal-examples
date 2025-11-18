// frame_receiver.cpp : Linux version cleaned and adapted
// Compile with:
// g++ -std=c++17 frame_receiver.cpp -o frame_receiver -lsfml-graphics -lsfml-window -lsfml-system -lpcap -pthread

#include <iostream>
#include <thread>
#include <cstring>
#include <cstdint>
#include <pcap.h>

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <SFML/Network.hpp>
#include <SFML/OpenGL.hpp>

// -----------------------------------------------------------------------------
// Type aliases for Windows-like types
// -----------------------------------------------------------------------------
using BYTE   = uint8_t;
using USHORT = uint16_t;
using DWORD  = uint32_t;
using BOOL   = int;
#define TRUE  1
#define FALSE 0

// -----------------------------------------------------------------------------
// Global texture
// -----------------------------------------------------------------------------
sf::Texture texture;

// -----------------------------------------------------------------------------
// ARP packet structure
// -----------------------------------------------------------------------------
struct ArpPacket {
    BYTE destMac[6];
    BYTE sourceMac[6];
    USHORT frameType;
    USHORT hwType;
    USHORT protoType;
    BYTE hwSize;
    BYTE protoSize;
    USHORT opcode;
    BYTE senderMac[6];
    DWORD senderIp;
    BYTE targetMac[6];
    DWORD targetIp;
};

// -----------------------------------------------------------------------------
// Send ARP response (dummy, not really used here)
// -----------------------------------------------------------------------------
void sendArpResponse(pcap_t* pcapHandle, const BYTE* packet, int packetSize)
{
    const ArpPacket* arpPacket = reinterpret_cast<const ArpPacket*>(packet);

    // Check if it's an ARP request
    if (ntohs(arpPacket->opcode) == 1)
    {
        BYTE responsePacket[42]; // ARP response packet size
        memcpy(responsePacket, arpPacket->senderMac, 6);           // Dest MAC
        memcpy(responsePacket + 6, arpPacket->sourceMac, 6);       // Source MAC
        *reinterpret_cast<USHORT*>(responsePacket + 12) = htons(0x0806); // Frame type
        *reinterpret_cast<USHORT*>(responsePacket + 14) = htons(0x0001); // HW type
        *reinterpret_cast<USHORT*>(responsePacket + 16) = htons(0x0800); // Proto type
        responsePacket[18] = 6;                                     // HW size
        responsePacket[19] = 4;                                     // Proto size
        *reinterpret_cast<USHORT*>(responsePacket + 20) = htons(2); // Opcode (response)
        memcpy(responsePacket + 22, arpPacket->targetMac, 6);       // Sender MAC
        *reinterpret_cast<DWORD*>(responsePacket + 28) = arpPacket->targetIp; // Sender IP
        memcpy(responsePacket + 32, arpPacket->senderMac, 6);       // Target MAC
        *reinterpret_cast<DWORD*>(responsePacket + 38) = arpPacket->senderIp; // Target IP

        // Send ARP response packet
        if (pcap_sendpacket(pcapHandle, responsePacket, 42) == -1) {
            std::cerr << "Error sending ARP response packet: " << pcap_geterr(pcapHandle) << "\n";
        }
    }
}

// -----------------------------------------------------------------------------
// Packet handler prototype
// -----------------------------------------------------------------------------
void packet_handler(u_char* param, const struct pcap_pkthdr* header, const u_char* pkt_data);

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

    // Print the list
    for (d = alldevs; d; d = d->next)
    {
        std::cout << ++i << ". " << d->name;
        if (d->description)
            std::cout << " (" << d->description << ")\n";
        else
            std::cout << " (No description available)\n";
    }

    if (i == 0)
    {
        std::cout << "\nNo interfaces found! Make sure libpcap is installed.\n";
        pcap_freealldevs(alldevs);
        return -1;
    }

    std::cout << "Enter the interface number (1-" << i << "): ";
    std::cin >> inum;

    if (inum < 1 || inum > i)
    {
        std::cerr << "Interface number out of range.\n";
        pcap_freealldevs(alldevs);
        return -1;
    }

    // Jump to the selected adapter
    for (d = alldevs, i = 0; i < inum - 1; d = d->next, i++);

    // Open the adapter
    adhandle = pcap_open_live(d->name, 65536, 1, 1000, errbuf);
    if (adhandle == nullptr)
    {
        std::cerr << "Unable to open the adapter: " << d->name << "\n";
        pcap_freealldevs(alldevs);
        return -1;
    }

    std::cout << "\nListening on " << (d->description ? d->description : d->name) << "...\n";

    // Free the device list
    pcap_freealldevs(alldevs);

    // Start capture in a separate thread
    std::thread([adhandle]() {
        while (true) {
            pcap_loop(adhandle, 0, packet_handler, nullptr);
        }
    }).detach();

    // -------------------------------------------------------------------------
    // SFML window setup
    // -------------------------------------------------------------------------
	unsigned int windowSize = 960;

	sf::RenderWindow window(sf::VideoMode(windowSize, windowSize), "RFIM Image Display");
	sf::View view(sf::FloatRect(0.f, 0.f, 2048.f, 2048.f));  // vista basada en la textura original
	window.setView(view);

	window.setVerticalSyncEnabled(true);
	window.setFramerateLimit(60);


    if (!texture.create(2048, 2048))
    {
        std::cerr << "Error: could not create texture\n";
        return -1;
    }

    texture.setRepeated(false);
    sf::Sprite sprite(texture);
    sprite.setTextureRect(sf::IntRect(0, 0, texture.getSize().x, texture.getSize().y));

	// Escala inicial: ajusta sprite para llenar la ventana
	float scaleX = static_cast<float>(window.getSize().x) / texture.getSize().x;
	float scaleY = static_cast<float>(window.getSize().y) / texture.getSize().y;
	sprite.setScale(scaleX, scaleY);


    uint64_t cnt = 0;

    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        if (window.getSize().x != view.getSize().x || window.getSize().y != view.getSize().y)
		{
			// Ajusta la vista al nuevo tamaño
			view.setSize(sf::Vector2f(2048.f, 2048.f));
			view.setCenter(1024.f, 1024.f);
			window.setView(view);

			// Escala el sprite para llenar la nueva ventana
			float scaleX = static_cast<float>(window.getSize().x) / texture.getSize().x;
			float scaleY = static_cast<float>(window.getSize().y) / texture.getSize().y;
			sprite.setScale(scaleX, scaleY);
		}


        window.clear();
        window.draw(sprite);
        window.display();
    }

    pcap_close(adhandle);
    return 0;
}

// -----------------------------------------------------------------------------
// Callback function invoked by libpcap for every incoming packet
// -----------------------------------------------------------------------------
void packet_handler(u_char* param, const struct pcap_pkthdr* header, const u_char* pkt_data)
{
    (void)param;
    (void)pkt_data;

    struct tm* ltime;
    char timestr[16];
    time_t local_tv_sec;

    local_tv_sec = header->ts.tv_sec;
    ltime = localtime(&local_tv_sec);
    strftime(timestr, sizeof timestr, "%H:%M:%S", ltime);

#define CHUNK_SIZE 1024

    struct packet_t
    {
        uint8_t opcode;
        uint32_t length;
        uint32_t magic_code;
        uint16_t width;
        uint16_t height;
        uint32_t crc;
        uint32_t chunk;
        uint32_t num_chunks;
        uint32_t buffer_size;
        uint8_t buffer[1048];
    };

    struct eth_frame_t
    {
        unsigned char dst_mac[6];
        unsigned char src_mac[6];
        unsigned short eth_type;
        unsigned char payload[sizeof(packet_t)];
    };

    const eth_frame_t* frame = reinterpret_cast<const eth_frame_t*>(pkt_data);
    const packet_t* pckt = reinterpret_cast<const packet_t*>(pkt_data + 16);

    if (pckt->magic_code == 0xb007c0deu)
    {
        printf("%s,%.6ld len:%d, chunk %d/%d, %dx%d, buf %d\n",
               timestr, header->ts.tv_usec, header->len,
               pckt->chunk, pckt->num_chunks, pckt->width, pckt->height, pckt->buffer_size);

        static uint8_t* texels = nullptr;
        if (texels == nullptr)
        {
            texels = new uint8_t[pckt->width * pckt->height * 4];
            memset(texels, 255, pckt->width * pckt->height * 4);
        }

        auto idx = pckt->chunk * (CHUNK_SIZE / 2) * 4;
        auto j = 0;
        for (auto i = 0; i < CHUNK_SIZE; i += 2)
        {
            uint8_t gray_lo = pckt->buffer[j++];
            uint8_t gray_hi = pckt->buffer[j++];
            uint16_t color = (gray_hi << 8) | gray_lo;
            float clr = float(color) / 4095.0f;
            uint8_t final = static_cast<uint8_t>(clr * 255.0f);

            texels[idx++] = final;
            texels[idx++] = final;
            texels[idx++] = final;
            texels[idx++] = 255;
        }

        texture.update(texels);

        // Save the image
        if (pckt->chunk == pckt->num_chunks - 1)
        {
            std::cout << "Image completed. Saving PNG...\n";

            sf::Image img;
            img.create(pckt->width, pckt->height, texels);

            // Obtain date for the filename
            time_t now = time(nullptr);
            struct tm tm_now;
            localtime_r(&now, &tm_now);

            char filename[64];
            strftime(filename, sizeof(filename),
                    "capture_%Y%m%d_%H%M%S.png", &tm_now);

            if (img.saveToFile(filename))
                std::cout << "Image saved: " << filename << "\n";
            else
                std::cerr << "Error saving the image\n";
        }

    }

    printf("%s,%.6ld len:%d\n", timestr, header->ts.tv_usec, header->len);
}
