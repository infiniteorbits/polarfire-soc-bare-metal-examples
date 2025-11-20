// frame_reciever.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#define RAW_CHUNKS
#define CHUNK_SIZE 1024
/// #define DEBUG_PRINT


#include <iostream>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <SFML/Network.hpp>
#include <SFML/OpenGL.hpp>

#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
#include <iphlpapi.h>
#include <tchar.h>
#endif

#include <pcap.h>
#include <thread>

#ifdef _WIN32
#pragma comment(lib, "wpcap.lib")
#pragma comment(lib, "Packet.lib")
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "sfml-graphics.lib")
#pragma comment(lib, "sfml-window.lib")
#pragma comment(lib, "sfml-system.lib")
#endif

sf::Texture texture;

#ifdef _WIN32
BOOL LoadNpcapDlls()
{
    WCHAR npcap_dir[MAX_PATH];
    UINT len;
    len = GetCurrentDirectory(MAX_PATH, npcap_dir);
    if (!len) {
        fprintf(stderr, "Error in GetSystemDirectory: %x", GetLastError());
        return FALSE;
    }
    _tcscat_s(npcap_dir, 512, _T("\\Npcap"));
    if (SetDllDirectory(npcap_dir) == 0) {
        fprintf(stderr, "Error in SetDllDirectory: %x", GetLastError());
        return FALSE;
    }
    return TRUE;
}
#endif

/* prototype of the packet handler */
void packet_handler(u_char* param, const struct pcap_pkthdr* header, const u_char* pkt_data);

int main() {

		pcap_if_t* alldevs;
		pcap_if_t* d;
		int inum;
		int i = 0;
		pcap_t* adhandle;
		char errbuf[PCAP_ERRBUF_SIZE];

#ifdef _WIN32
		/* Load Npcap and its functions. */
		if (!LoadNpcapDlls())
		{
			fprintf(stderr, "Couldn't load Npcap\n");
			exit(1);
		}
#endif

		/* Retrieve the device list */
		if (pcap_findalldevs(&alldevs, errbuf) == -1)
		{
			fprintf(stderr, "Error in pcap_findalldevs: %s\n", errbuf);
			exit(1);
		}

		/* Print the list */
		for (d = alldevs; d; d = d->next)
		{
			printf("%d. %s", ++i, d->name);
			if (d->description)
				printf(" (%s)\n", d->description);
			else
				printf(" (No description available)\n");
		}

		if (i == 0)
		{
			printf("\nNo interfaces found! Make sure Npcap is installed.\n");
			return -1;
		}

		printf("Enter the interface number (1-%d):", i);
		scanf("%d", &inum);

		if (inum < 1 || inum > i)
		{
			printf("\nInterface number out of range.\n");
			/* Free the device list */
			pcap_freealldevs(alldevs);
			return -1;
		}

		/* Jump to the selected adapter */
		for (d = alldevs, i = 0; i < inum - 1; d = d->next, i++);

		/* Open the device */
		/* Open the adapter */
		if ((adhandle = pcap_open_live(d->name,	// name of the device
			8192,			// portion of the packet to capture. 
			// 65536 grants that the whole packet will be captured on all the MACs.
			1,				// promiscuous mode (nonzero means promiscuous)
			1,			// read timeout
			errbuf			// error buffer
		)) == NULL)
		{
			fprintf(stderr, "\nUnable to open the adapter. %s is not supported by Npcap\n", d->name);
			/* Free the device list */
			pcap_freealldevs(alldevs);
			return -1;
		}

		printf("\nlistening on %s...\n", d->description);

		/* At this point, we don't need any more the device list. Free it */
		pcap_freealldevs(alldevs);
		pcap_set_buffer_size(adhandle, 10000);

		std::thread([adhandle]() {
			while (1) {
				/* start the capture */
				pcap_loop(adhandle, 0, packet_handler, NULL);
			}
		}).detach();

		
		sf::RenderWindow window(sf::VideoMode(sf::Vector2u(1024, 1024)), "RFIM Image Display");	
		
		(void)texture.resize(sf::Vector2u(2048, 2048), false);
		texture.setRepeated(false);

		sf::Sprite sprite(texture);
		sprite.setScale(
			{
				static_cast<float>(window.getSize().x)/texture.getSize().x,
				static_cast<float>(window.getSize().y) / texture.getSize().y,
			});

		

		uint64_t cnt = 0;
		// Capture and respond to ARP packets
		while (window.isOpen()) {
			// Event processing
			while (const std::optional event = window.pollEvent())
			{
				// Request for closing the window
				if (event->is<sf::Event::Closed>())
					window.close();
			}
			
			window.clear();
			window.draw(sprite);
			window.display();

#if 0
			sf::Image img = texture.copyToImage();
			img.saveToFile({ "./frames/frame" + std::to_string(cnt++) + ".png" });

			cnt = cnt % 8192;
#endif
		}

		pcap_close(adhandle);
		return 0;
	
}


/* Callback function invoked by libpcap for every incoming packet */
void packet_handler(u_char * param, const struct pcap_pkthdr* header, const u_char * pkt_data)
{
	struct tm* ltime;
	char timestr[16];
	time_t local_tv_sec;

	/*
		* unused parameters
		*/
	(VOID)(param);
	//(VOID)(pkt_data);

	/* convert the timestamp to readable format */
	local_tv_sec = header->ts.tv_sec;
	ltime = localtime(&local_tv_sec);
	strftime(timestr, sizeof timestr, "%H:%M:%S", ltime);	

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
		uint8_t		buffer[1048];
	};
	struct eth_frame_t {
		unsigned char dst_mac[6];
		unsigned char src_mac[6];
		unsigned short eth_type;
		unsigned char payload[sizeof(packet_t)];
	};

	//if (header->len >= sizeof(eth_frame_t))
	{
		eth_frame_t* frame = (eth_frame_t*)pkt_data;
		
		packet_t* pckt = (packet_t*)((uint8_t*)pkt_data + 16);

#ifdef RAW_CHUNKS
		if (header->len == CHUNK_SIZE)
#else
		if (pckt->magic_code == 0xb007c0deu)
#endif
		{
#ifndef RAW_CHUNKS
#ifdef DEBUG_PRINT
			printf("%s,%.6d len:%d, chunk %d/%d, %dx%d, buf %d\n", timestr, header->ts.tv_usec, header->len,
				pckt->chunk, pckt->num_chunks, pckt->width, pckt->height, pckt->buffer_size);
#endif 
#endif		
			
			static uint8_t* texels = nullptr;
			if (texels == nullptr)
			{
#ifndef RAW_CHUNKS
				texels = new uint8_t[pckt->width * pckt->height * 4];
				memset(texels, 255, pckt->width* pckt->height * 4);
#else
				texels = new uint8_t[2048 * 2048 * 4];
				memset(texels, 255, 2048 * 2048 * 4);
#endif
			}				
							
#ifdef RAW_CHUNKS
			uint16_t* header = (uint16_t*)((uint64_t)pkt_data);
			auto idx = (*header) * (CHUNK_SIZE / 2) * 4; /// If the chunk index is encoded in the first texel

			// if (*(header+1) != 0xb007u) return;

#if 0
			static uint16_t chunk_index = 0;
			idx = chunk_index * (CHUNK_SIZE / 2) * 4; /// Till then, like this

			chunk_index = (chunk_index + 1) % ((2048 * 2048 * 2) / CHUNK_SIZE);
#endif
#else
			auto idx = pckt->chunk * (CHUNK_SIZE / 2) * 4;
#endif
#ifdef DEBUG_PRINT
			printf("idx = %d\n", idx);
#endif

			auto j = 0;
			auto k = 0;
			for (auto i = 0; i < CHUNK_SIZE; i +=2)
			{
#ifdef RAW_CHUNKS
				uint8_t gray_lo = pkt_data[j++];
				uint8_t gray_hi = pkt_data[j++];
#else
				uint8_t gray_lo = pckt->buffer[j++];
				uint8_t gray_hi = pckt->buffer[j++];
#endif

				uint16_t color = (gray_hi << 8) | gray_lo;
				float  clr = float(color) / 4095;
				uint8_t final = clr * 255u;

				texels[idx++] = texels[idx++] = texels[idx++] = final;
				texels[idx++] = 255;
			}

			static uint16_t updt = 0;
			if (updt++ % 60 == 0)
			{
				texture.update(texels);
			}

#ifdef RAW_CHUNKS
			uint8_t* bptr = (uint8_t*)pkt_data;
#else
			uint8_t* bptr = (uint8_t*)pckt->buffer;
#endif
#ifdef DEBUG_PRINT
			for (uint8_t i = 0; i < 16 * 4; i++)
			{
				// output 16 HEX Bytes:
				printf(" %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x\r\n",
					*bptr++, *bptr++, *bptr++, *bptr++, *bptr++, *bptr++, *bptr++, *bptr++, *bptr++, *bptr++, *bptr++, *bptr++, *bptr++, *bptr++, *bptr++, *bptr++);

			}
#endif
		}
	}
#ifdef DEBUG_PRINT
	printf("%s,%.6d len:%d\n", timestr, header->ts.tv_usec, header->len);
#endif
	

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
