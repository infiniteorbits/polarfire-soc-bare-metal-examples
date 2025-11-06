// frame_reciever.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <SFML/Network.hpp>
#include <SFML/OpenGL.hpp>

#include <winsock2.h>
#include <windows.h>
#include <iphlpapi.h>
#include <pcap.h>

#include <thread>
#include <tchar.h>

#pragma comment(lib, "wpcap.lib")
#pragma comment(lib, "Packet.lib")
#pragma comment(lib, "ws2_32.lib")

// ARP packet structure
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

void sendArpResponse(pcap_t* pcapHandle, const BYTE* packet, int packetSize) {
    // Parse ARP packet
    ArpPacket* arpPacket = (ArpPacket*)packet;

    // Check if it's an ARP request
    if (ntohs(arpPacket->opcode) == 1) {
        // Create ARP response packet
        BYTE responsePacket[42]; // ARP response packet size
        memcpy(responsePacket, arpPacket->senderMac, 6); // Dest MAC
        memcpy(responsePacket + 6, arpPacket->sourceMac, 6); // Source MAC
        *(USHORT*)(responsePacket + 12) = htons(0x0806); // Frame type
        *(USHORT*)(responsePacket + 14) = htons(0x0001); // HW type
        *(USHORT*)(responsePacket + 16) = htons(0x0800); // Proto type
        responsePacket[18] = 6; // HW size
        responsePacket[19] = 4; // Proto size
        *(USHORT*)(responsePacket + 20) = htons(2); // Opcode (response)
        memcpy(responsePacket + 22, arpPacket->targetMac, 6); // Sender MAC
        *(DWORD*)(responsePacket + 28) = arpPacket->targetIp; // Sender IP
        memcpy(responsePacket + 32, arpPacket->senderMac, 6); // Target MAC
        *(DWORD*)(responsePacket + 38) = arpPacket->senderIp; // Target IP

        // Send ARP response packet
        if (pcap_sendpacket(pcapHandle, responsePacket, 42) == -1) {
            printf("Error sending ARP response packet: %s\n", pcap_geterr(pcapHandle));
        }
    }
}
BOOL LoadNpcapDlls()
{
    WCHAR npcap_dir[512];
    UINT len;
    len = GetSystemDirectory(npcap_dir, 480);
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
			65536,			// portion of the packet to capture. 
			// 65536 grants that the whole packet will be captured on all the MACs.
			1,				// promiscuous mode (nonzero means promiscuous)
			1000,			// read timeout
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

		std::thread([adhandle]() {
			while (1) {
				/* start the capture */
				pcap_loop(adhandle, 0, packet_handler, NULL);
			}
		}).detach();

		sf::RenderWindow window(sf::VideoMode(sf::Vector2u(800, 600)), "Image Display");
		sf::Texture texture;
		texture.loadFromFile("image.jpg");
		sf::Sprite sprite(texture);

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
	(VOID)(pkt_data);

	/* convert the timestamp to readable format */
	local_tv_sec = header->ts.tv_sec;
	ltime = localtime(&local_tv_sec);
	strftime(timestr, sizeof timestr, "%H:%M:%S", ltime);

	printf("%s,%.6d len:%d\n", timestr, header->ts.tv_usec, header->len);

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
