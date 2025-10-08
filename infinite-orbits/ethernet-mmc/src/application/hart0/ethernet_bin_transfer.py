#!/usr/bin/env python3
import socket
import struct
import os
import time
import select

# ============================
# CONFIGURATION
# ============================
IFACE = "enx0011226846b7"             # Network interface (remote srever)
SRC_MAC = b'\x00\x11\x22\x68\x46\xb7' # PC MAC (remote srever)
DST_MAC = b'\x00\xfc\x00\x12\x34\x56' # custom board MAC
ETH_TYPE = b'\x88\xb5'                # Custom EtherType

FILE_PATH = "core-image-minimal-dev-icicle-kit-es-amp-20250730092842.rootfs.wic"                # File to send
CHUNK_SIZE = 1024                     # Must be multiple of 512
BIG_DELAY_BYTES = 1 * 1024 * 1024     # Bytes before big delay trigger (1 MB)
BIG_DELAY_TIME = 10                   # Fallback delay (seconds)
ADD_SEQ_HEADER = True                 # Add 4-byte sequence counter
SHOW_PROGRESS = True                  # Show progress
DELAY = 0.00015                        # Small delay between frames (seconds)

# Commands
CMD_START = 0x01
CMD_DATA  = 0x02
CMD_END   = 0x03
CMD_SAVE_COMPLETED = 0x04

# ============================
# HELPER FUNCTIONS
# ============================

def read_file_chunks(path, chunk_size):
    """Generator to read the file in fixed-size chunks."""
    with open(path, "rb") as f:
        while True:
            chunk = f.read(chunk_size)
            if not chunk:
                break
            if len(chunk) < chunk_size:
                chunk += bytes(chunk_size - len(chunk))  # Pad last chunk
            yield chunk


def build_frame(cmd, seq, data):
    """Build Ethernet frame with header: [CMD][SEQ(4B)][DATA]"""
    cmd_byte = struct.pack("B", cmd)
    seq_bytes = struct.pack(">I", seq)
    payload = cmd_byte + seq_bytes + data
    frame = DST_MAC + SRC_MAC + ETH_TYPE + payload
    return frame


def wait_for_save_completed(sock_rx, timeout=5.0):
    """Wait for CMD_SAVE_COMPLETED (0x04) frame from Icicle, with debug traces."""
    start_time = time.time()
    while True:
        ready, _, _ = select.select([sock_rx], [], [], 0.1)
        if ready:
            frame = sock_rx.recv(2048)
            frame_len = len(frame)
            #print(f"[RX DEBUG] Received frame len={frame_len}")

            # Muestra los primeros 32 bytes en hex
           # print("         ", " ".join(f"{b:02X}" for b in frame[:32]))

            if frame_len < 15:
                print("[RX DEBUG] Frame too short, skipping.")
                continue

            ether_type = (frame[12] << 8) | frame[13]
            cmd = frame[14] if frame_len > 14 else None

           # print(f"[RX DEBUG] EtherType=0x{ether_type:04X}, CMD={cmd:#04x} if present")

            # Filtrado por EtherType
            if ether_type != int.from_bytes(ETH_TYPE, 'big'):
                #print(f"[RX DEBUG] Different EtherType, ignoring (expected 0x{int.from_bytes(ETH_TYPE, 'big'):04X})")
                continue

            if cmd == CMD_SAVE_COMPLETED:
                #print("[INFO] Received 'Save completed' ACK from Icicle — continuing transmission")
                return True
            else:
                #print(f"[RX DEBUG] Frame CMD {cmd:#04x} != CMD_SAVE_COMPLETED (0x{CMD_SAVE_COMPLETED:02X})")
                time.sleep(0.1)

        if time.time() - start_time > timeout:
            print("[WARN] Timeout waiting for 'Save completed' (fallback to delay)")
            return False



def send_raw_ethernet(file_path, chunk_size, delay=0.0):
    """Send binary file with command header and 'save completed' wait support."""
    sock_tx = socket.socket(socket.AF_PACKET, socket.SOCK_RAW)
    sock_tx.bind((IFACE, 0))

    # Separate socket for reception
    # Correct socket for RX
    sock_rx = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x88B5))
    sock_rx.bind((IFACE, 0))
    sock_rx.setblocking(False)


    # Correct definitions for Linux AF_PACKET membership
    SOL_PACKET = 263
    PACKET_ADD_MEMBERSHIP = 1
    PACKET_MR_PROMISC = 1

    # Get interface index (must be non-zero)
    if_index = socket.if_nametoindex(IFACE)

    # struct packet_mreq {
    #     int mr_ifindex;      /* Interface index */
    #     unsigned short mr_type;  /* PACKET_MR_PROMISC */
    #     unsigned short mr_alen;  /* Not used */
    #     unsigned char mr_address[8]; /* Not used */
    # }
    mreq = struct.pack("IHH8s", if_index, PACKET_MR_PROMISC, 0, b"\x00" * 8)
    sock_rx.setsockopt(SOL_PACKET, PACKET_ADD_MEMBERSHIP, mreq)



    file_size = os.path.getsize(file_path)
    total_sent = 0
    seq = 0
    start = time.time()
    bytes_since_delay = 0

    print("[INFO] Sending START frame...")
    start_frame = build_frame(CMD_START, 0, b'START')
    sock_tx.send(start_frame)
    time.sleep(0.1)

    for chunk in read_file_chunks(file_path, chunk_size):
        frame = build_frame(CMD_DATA, seq, chunk)
        sock_tx.send(frame)
        seq += 1
        total_sent += len(chunk)
        bytes_since_delay += len(chunk)

        if SHOW_PROGRESS and seq % 100 == 0:
            progress = 100 * total_sent / file_size
            print(f"\rSent: {total_sent}/{file_size} bytes ({progress:.1f}%)", end="")

        # Small delay between frames
        if delay > 0:
            time.sleep(delay)

        # Big delay (wait for Save Completed)
        if bytes_since_delay >= BIG_DELAY_BYTES:
            #print(f"\n[INFO] Waiting for 'Save completed' after {bytes_since_delay/1024:.1f} KB sent...")
            ack = wait_for_save_completed(sock_rx, timeout=BIG_DELAY_TIME)
            if not ack:
                print("[WARN] No ACK received — applying fallback delay")
                time.sleep(BIG_DELAY_TIME)
            bytes_since_delay = 0

    print("\n[INFO] Sending END frame...")
    end_frame = build_frame(CMD_END, seq, b'END')
    sock_tx.send(end_frame)

    elapsed = time.time() - start
    bits_sent = total_sent * 8
    bps = bits_sent / elapsed
    mbps = bps / 1_000_000

    print(f"\nTransfer completed in {elapsed:.2f}s")
    print(f"Transfer speed: {bps:,.0f} bps ({mbps:.2f} Mbps)")
    print(f"Total frames sent: {seq + 2} (including START and END)")

    sock_tx.close()
    sock_rx.close()

# ============================
# MAIN
# ============================

if __name__ == "__main__":
    if CHUNK_SIZE % 512 != 0:
        raise ValueError("CHUNK_SIZE must be a multiple of 512 bytes")

    print(f"Using CHUNK_SIZE = {CHUNK_SIZE} bytes (multiple of 512)")
    print(f"Interface: {IFACE}")
    print(f"Destination MAC: {DST_MAC.hex(':')}")
    print(f"Source MAC: {SRC_MAC.hex(':')}")
    print(f"Delay per frame: {DELAY} s")
    print("Starting raw Ethernet transfer...\n")

    send_raw_ethernet(FILE_PATH, CHUNK_SIZE, delay=DELAY)
