#!/usr/bin/env python3
import socket
import struct
import os
import time

# ============================
# CONFIGURATION
# ============================
IFACE = "enx00e04c68002b"             # Network interface
DST_MAC = b'\x00\xfc\x00\x12\x34\x56' # Icicle MAC
SRC_MAC = b'\xd8\x43\xae\xbb\xaa\x4f' # PC MAC
ETH_TYPE = b'\x88\xb5'                # Custom EtherType

FILE_PATH = "test.bin"                # File to send
CHUNK_SIZE = 512                     # Must be multiple of 512 (e.g. 512, 1024, 1536, ...)
ADD_SEQ_HEADER = True                 # Add 4-byte sequence counter
SHOW_PROGRESS = True                  # Show progress on screen
DELAY = 0.0016                        # Delay between frames (seconds) — set to 0 for max speed

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

def send_raw_ethernet(file_path, chunk_size, delay=0.0, add_seq=True):
    """Send a binary file using raw Ethernet frames via AF_PACKET sockets."""
    sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW)
    sock.bind((IFACE, 0))

    file_size = os.path.getsize(file_path)
    total_sent = 0
    seq = 0
    start = time.time()

    for chunk in read_file_chunks(file_path, chunk_size):
        # Optional 4-byte sequence number (big endian)
        if add_seq:
            seq_bytes = struct.pack(">I", seq)
            payload = seq_bytes + chunk
        else:
            payload = chunk

        # Build Ethernet frame: DST + SRC + EtherType + Payload
        frame = DST_MAC + SRC_MAC + ETH_TYPE + payload

        # Send the frame directly
        sock.send(frame)
        total_sent += len(chunk)
        seq += 1

        if delay > 0:
            time.sleep(delay)

        if SHOW_PROGRESS and seq % 100 == 0:
            progress = 100 * total_sent / file_size
            print(f"\rSent: {total_sent}/{file_size} bytes ({progress:.1f}%)", end="")

    elapsed = time.time() - start
    bits_sent = total_sent * 8
    bps = bits_sent / elapsed
    mbps = bps / 1_000_000

    print(f"\nTransfer completed in {elapsed:.2f}s")
    print(f"Transfer speed: {bps:,.0f} bps ({mbps:.2f} Mbps)")
    print(f"Total frames sent: {seq}")

    sock.close()

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

    send_raw_ethernet(FILE_PATH, CHUNK_SIZE, delay=DELAY, add_seq=ADD_SEQ_HEADER)
