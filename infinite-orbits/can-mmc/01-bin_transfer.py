#!/usr/bin/env python3
"""
@file send_wic_over_can.py
@brief Send binary data (e.g. .wic image) over CAN to an embedded target.
@details
  - Reads a binary file and splits it into reordered 8-byte chunks.
  - Sends data over CAN using python-can.
  - Introduces configurable small/large delays for pacing.
  - Displays progress and validates inputs.

@example
  python3 send_wic_over_can.py \
      --file test.bin \
      --channel can0 \
      --can-id 0x00 \
      --small-delay 0.0005 \
      --large-delay 0.02 \
      --threshold 512
"""

import argparse
import os
import sys
import time
import logging
from tqdm import tqdm
import can

# -----------------------------------------------------------------------------
# Logging setup
# -----------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("send_wic_over_can")

# -----------------------------------------------------------------------------
# CAN Communication
# -----------------------------------------------------------------------------
def send_can_message(bus: can.interface.Bus, can_id: int, data: bytes) -> None:
    """Send a single CAN frame."""
    try:
        message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        bus.send(message, timeout=0.1)
    except can.CanError as e:
        logger.error(f"Failed to send CAN frame: {e}")
        sys.exit(1)

# -----------------------------------------------------------------------------
# File Reading
# -----------------------------------------------------------------------------
def read_data_from_file(file_path: str, chunk_size: int = 8):
    """Generator: read and reorder data from file."""
    with open(file_path, 'rb') as file:
        while True:
            chunk = file.read(chunk_size)
            if not chunk:
                break

            # Byte reordering logic
            reordered_chunk = [chunk[i:i + 2][::-1] for i in range(0, len(chunk), 2)]
            reordered_chunk = [
                reordered_chunk[i + 1] + reordered_chunk[i]
                for i in range(0, len(reordered_chunk) - 1, 2)
            ]
            yield b''.join(reordered_chunk)

# -----------------------------------------------------------------------------
# Main Transfer
# -----------------------------------------------------------------------------
def send_data_from_wic_file(file_path: str, channel: str, can_id: int,
                            small_delay: float, large_delay: float,
                            large_delay_threshold: int = 512):
    """Send data from binary file over CAN with pacing."""
    if not os.path.exists(file_path):
        logger.error(f"File not found: {file_path}")
        sys.exit(1)

    file_size = os.path.getsize(file_path)
    logger.info(f"Starting transfer of {file_size} bytes from {file_path}")

    total_bytes_sent = 0
    bytes_since_last_pause = 0

    try:
        bus = can.interface.Bus(channel=channel, bustype='socketcan')
    except OSError as e:
        logger.error(f"Failed to open CAN channel {channel}: {e}")
        sys.exit(1)

    with tqdm(total=file_size, unit='B', unit_scale=True, desc="Sending") as pbar:
        for data in read_data_from_file(file_path):
            send_can_message(bus, can_id, data)
            time.sleep(small_delay)
            total_bytes_sent += len(data)
            bytes_since_last_pause += len(data)
            pbar.update(len(data))

            if bytes_since_last_pause >= large_delay_threshold:
                time.sleep(large_delay)
                bytes_since_last_pause = 0

    logger.info(f"✅ Transfer complete. Total bytes sent: {total_bytes_sent}")

# -----------------------------------------------------------------------------
# Argument Parser
# -----------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(
        description="Send binary data (.wic, .bin) over CAN to an embedded target."
    )
    parser.add_argument("--file", required=True, help="Path to binary file.")
    parser.add_argument("--channel", default="can0", help="CAN channel (default: can0).")
    parser.add_argument("--can-id", type=lambda x: int(x, 0), default=0x00,
                        help="CAN ID in hex or decimal (default: 0x00).")
    parser.add_argument("--small-delay", type=float, default=0.0005,
                        help="Delay between frames in seconds (default: 0.0005).")
    parser.add_argument("--large-delay", type=float, default=0.02,
                        help="Longer delay after threshold bytes (default: 0.02).")
    parser.add_argument("--threshold", type=int, default=512,
                        help="Bytes to send before applying large delay (default: 512).")
    return parser.parse_args()

# -----------------------------------------------------------------------------
# Entry Point
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    args = parse_args()
    send_data_from_wic_file(
        file_path=args.file,
        channel=args.channel,
        can_id=args.can_id,
        small_delay=args.small_delay,
        large_delay=args.large_delay,
        large_delay_threshold=args.threshold
    )
