
# CAN-MMC Baremetal App + Python Transfer Tool

## Overview

This project allows transferring binary files (e.g. `.bin`, `.wic`) to the PolarFire SoC board over **CAN**.
It includes:

* **Baremetal firmware** running on `hart1 (U54_1)` — receives CAN data and writes it to eMMC.
* **Python script** — sends the binary file from the PC to the board via CAN.

---

## 1️⃣ Baremetal Application

**File:** `can_mmc.c`
**Processor:** `U54_1`
**Purpose:** Receive binary data over CAN and write it to the eMMC memory.

### Main Features

* Initializes UART, eMMC, and CAN0/CAN1.
* Receives 8-byte CAN frames, stores them in a 512-byte buffer.
* Performs eMMC write every 512 bytes.
* Logs all messages over UART (`115200 baud`).
* Supports primary and secondary eMMC selection.

### Build Notes

* Build from **SoftConsole** using your board configuration.
* Set the environment variable `BOARD` in:
  **Properties → C/C++ Build → Build Variables**
  Values: `os1`, `os2`, `icicle-kit`.
* No need to modify XML files manually anymore.

---

## 2️⃣ Python Tool

**File:** `01-bin_transfer.py`
**Purpose:** Send a binary file over CAN to the board.

### Requirements

```bash
pip install python-can tqdm
```

## Example Workflow

1. Build and flash the baremetal firmware.
2. Configure CAN on your PC:

   ```bash
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 up
   ```
3. Run the Python script to send the binary file:

   ```bash
   python3 01-bin_transfer.py --file firmware.bin --channel can0
   ```
4. Check UART logs for eMMC write confirmation.
