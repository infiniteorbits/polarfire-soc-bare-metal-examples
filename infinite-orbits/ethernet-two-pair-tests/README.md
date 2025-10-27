# 📡 Ethernet File Transfer Setup (Icicle Kit / Engineering Model)

This project implements **raw Ethernet file transfer** between a host PC (remote server) and the Icicle Kit or Engineering Model (EM).
The system transfers binary files (e.g. Linux images) directly to eMMC over Ethernet using a lightweight custom protocol.

---

## ⚙️ 1. Adjust the Python Script (`ethernet_bin_transfer.py`)

Edit the configuration section at the top of the file:

```python
# ============================
# CONFIGURATION
# ============================

IFACE = "enx0011226846b7"             # Network interface (remote server)
SRC_MAC = b'\x00\x11\x22\x68\x46\xb7' # PC MAC (remote server)

FILE_PATH = "core-image-minimal-dev-icicle-kit-es-amp-20250730092842.rootfs.wic"  # File to send
```

### 🧠 Notes

* `IFACE`: network interface name on your PC (use `ip link show` to list available interfaces).
* `SRC_MAC`: your PC’s MAC address (e.g. `ip link show enx0011226846b7`).
* `FILE_PATH`: path to the file you want to transfer (typically a `.wic` image).

---

## ⚙️ 2. Adjust Constants in the Firmware (`e51.c`)

Open `src/application/hart0/e51.c` and verify the following defines:

```c
#define ICICLE_KIT              0   // Keep 0 for Engineering Model (EM)      [line 40]
#define START_ADDRESS           0   // eMMC block offset (must be multiple of 512 bytes) [line 49]
```

### 🧠 Notes

* `ICICLE_KIT = 0` → for **Engineering Model (EM)** configuration.
  Set to `1` when compiling for **Icicle Kit** hardware.
* `START_ADDRESS` specifies the **starting eMMC block offset**.

  * `0` → start writing at eMMC block 0.
  * `1` → offset = 512 bytes.
  * `2` → offset = 1024 bytes, etc.

---

## ⚙️ 3. Set the Destination MAC in `send_confirmation_frame()`

Inside the same file (`e51.c`), in the function `send_confirmation_frame()`, configure the destination MAC address of your PC:

```c
uint8_t dest_mac[6] = {0x00, 0x11, 0x22, 0x68, 0x46, 0xb7}; // MAC of your PC  [line 91]
```

### 🧠 Notes

* This ensures that the **“SAVE_COMPLETED” acknowledgment frame** is sent back to your PC.
* Must match the `SRC_MAC` value in the Python script.

---

## ⚙️ 4. Build and Flash the Firmware

1. Open the project in **SoftConsole**.
2. Select the **correct target** (E51 baremetal).
3. Build the application.
4. Flash or run it on your board using the debugger.

---

## 🧪 5. Serial Output Verification

After loading and running the firmware, open the **UART console** (115200 baud).
You should see the following messages printed:

```
ethernet Init status: 0 
eMMC Init status: 0 
eMMC erase status: 5 
```

✅ These lines confirm:

* Ethernet MAC successfully initialized.
* eMMC initialized correctly.
* The eMMC erase command has completed successfully.

Once you see this output, the board is **ready to receive data** via Ethernet.

---

## ▶️ 6. Start the File Transfer

On your PC, run:

```bash
sudo python3 ethernet_bin_transfer.py
```

The script will:

* Send a **START** frame to the board.
* Transmit the file in fixed-size chunks.
* Pause every few megabytes to wait for the board’s **SAVE_COMPLETED** acknowledgment.
* Resume until the transfer is complete.

During the process, progress and transfer rate will be displayed in your terminal.

---



## ✅ Summary of Parameters

| Parameter       | Location                               | Example Value                                                          | Description                    |
| --------------- | -------------------------------------- | ---------------------------------------------------------------------- | ------------------------------ |
| `IFACE`         | `ethernet_bin_transfer.py`             | `"enx0011226846b7"`                                                    | Network interface name         |
| `SRC_MAC`       | `ethernet_bin_transfer.py`             | `b'\x00\x11\x22\x68\x46\xb7'`                                          | MAC address of PC              |
| `FILE_PATH`     | `ethernet_bin_transfer.py`             | `"core-image-minimal-dev-icicle-kit-es-amp-20250730092842.rootfs.wic"` | File to send                   |
| `ICICLE_KIT`    | `e51.c`                                | `0`                                                                    | 0 for EM, 1 for Icicle Kit     |
| `START_ADDRESS` | `e51.c`                                | `0`                                                                    | eMMC block offset (×512 bytes) |
| `dest_mac[]`    | `send_confirmation_frame()` in `e51.c` | `{0x00, 0x11, 0x22, 0x68, 0x46, 0xb7}`                                 | Destination MAC (your PC)      |

