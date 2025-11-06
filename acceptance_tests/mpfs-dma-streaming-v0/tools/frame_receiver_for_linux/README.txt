Aquí tienes un **README.md** profesional, claro y documentado para tu `frame_receiver` en Linux 👇

---

# 🛰️ Frame Receiver (Linux Version)

This application captures image data packets over Ethernet and displays them in real time using **SFML**.
It was adapted from the Windows version of the same tool and cleaned up for Linux environments.

---

## 📖 Overview

The `frame_receiver` application:

* Captures Ethernet frames from a selected network interface using **libpcap**.
* Decodes image payloads based on a custom packet format.
* Reconstructs the image dynamically in memory.
* Displays it in a scalable window using **SFML (Simple and Fast Multimedia Library)**.
* Handles live updates in real-time to visualize image data streamed from a remote embedded device.

This is typically used in test setups such as **OrbSight DMA Streaming validation**.

---

## ⚙️ Dependencies

You’ll need the following packages installed on your Linux system:

```bash
sudo apt update
sudo apt install g++ libpcap-dev libsfml-dev
```

This installs:

* `libpcap` — for raw packet capture.
* `SFML` — for window management and image display.
* `pthread` — for multi-threading support (included by default).

---

## 🧩 Build Instructions

Compile using `g++` with the following command:

```bash
g++ -std=c++17 frame_receiver.cpp -o frame_receiver \
    -lsfml-graphics -lsfml-window -lsfml-system -lpcap -pthread
```

If you get an error like `cannot find -lsfml-graphics`, check your SFML installation or run:

```bash
sudo apt install libsfml-dev
```

---

## 🚀 Usage

1. Run the executable:

   ```bash
   ./frame_receiver
   ```

2. A list of network interfaces will be displayed, for example:

   ```
   1. enp10s0 (Ethernet interface)
   2. lo (Loopback)
   3. wlp2s0 (WiFi)
   ...
   ```

3. Enter the number corresponding to the interface receiving the image stream.

4. A window will open showing the live image received via Ethernet.

---

## 🧠 Notes

* The app uses `libpcap` in **promiscuous mode**, so root privileges may be required:

  ```bash
  sudo ./frame_receiver
  ```

* Image data is reconstructed chunk-by-chunk based on the following packet structure:

  ```cpp
  struct packet_t {
      uint8_t opcode;
      uint32_t length;
      uint32_t magic_code;   // must equal 0xb007c0de
      uint16_t width;
      uint16_t height;
      uint32_t crc;
      uint32_t chunk;
      uint32_t num_chunks;
      uint32_t buffer_size;
      uint8_t  buffer[1048];
  };
  ```

