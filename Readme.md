# MCP251863 Production-Ready CAN FD Driver

[![Version](https://img.shields.io/badge/version-3.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-blue)]()
[![CAN FD](https://img.shields.io/badge/CAN-FD-orange.svg)]()

A **comprehensive** driver for the [MCP251863](https://www.microchip.com/en-us/product/MCP251863) chip (an **external CAN FD controller + integrated transceiver** from Microchip). This library is designed for **production-level usage** on ESP32-based systems (e.g., ESP32-DevKitC and ESP32-S3) running the Arduino framework, featuring:

- **Full FIFO Management** (TX, RX, TEF)  
- **Acceptance Filters** (standard or extended)  
- **CRC-based** SPI read/write for robust communications  
- **ECC** for RAM error correction  
- **Interrupt-driven** concurrency support (with FreeRTOS semaphores)  
- **Bus-Off Detection** and re-initialization  
- **Bit Timing** example for 500 kbits/s nominal + 2 Mbits/s data at 40 MHz  
- **Advanced** usage hooks for TEF, acceptance filtering, error injection, etc.

---

## Table of Contents

1. [Purpose](#purpose)  
2. [Features](#features)  
3. [Requirements](#requirements)  
4. [Getting Started](#getting-started)  
5. [Usage](#usage)  
6. [Project / Library Architecture](#project--library-architecture)  
7. [Configuration & Concurrency](#configuration--concurrency)  
8. [Supported Features](#supported-features)  
9. [Advanced Notes](#advanced-notes)  
10. [License](#license)

---

## Purpose

The **MCP251863** device combines a CAN FD controller with an on-board high-speed transceiver. It’s an ideal solution when your primary MCU (like the ESP32-S3) does not include a built-in CAN FD peripheral, or you need additional CAN FD channels.

This repository provides a **robust** implementation that covers almost all essential details for stable, production-grade operation:

- Reliable **SPI** usage with optional **CRC** verification  
- Proper handling of the device’s **2 KB** message RAM, including advanced features like **ECC** (Error Correction Code)  
- Real-time concurrency and **interrupt-driven** approach for **high-speed** CAN FD up to 5 Mbps data phase

**NOTE**: Although this driver is quite complete, you must still test and tune it for your specific hardware setup, especially if you change bit timing, acceptance filters, concurrency, etc.

---

## Features

- **Multiple TX FIFOs** (with configurable payloads up to 64 bytes)  
- **One or more RX FIFOs** with optional timestamp capture  
- **Transmit Event FIFO (TEF)** for TX completion tracking and timestamps  
- **Acceptance Filters** (standard or extended ID)  
- **ECC** (enable or disable) plus single/double-bit error detection  
- **CRC-based** SPI commands (read/write) to combat noise on the SPI lines  
- **Bus-Off** detection and auto-reinit  
- **Mode transitions**: Configuration, Normal, Listen-Only, and Sleep  
- **Arduino** (PlatformIO) / **ESP-IDF** friendly  

---

## Requirements

### Hardware

1. **MCU**: ESP32, ESP32-S3, or similar with SPI interface.  
2. **MCP251863** or a pin-compatible device (e.g. MCP2517FD, MCP2518FD + ATA6563 transceiver).  
3. **CAN Bus** with correct termination (120 Ω).  
4. **Oscillator** on MCP251863: typically 40 MHz for high bit rates.

### Software

1. **PlatformIO** or Arduino IDE with ESP32 board support.  
2. **FreeRTOS** semaphores (available in ESP32 Arduino) if you want concurrency.  
3. A working **SPI bus** on the pins you define (e.g. VSPI).

---

## Getting Started

Below is a minimal **PlatformIO** configuration. Place it in your `platformio.ini`:

```ini
[platformio]
default_envs = esp32dev

[env:esp32dev]
platform  = espressif32
board     = esp32dev
framework = arduino

[env:esp32s3]
platform  = espressif32
board     = esp32-s3-devkitc-1
framework = arduino
```

Then, ensure the library code is in `lib/MCP251863`, and your main application code is in `src/main.cpp`, for example:

```cpp
#include <Arduino.h>
#include "MCP251863.h"

MCP251863 canDriver(5, 4, 2);  // cs, int, stby pins

void setup() {
  Serial.begin(115200);

  SPI.begin(18, 19, 23, 5);
  canDriver.begin(SPI, 8000000);

  // Configure bit timing for 500k/2M at 40MHz
  canDriver.configureBitTiming(500000, 2000000, 40000000);
  canDriver.setNormalMode();
}

void loop() {
  // Periodically transmit
  static uint32_t lastTx=0;
  if (millis()-lastTx > 1000) {
    lastTx=millis();
    MCP251863_MSG txMsg={0};
    txMsg.id=0x123;
    txMsg.canFD=true;
    txMsg.brs=true;
    txMsg.dlc=8;
    for(int i=0; i<8; i++){
      txMsg.data[i]=i;
    }
    canDriver.transmitMessage(1, txMsg);
    Serial.println("Message sent");
  }

  // Check for received frames
  MCP251863_MSG rxMsg;
  while (canDriver.receiveMessage(2, rxMsg)==MCP251863_OK) {
    Serial.print("Received: ID=0x");
    Serial.println(rxMsg.id,HEX);
  }

  canDriver.service(); // check bus-off, ECC, etc.
  delay(50);
}
```

Finally, **Build** and **Upload** via PlatformIO, ensuring you’ve selected `esp32dev` or `esp32s3` environment.

---

## Usage

### Basic Polling

You can poll for incoming frames in `loop()` using:

```cpp
MCP251863_MSG rxMsg;
while (canDriver.receiveMessage(RX_FIFO, rxMsg)==MCP251863_OK) {
  // handle rxMsg
}
```

Or periodically send messages:

```cpp
MCP251863_MSG txMsg = {/*...*/};
canDriver.transmitMessage(TX_FIFO, txMsg);
```

### Interrupt-Driven Approach

For higher throughput or lower latency:

1. Hook the **INT** pin on the MCP251863 to an ESP32 GPIO.  
2. `attachInterrupt(digitalPinToInterrupt(INT_PIN), onCANInterrupt, FALLING);`  
3. In your ISR, call `canDriver.handleInterrupt();` or set a flag to do so in a background task.

Inside `handleInterrupt()`, you can check TEF or RX flags, read the FIFO, etc.

### TEF (Transmit Event FIFO)

Enable with:

```cpp
canDriver.enableTEF(4, true); // up to 4 TEF entries, with timestamps
```

Then read events:

```cpp
MCP251863_TEF_Message tef;
while(canDriver.readTEF(tef) == MCP251863_OK) {
   // handle transmit completion
}
```

---

## Project / Library Architecture

Your top-level project layout might look like:

```
MyProject/
 ├─ .pio/
 ├─ src/
 │   └─ main.cpp                 (Your application code)
 ├─ lib/
 │   └─ MCP251863/
 │       ├─ library.json         (Manifest)
 │       ├─ include/
 │       │   └─ MCP251863.h
 │       ├─ src/
 │       │   └─ MCP251863.cpp
 │       └─ examples/
 │           ├─ ProductionExample/
 │           └─ ...
 ├─ platformio.ini
 └─ ...
```

**Key points**:

- `src/main.cpp` is your main application with `setup()` and `loop()`.  
- The library in `lib/MCP251863` has `library.json`, `MCP251863.h`, and `MCP251863.cpp`.  
- Additional library examples remain in `lib/MCP251863/examples/` for reference.

---

## Configuration & Concurrency

1. **Bit Timing**  
   - A default example for 500 kbps nominal / 2 Mbps data (40 MHz oscillator) is included. Adjust via `configureBitTiming()`.  
   - For advanced usage at higher data rates, set TDC in `CiTDC`.

2. **Concurrency**  
   - The library’s SPI calls are protected by `mutexLock()`/`mutexUnlock()` on ESP32, which uses a FreeRTOS semaphore if available.  
   - If using a bare-metal or non-RTOS environment, replace those calls with `noInterrupts()` / `interrupts()` or other concurrency methods.

---

## Supported Features

1. **CRC**: Use `CMD_WRITE_CRC` / `CMD_READ_CRC` to verify data on noisy lines.  
2. **ECC**: Single-bit correction in message RAM; detect double-bit errors with `checkECC()`.  
3. **Bus-Off**: The driver automatically re-initializes when it detects bus-off in `canDriver.service()`.  
4. **Accept Filters**: Up to 32 filters (standard/extended).  
5. **TEF**: Transmit Event FIFO for time-stamped TX completions.  
6. **Sleep & Listen-Only**: Provided by `setSleepMode()`, `setListenOnlyMode()`.

---

## Advanced Notes

1. **2 KB of Message RAM**  
   - You must be mindful of how many FIFOs, TEF entries, and payload sizes you configure. Summed usage can’t exceed 2 KB.  
2. **Interrupt**  
   - For maximum throughput, do minimal SPI calls in the ISR. Often, we just read the interrupt flags, then handle the details in a lower-priority task.  
3. **ECC Double-Bit Error**  
   - If `checkECC()` returns `MCP251863_ECC_DBE`, it indicates a **non-correctable** error in RAM. Decide how to handle (reset or log).  
4. **Bit Timing**  
   - For truly high speeds (e.g., 5 Mbps data), tune `CiDBTCFG` and `CiTDC` carefully.

---

## License

This project is licensed under the **MIT License**. See [LICENSE](./LICENSE) for details.

---

**Happy hacking with MCP251863 and CAN FD!**  

Feel free to open issues or pull requests if you discover any problems or want to add new features.