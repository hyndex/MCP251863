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
- Proper handling of the device’s **2 KB** message RAM, including advanced features like **ECC** (Error Correction Code)  
- Real-time concurrency and **interrupt-driven** approach for **high-speed** CAN FD up to 5 Mbps data phase

**NOTE**: Although this driver is quite complete, you must still test and tune it for your specific hardware setup, especially if you change bit timing, acceptance filters, concurrency, etc.

---

## Features

- **Multiple TX FIFOs** (with configurable payloads up to 64 bytes)  
- **One or more RX FIFOs** with optional timestamp capture  
- **Transmit Event FIFO (TEF)** for TX completion tracking and timestamps  
- **Acceptance Filters** (standard or extended ID)  
- **ECC** (enable or disable) plus single/double-bit error detection  
- **CRC-based** SPI commands (read/write)  
- **Bus-Off** detection and auto-reinit  
- **Mode transitions**: Configuration, Normal, Listen-Only, and Sleep  
- **Arduino** (PlatformIO) / **ESP-IDF** friendly  

---

## Requirements

### Hardware

1. **MCU**: ESP32, ESP32-S3, or similar with SPI interface.  
2. **MCP251863** or a pin-compatible device (MCP2517FD, MCP2518FD + ATA6563).  
3. **CAN Bus** with correct termination (120 Ω).  
4. **Oscillator** on MCP251863: typically 40 MHz for high bit rates.

### Software

1. **PlatformIO** or Arduino IDE with ESP32 board support.  
2. **FreeRTOS** semaphores (built into ESP32 Arduino) if you want concurrency.  
3. A working **SPI bus** on the pins you define (e.g. VSPI).

---

## Getting Started

### Install via PlatformIO

In your `platformio.ini`, reference **dikibhuyan/MCP251863@3.0.0** as a library dependency:
```ini
[env:esp32dev]
platform  = espressif32
board     = esp32dev
framework = arduino

lib_deps =
    dikibhuyan/MCP251863@3.0.0
```

Then the library will be automatically downloaded and included in your build.

### Wiring

1. **Connect** MCP251863 pins to your MCU SPI lines (SCK, MOSI, MISO, CS).  
2. **Set STBY pin** low for normal transceiver operation.  
3. (Optional) Hook up **INT** pin if you want interrupt-based reception.

### Example `main.cpp`

```cpp
#include <Arduino.h>
#include "MCP251863.h"

static const int CS_PIN   = 5;
static const int INT_PIN  = 4;
static const int STBY_PIN = 2;

MCP251863 canDriver(CS_PIN, INT_PIN, STBY_PIN);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("MCP251863 Demo Start");

  // For example, VSPI
  SPI.begin(18,19,23,5);

  // Initialize driver
  auto st= canDriver.begin(SPI, 8000000);
  if(st!=MCP251863_OK){
    Serial.printf("Error init: %d\n", st);
    while(1){ delay(1000); }
  }

  // 500k/2M at 40 MHz
  canDriver.configureBitTiming(500000,2000000,40000000);
  canDriver.setNormalMode();
  Serial.println("CAN FD Ready");
}

void loop() {
  // Periodic transmit
  static uint32_t lastTx=0;
  if(millis()-lastTx>1000){
    lastTx=millis();
    MCP251863_MSG tx={0};
    tx.id=0x123;
    tx.canFD=true;
    tx.brs=true;
    tx.dlc=8;
    for(int i=0;i<8;i++){
      tx.data[i]=i;
    }
    canDriver.transmitMessage(1,tx);
    Serial.println("Sent message");
  }

  // Check for RX
  MCP251863_MSG rx;
  while(canDriver.receiveMessage(2,rx)==MCP251863_OK){
    Serial.printf("RX: ID=0x%X, DLC=%d\n", rx.id, rx.dlc);
  }

  canDriver.service(); // check bus-off, ECC, etc.
  delay(50);
}
```

Then `pio run -t upload` to build and flash.

---

## Usage

### Basic Polling

Use `receiveMessage()` in your loop to fetch frames from a RX FIFO. If you’re not under intense traffic, polling is often sufficient.

### Interrupt-Driven

To enable higher throughput or real-time usage:

1. **Attach** your `INT` pin to an MCU GPIO.  
2. **Use** `attachInterrupt(digitalPinToInterrupt(INT_PIN), onCANInterrupt, FALLING);`  
3. **In** the ISR, call `canDriver.handleInterrupt()` or set a flag.

Inside `handleInterrupt()`, you can parse TEF, RX, or error flags, clearing them as needed.

---

## Project / Library Architecture

A typical layout using this library via **PlatformIO**:

```
MyProject/
 ├─ src/
 │   └─ main.cpp              (Your application code)
 ├─ platformio.ini
 └─ .pio/ ...
```

Where `platformio.ini` references `dikibhuyan/MCP251863@3.0.0` in `lib_deps`. The library and its `MCP251863.cpp` / `MCP251863.h` are automatically downloaded into `.pio/libdeps/...` by PlatformIO.

If you want to **develop** the library locally instead, you can place it in your project’s `lib/MCP251863` folder with a `library.json`, then remove `dikibhuyan/MCP251863@3.0.0` from `lib_deps`.

---

## Configuration & Concurrency

1. **Bit Timing**  
   - This library shows an example for 500k/2M with a 40 MHz oscillator. Adjust `configureBitTiming()` for your actual bus rates.  
2. **Concurrency**  
   - On ESP32, we use a FreeRTOS mutex for SPI calls (if present).  
   - On bare-metal or other MCUs, replace `mutexLock()` with a critical section or do single-thread usage.

---

## Supported Features

1. **CRC**-based SPI read/write (optional).  
2. **ECC** for single-bit error correction, double-bit error detection.  
3. **Acceptance Filters** for standard or extended IDs.  
4. **TEF** for TX event / timestamps.  
5. **Multiple TX/RX FIFOs**.  
6. **Bus-Off** auto-recovery in `service()`.

---

## Advanced Notes

1. **2 KB** RAM limit for TX/RX FIFO definitions plus TEF.  
2. For FD bit rates > 2 Mbps, consider carefully adjusting TDC (`CiTDC` register).  
3. Avoid heavy SPI calls in your ISR if traffic is high; prefer reading minimal flags, then queueing for a background task.

---

## License

This project is licensed under the **MIT License**. See [LICENSE](./LICENSE) for details.

---

**Enjoy using the MCP251863 for your next CAN FD project**—and remember to thoroughly **test** in your real hardware environment to ensure reliability. If you have questions, open an issue or submit a pull request!  