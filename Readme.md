# MCP251863 Production-Ready CAN FD Driver

[![Version](https://img.shields.io/badge/version-3.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-blue)]()
[![CAN FD](https://img.shields.io/badge/CAN-FD-orange.svg)]()

A **comprehensive** driver for the [MCP251863](https://www.microchip.com/en-us/product/MCP251863) chip (an **external CAN FD controller + integrated transceiver** from Microchip). This library is designed for **production-level usage** on ESP32-based systems (such as ESP32-S3) running the Arduino framework or ESP-IDF, featuring:

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
6. [Library Architecture](#library-architecture)  
7. [Configuration & Concurrency](#configuration--concurrency)  
8. [Supported Features](#supported-features)  
9. [Advanced Notes](#advanced-notes)  
10. [License](#license)

---

## Purpose

The **MCP251863** device combines a CAN FD controller with an on-board high-speed transceiver. It’s an ideal solution when your primary MCU (like the ESP32-S3) does not include a built-in CAN FD peripheral, or you need additional CAN FD channels.

This repository provides a **robust** implementation that covers almost all essential details for stable, production-grade operation:

- Reliable **SPI** usage with optional **CRC** verification  
- Proper handling of the device’s **2 KB** message RAM, including advanced features like **ECC** (Error Correction Code) for single-bit error correction  
- Real-time concurrency and **interrupt-driven** approach for **high-speed** CAN FD at up to 5 Mbps data phase

**NOTE**: Although this driver is quite complete, you must still test and tune it for your specific hardware setup, especially if you alter bit timing, acceptance filters, or concurrency patterns.

---

## Features

- **Multiple TX FIFOs** with configurable payload sizes (8 to 64 bytes)  
- **One or more RX FIFOs**, each with optional timestamping  
- **Transmit Event FIFO (TEF)** to track transmission completions and timestamps  
- **Acceptance Filters** for both standard and extended IDs  
- **ECC** control (enable/disable) and detection of single/double-bit errors  
- **CRC** read/write commands to protect against noisy SPI lines  
- **Bus-off** monitoring with automatic re-initialization  
- **Mode transitions** (Configuration, Normal, Listen-Only, Sleep) with timeouts  
- **Arduino** (PlatformIO) and **ESP-IDF** compatibility (with slight changes in concurrency)  

---

## Requirements

### Hardware

1. **Microcontroller**: ESP32 or ESP32-S3 (or other CPU with SPI).  
2. **MCP251863** or a pin-compatible device (e.g. other MCP25xxFD variants with integrated transceiver).  
3. **CAN Bus**: Proper termination (120 Ω) if using multiple nodes, typical CAN FD transceiver lines (CANH, CANL).  
4. **Oscillator**: 4, 20, or 40 MHz crystal or clock input for the MCP251863. The library’s example uses 40 MHz as an example.

### Software

1. **PlatformIO** or Arduino IDE with ESP32 board support (esp32/esp32s3).  
2. **FreeRTOS** semaphores if you want concurrency (built into ESP32’s Arduino environment).  
3. A working **SPI** bus (HSPI or VSPI on ESP32) mapped to the pins you configure.

---

## Getting Started

1. **Install** this library into your PlatformIO or Arduino environment.
   ```ini
   [env:mcp251863]
   platform = espressif32
   board = esp32dev
   framework = arduino
   lib_deps =
     dikibhuyan/MCP251863
   ```
2. **Connect** your MCP251863 (SCK, MOSI, MISO, CS) to the correct pins on your ESP32. Ensure STBY pin is driven LOW for normal transceiver operation.
3. **Include** the header and instantiate:
   ```cpp
   #include "MCP251863.h"
   MCP251863 canDriver(CS_PIN, INT_PIN, STBY_PIN);
   ```
4. **Initialize** in `setup()`:
   ```cpp
   SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
   canDriver.begin(SPI, 8000000);
   canDriver.configureBitTiming(500000, 2000000, 40000000);
   canDriver.setNormalMode();
   ```
5. **Send and receive** frames:
   ```cpp
   // Transmit
   MCP251863_MSG txMsg = { 
     .id = 0x123, .extended=false, .canFD=true, .brs=true, 
     .dlc=8, .dataLen=8
   };
   canDriver.transmitMessage(1, txMsg);

   // Receive
   MCP251863_MSG rxMsg;
   if(canDriver.receiveMessage(2, rxMsg)==MCP251863_OK){
     // handle message
   }
   ```

---

## Usage

### Basic Polling

If your application isn’t extremely high speed, you can poll for messages in the `loop()`:

```cpp
void loop() {
  // Poll for RX frames
  MCP251863_MSG rxMsg;
  while(canDriver.receiveMessage(RX_FIFO, rxMsg)==MCP251863_OK) {
     // process
  }

  // Periodic TX
  // ...
  
  // Check bus-off / ECC
  canDriver.service();
}
```

### Interrupt-Driven

For higher throughput:
1. **Attach** an interrupt to the MCP251863’s `INT` pin:
   ```cpp
   attachInterrupt(digitalPinToInterrupt(INT_PIN), onCANInterrupt, FALLING);
   ```
2. **ISR** calls `canDriver.handleInterrupt()`, which reads flags from `CiINT`, can read TEF or RX, etc. Or sets an event for a dedicated “CAN Task.”

### Acceptance Filters

Use `configureFilter()` to specify which frames land in which FIFO, either **standard** or **extended** IDs:

```cpp
MCP251863_FilterConfig fcfg;
fcfg.extended = false;
fcfg.id = 0x123;
fcfg.mask = 0x7FF;
fcfg.filterNum = 0;
fcfg.targetFIFO = RX_FIFO_NUM;
canDriver.configureFilter(fcfg);
```

### TEF (Transmit Event FIFO)

For each transmitted frame, a TEF record can be produced containing ID, DLC, and optional timestamp. Enable TEF with:

```cpp
canDriver.enableTEF(4, true); // 4 deep, timestamps enabled
```

Then read events in a loop:

```cpp
MCP251863_TEF_Message tefMsg;
while(canDriver.readTEF(tefMsg)==MCP251863_OK) {
  // indicates a completed transmission
}
```

---

## Library Architecture

```
MCP251863/
 ├─ library.json
 ├─ src/
 │   ├─ MCP251863.h      (driver class declarations)
 │   └─ MCP251863.cpp    (driver implementation)
 └─ examples/
     └─ ProductionFullDemo/
         └─ main.cpp    (full usage demonstration)
```

1. **MCP251863.h** declares the class, enumerations, data structures, etc.
2. **MCP251863.cpp** implements all register-level operations, FIFO management, concurrency hooks, etc.
3. **examples/** includes a “ProductionFullDemo” that shows advanced usage: multiple TX FIFOs, acceptance filters, TEF, concurrency, etc.

---

## Configuration & Concurrency

- **Bit Timing**: The library comes with a sample for 500k/2M (40 MHz). Customize `configureBitTiming()` or implement your own method for other speeds or clock sources.  
- **Concurrency**:  
  - By default, we wrap all SPI operations in `mutexLock()` / `mutexUnlock()`. On ESP32, these call FreeRTOS semaphores.  
  - For a bare-metal environment with no RTOS, you can remove or replace them with `noInterrupts()` / `interrupts()` calls.  

---

## Supported Features

1. **CRC**: We use `CMD_WRITE_CRC` / `CMD_READ_CRC` for robust SPI if you choose. Check `writeRegisterCRC()` / `readRegisterCRC()`.  
2. **ECC**: Single-bit corrections in message RAM, detect double-bit errors with `checkECC()`.  
3. **Bus-Off**: `service()` checks `txBusOff` and reinitializes automatically.  
4. **TX**: Any number of TX FIFOs (up to memory constraints).  
5. **RX**: FIFO-based reception, optional timestamping (`RXTSEN`).  
6. **Interrupt**: `handleInterrupt()` for ISR usage.  
7. **TEF**: enable with `enableTEF()`, read with `readTEF()`.  
8. **Acceptance Filters**: Up to 32 filters, each can route frames to a specific FIFO.

---

## Advanced Notes

1. **2 KB RAM Limit**  
   - Each FIFO requires space for its message objects. For example, if a FIFO can hold 4 messages of 64 bytes each, that’s 256 bytes + overhead. Make sure your combined usage doesn’t exceed 2 KB.  
2. **ECC Double-Bit Errors**  
   - If `checkECC()` returns `MCP251863_ECC_DBE`, it indicates an uncorrectable memory error. Decide how to handle it (e.g., log the event, reinit).  
3. **No Direct SPI in ISR**  
   - If your traffic is very high, we recommend you do minimal work in `onCANInterrupt()`—just set a flag or read minimal flags, then let a background task handle the full SPI read/writes.  
4. **Bit Timing**  
   - For robust FD operation at high data rates (e.g. 5 Mbps), carefully configure TDC in `CiTDC`. Our example uses the nominal approach for 2 Mbps data.

---

## License

This project is licensed under the **MIT License**. See [LICENSE](./LICENSE) for details.

---

**Enjoy using the MCP251863 for your next CAN FD project**—and remember to thoroughly **test** in your actual hardware + bus environment to ensure 100% reliability. If you encounter issues, open an issue or submit a pull request! 

*Happy hacking with CAN FD!* 

