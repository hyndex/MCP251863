#ifndef MCP251863_H
#define MCP251863_H

#include <Arduino.h>
#include <SPI.h>

#ifdef ARDUINO_ARCH_ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#endif

/* ======================
 *  MCP251863 Commands
 * ====================== */
#define MCP_CMD_RESET       0x00
#define MCP_CMD_READ        0x03
#define MCP_CMD_WRITE       0x02
#define MCP_CMD_READ_CRC    0x0B
#define MCP_CMD_WRITE_CRC   0x0A
#define MCP_CMD_WRITE_SAFE  0x0C

/* ======================
 *  Key Registers
 * ====================== */
#define REG_CiCON       0x000
#define REG_CiNBTCFG    0x004
#define REG_CiDBTCFG    0x008
#define REG_CiTDC       0x00C
#define REG_CiTSCON     0x014
#define REG_CiINT       0x01C
#define REG_CiRXIF      0x020
#define REG_CiTXIF      0x024
#define REG_CiTXREQ     0x030
#define REG_CiTREC      0x034
#define REG_CiBDIAG0    0x038
#define REG_CiBDIAG1    0x03C
#define REG_CiTEFCON    0x040
#define REG_CiTEFSTA    0x044
#define REG_CiTEFUA     0x048

// ECC and CRC
#define REG_ECCCON      0xE0C
#define REG_ECCSTAT     0xE10
#define REG_CRC         0xE08

// FIFO base
#define REG_FIFO_BASE   0x05C
static inline uint16_t MCP_FIFO_REG_ADDR(uint8_t num, uint8_t offset)
{
  // Each FIFO has 12 bytes (FIFOCON, FIFOSTA, FIFOUA)
  // FIFO1 => base + (1-1)*12=0 => 0x5C.., FIFO2 => base+12.., etc.
  return (REG_FIFO_BASE + 12*(num-1)) + offset;
}

/* ======================
 * Return Codes
 * ====================== */
enum MCP251863_Status {
    MCP251863_OK=0,
    MCP251863_FAIL,
    MCP251863_CRC_ERROR,
    MCP251863_SPI_ERROR,
    MCP251863_INVALID_PARAM,
    MCP251863_TIMEOUT,
    MCP251863_BUS_OFF,
    MCP251863_ECC_DBE, // example if a double-bit error is detected
    // etc.
};

/* ======================
 * Bus Status
 * ====================== */
struct MCP251863_BusStatus {
    bool txBusOff;
    bool txErrorPassive;
    bool rxErrorPassive;
    bool busWarning;
    uint8_t tec;
    uint8_t rec;
};

/* ======================
 * FIFO Config
 * ====================== */
struct MCP251863_FIFO_Config {
    bool isTx;
    uint8_t fifoSize;      // up to 32
    uint8_t payLoadSize;   // 8..64
    uint8_t txPriority;    // 0..31
    uint8_t txAttempts;    // used if CiCON.RTXAT=1
    bool timeStampEnable;  // for RX
};

/* ======================
 * CAN FD Message
 * ====================== */
struct MCP251863_MSG {
    bool extended;
    bool canFD;
    bool brs; 
    bool esi; 
    bool remote; 
    uint32_t id;   // 11-bit or 29-bit
    uint8_t dlc;   // 0..15
    uint8_t data[64];
    uint8_t dataLen; 
    uint32_t timeStamp;
};

/* ======================
 * Transmit Event FIFO (TEF)
 * ====================== */
struct MCP251863_TEF_Message {
    uint32_t id;
    bool extended;
    bool canFD;
    bool brs;
    bool esi;
    uint8_t dlc;
    uint32_t timeStamp;
};

/* ======================
 * Filter Config
 * ====================== */
struct MCP251863_FilterConfig {
    bool extended; 
    uint32_t id;    // ID
    uint32_t mask;  // bit mask
    uint8_t filterNum;
    uint8_t targetFIFO;
};


/* 
 * Production-Ready MCP251863 Driver
 */
class MCP251863
{
public:
    MCP251863(int8_t csPin, int8_t intPin=-1, int8_t stbyPin=-1);
    ~MCP251863();

    // Initialization
    MCP251863_Status begin(SPIClass &spi=SPI, uint32_t spiFreq=8000000);

    // Mode
    MCP251863_Status resetDevice();
    MCP251863_Status setConfigurationMode();
    MCP251863_Status setNormalMode();
    MCP251863_Status setSleepMode();
    MCP251863_Status setListenOnlyMode();

    // FIFO, TEF
    MCP251863_Status configureFIFO(uint8_t fifoNum, const MCP251863_FIFO_Config &cfg);
    MCP251863_Status enableTEF(uint8_t tefSize, bool timeStampEnable=false);
    MCP251863_Status readTEF(MCP251863_TEF_Message &m);

    // Acceptance Filters
    MCP251863_Status configureFilter(const MCP251863_FilterConfig &fc);

    // Bit timing
    MCP251863_Status configureBitTiming(uint32_t nominalBitrate,
                                        uint32_t dataBitrate,
                                        uint32_t sysClock=40000000);

    // ECC
    MCP251863_Status enableECC();
    MCP251863_Status disableECC();
    // check or handle double-bit ECC errors?
    MCP251863_Status checkECC();

    // Bus status
    MCP251863_Status getBusStatus(MCP251863_BusStatus &bs);

    // TX / RX
    MCP251863_Status transmitMessage(uint8_t txFifo, const MCP251863_MSG &m);
    MCP251863_Status receiveMessage(uint8_t rxFifo, MCP251863_MSG &m);

    // R/W registers
    MCP251863_Status writeRegister(uint16_t addr, uint32_t val);
    MCP251863_Status readRegister(uint16_t addr, uint32_t &val);
    MCP251863_Status writeRegisterCRC(uint16_t addr, uint32_t val);
    MCP251863_Status readRegisterCRC(uint16_t addr, uint32_t &val);

    // Interrupt
    MCP251863_Status getInterrupts(uint32_t &iflags);
    MCP251863_Status clearInterrupts(uint32_t mask);

    // Periodic
    void service();

    // Called from ISR
    void handleInterrupt();

private:
    // concurrency
#ifdef ARDUINO_ARCH_ESP32
    SemaphoreHandle_t _lock;
#endif
    void mutexLock();
    void mutexUnlock();

    bool _configured;
    SPIClass *_spi;
    int8_t _csPin;
    int8_t _intPin;
    int8_t _stbyPin;
    uint32_t _spiFreq;

    // Private helpers
    MCP251863_Status waitForOpMode(uint8_t target, uint32_t timeoutMs=500);
    MCP251863_Status updateFIFOUserAddress(uint8_t fifoNum, uint32_t &ua);

    // TEF
    MCP251863_Status getTEFUserAddress(uint32_t &ua);

    // SPI
    void spiBegin();
    void spiEnd();
    MCP251863_Status writeDataSPI(uint8_t *tx, uint8_t txLen,
                                  uint8_t *rx=nullptr, uint8_t rxLen=0);
    MCP251863_Status writeDataCRCSPI(uint8_t cmd, uint16_t addr,
                                     uint8_t *data, uint16_t dataLen);
    MCP251863_Status readDataCRCSPI(uint8_t cmd, uint16_t addr,
                                    uint8_t *data, uint16_t dataLen);
    uint16_t buildCmdWord(uint8_t cmd, uint16_t addr);
    bool checkCRCErrorFlag();
};


#endif // MCP251863_H
