#include <Arduino.h>
#include "MCP251863.h"

/*
  Comprehensive example that uses:
   - TEF for TX completions
   - Acceptance filter
   - Multiple TX, RX FIFOs
   - Interruption-based concurrency
*/

static const int PIN_CS   = 10;
static const int PIN_INT  = 9;
static const int PIN_STBY = 8;

MCP251863 canDriver(PIN_CS, PIN_INT, PIN_STBY);

// For concurrency, we might store incoming frames in a ring buffer
// in the ISR or handleInterrupt(). We'll do a simple approach here.

void IRAM_ATTR onCANInterrupt()
{
  // minimal: call handleInterrupt() 
  // For advanced usage, just set a flag or queue
  canDriver.handleInterrupt();
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("MCP251863 Production Demo Start");

  // SPI pins
  SPI.begin(12,13,11,10);

  // attach interrupt
  pinMode(PIN_INT,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_INT), onCANInterrupt, FALLING);

  // initialize 
  auto st= canDriver.begin(SPI, 8000000);
  if(st!=MCP251863_OK){
    Serial.printf("Error init MCP251863: code=%d\n", st);
    while(1){ delay(1000); }
  }

  // Bit timing: 500k/2M on 40MHz
  st= canDriver.configureBitTiming(500000,2000000,40000000);
  if(st!=MCP251863_OK){
    Serial.printf("Bit timing fail: code=%d\n", st);
  }

  // TEF => 4 events, timestamps
  canDriver.enableTEF(4,true);

  // TX FIFO #1
  MCP251863_FIFO_Config tx1={true,4,64,1,0,false};
  canDriver.configureFIFO(1,tx1);

  // Another TX FIFO #2
  MCP251863_FIFO_Config tx2={true,2,64,2,0,false};
  canDriver.configureFIFO(2,tx2);

  // RX FIFO #3
  MCP251863_FIFO_Config rx3={false,8,64,0,0,true};
  canDriver.configureFIFO(3,rx3);

  // configure acceptance filter #0 => standard ID=0x123, 
  {
    MCP251863_FilterConfig fc;
    fc.extended=false;
    fc.id=0x123;
    fc.mask=0x7FF;
    fc.filterNum=0;
    fc.targetFIFO=3; 
    canDriver.configureFilter(fc);
  }

  // Another acceptance filter #1 => extended ID=0x123456
  {
    MCP251863_FilterConfig fc;
    fc.extended=true;
    fc.id=0x123456;
    fc.mask=0x1FFFFFFF;
    fc.filterNum=1;
    fc.targetFIFO=3;
    canDriver.configureFilter(fc);
  }

  // Normal mode
  st= canDriver.setNormalMode();
  if(st!=MCP251863_OK){
    Serial.printf("NormalMode fail: %d\n",st);
  }

  Serial.println("CAN driver ready in Normal Mode.");
}

uint32_t lastTx=0;
uint8_t toggle=0;

void loop()
{
  // Periodically send on TX FIFO #1
  if(millis()-lastTx>1000){
    MCP251863_MSG msg={0};
    msg.id=(toggle==0)?0x123:0x7A5;
    msg.extended=false;
    msg.canFD=true;
    msg.brs=true;
    msg.dlc=8; // => 8 bytes
    for(int i=0;i<8;i++){
      msg.data[i]= toggle*10+i;
    }
    msg.dataLen=8;
    auto st= canDriver.transmitMessage(1,msg);
    if(st==MCP251863_OK){
      Serial.printf("TXF1: ID=0x%X, 8 bytes\n", msg.id);
    } else {
      Serial.printf("TXF1 fail: %d\n", st);
    }
    toggle^=1;
    lastTx=millis();
  }

  // Also try TX FIFO #2
  if((millis()%1500)<50){
    // random
    MCP251863_MSG msg={0};
    msg.id=0x123456; // extended
    msg.extended=true;
    msg.canFD=true;
    msg.brs=true;
    msg.dlc=12; // =>16 bytes
    for(int i=0;i<16;i++){
      msg.data[i]= 0xA0+i;
    }
    msg.dataLen=16;
    auto st= canDriver.transmitMessage(2,msg);
    if(st==MCP251863_OK){
      Serial.println("TXF2: extended, 16 bytes");
    }
  }

  // Receive from FIFO #3
  {
    MCP251863_MSG rxm;
    while(canDriver.receiveMessage(3,rxm)==MCP251863_OK){
      Serial.printf("RXF3: ID=0x%X ext=%d dlc=%d\n", rxm.id, rxm.extended, rxm.dlc);
    }
  }

  // Read TEF => see if TX done
  {
    MCP251863_TEF_Message tef;
    while(canDriver.readTEF(tef)==MCP251863_OK){
      Serial.printf("TEF: ID=0x%X ext=%d dlc=%d, time=%u\n",
                    tef.id, tef.extended, tef.dlc, tef.timeStamp);
    }
  }

  // Service => check bus-off, ECC
  canDriver.service();

  delay(50);
}
