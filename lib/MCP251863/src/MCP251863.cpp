#include "MCP251863.h"

/*
  For CAN FD DLC -> actual length
*/
static const uint8_t DLCtoSize[16] = {
  0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64
};
// CRC polynomial for "CRC-16/USB"
static const uint16_t CRC_POLY = 0x8005;

/***********************************************************************
 * Constructor / Destructor
 ***********************************************************************/
MCP251863::MCP251863(int8_t csPin, int8_t intPin, int8_t stbyPin)
: _configured(false)
, _spi(nullptr)
, _csPin(csPin), _intPin(intPin), _stbyPin(stbyPin)
, _spiFreq(8000000)
{
#ifdef ARDUINO_ARCH_ESP32
    _lock = xSemaphoreCreateMutex();
#endif
}

MCP251863::~MCP251863()
{
#ifdef ARDUINO_ARCH_ESP32
    if(_lock) {
        vSemaphoreDelete(_lock);
        _lock=nullptr;
    }
#endif
}

/***********************************************************************
 * concurrency
 ***********************************************************************/
void MCP251863::mutexLock()
{
#ifdef ARDUINO_ARCH_ESP32
    if(_lock) {
        xSemaphoreTake(_lock, portMAX_DELAY);
    }
#endif
}

void MCP251863::mutexUnlock()
{
#ifdef ARDUINO_ARCH_ESP32
    if(_lock) {
        xSemaphoreGive(_lock);
    }
#endif
}

/***********************************************************************
 * begin
 ***********************************************************************/
MCP251863_Status MCP251863::begin(SPIClass &spi, uint32_t spiFreq)
{
    _spi=&spi;
    _spiFreq=spiFreq;

    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    if(_intPin>=0){
        pinMode(_intPin, INPUT_PULLUP);
    }
    if(_stbyPin>=0){
        pinMode(_stbyPin, OUTPUT);
        digitalWrite(_stbyPin, LOW); // normal
    }

    _spi->begin();

    // reset
    auto st=resetDevice();
    if(st!=MCP251863_OK) return st;
    delay(5);

    // enable ECC
    st=enableECC();
    if(st!=MCP251863_OK) return st;

    // set config
    st=setConfigurationMode();
    if(st!=MCP251863_OK) return st;

    // clear interrupts
    uint32_t tmp;
    readRegister(REG_CiINT, tmp);
    writeRegister(REG_CiINT, 0);

    _configured=true;
    return MCP251863_OK;
}

/***********************************************************************
 * resetDevice
 ***********************************************************************/
MCP251863_Status MCP251863::resetDevice()
{
    mutexLock();
    uint16_t cw= buildCmdWord(MCP_CMD_RESET, 0);
    uint8_t tx[2];
    tx[0]=(cw>>8)&0xFF;
    tx[1]=(cw)&0xFF;
    spiBegin();
    writeDataSPI(tx,2,nullptr,0);
    spiEnd();
    mutexUnlock();

    delay(2);
    return MCP251863_OK;
}

/***********************************************************************
 * setConfigurationMode
 ***********************************************************************/
MCP251863_Status MCP251863::setConfigurationMode()
{
    if(!_configured) return MCP251863_FAIL;
    // set REQOP=100 => bits [26..24]
    uint32_t cicon;
    readRegister(REG_CiCON,cicon);
    cicon &= ~(0x07<<24);
    cicon |= (4<<24);
    writeRegister(REG_CiCON,cicon);

    return waitForOpMode(4,500);
}

/***********************************************************************
 * setNormalMode
 ***********************************************************************/
MCP251863_Status MCP251863::setNormalMode()
{
    if(!_configured) return MCP251863_FAIL;
    uint32_t cicon;
    readRegister(REG_CiCON,cicon);
    cicon &= ~(0x07<<24);
    writeRegister(REG_CiCON,cicon);

    return waitForOpMode(0,500);
}

/***********************************************************************
 * setSleepMode
 ***********************************************************************/
MCP251863_Status MCP251863::setSleepMode()
{
    if(!_configured) return MCP251863_FAIL;
    uint32_t cicon;
    readRegister(REG_CiCON,cicon);
    cicon &= ~(0x07<<24);
    cicon |= (1<<24);
    writeRegister(REG_CiCON,cicon);

    return waitForOpMode(1,500);
}

/***********************************************************************
 * setListenOnlyMode
 ***********************************************************************/
MCP251863_Status MCP251863::setListenOnlyMode()
{
    if(!_configured) return MCP251863_FAIL;
    uint32_t cicon;
    readRegister(REG_CiCON,cicon);
    cicon &= ~(0x07<<24);
    cicon |= (3<<24);
    writeRegister(REG_CiCON,cicon);

    return waitForOpMode(3,500);
}

/***********************************************************************
 * waitForOpMode
 ***********************************************************************/
MCP251863_Status MCP251863::waitForOpMode(uint8_t target, uint32_t timeoutMs)
{
    uint32_t start=millis();
    while(true){
        uint32_t cicon;
        auto st= readRegister(REG_CiCON,cicon);
        if(st!=MCP251863_OK) return st;
        uint8_t opmod=(cicon>>21)&0x7;
        if(opmod==target) return MCP251863_OK;
        if(millis()-start>timeoutMs){
            return MCP251863_TIMEOUT;
        }
        delay(1);
    }
}

/***********************************************************************
 * writeRegister
 ***********************************************************************/
MCP251863_Status MCP251863::writeRegister(uint16_t addr, uint32_t val)
{
    if(!_configured) return MCP251863_FAIL;

    mutexLock();
    uint16_t cw= buildCmdWord(MCP_CMD_WRITE,addr);
    uint8_t tx[6];
    tx[0]=(cw>>8)&0xFF;
    tx[1]=(cw)&0xFF;
    tx[2]=(val>>24)&0xFF;
    tx[3]=(val>>16)&0xFF;
    tx[4]=(val>>8)&0xFF;
    tx[5]=(val)&0xFF;

    spiBegin();
    auto st= writeDataSPI(tx,6,nullptr,0);
    spiEnd();
    mutexUnlock();

    return st;
}

/***********************************************************************
 * readRegister
 ***********************************************************************/
MCP251863_Status MCP251863::readRegister(uint16_t addr, uint32_t &val)
{
    if(!_configured) return MCP251863_FAIL;
    mutexLock();

    uint16_t cw= buildCmdWord(MCP_CMD_READ,addr);
    uint8_t tx[2];
    tx[0]=(cw>>8)&0xFF;
    tx[1]=(cw)&0xFF;
    uint8_t rx[6]={0};

    spiBegin();
    auto st= writeDataSPI(tx,2, rx,6);
    spiEnd();
    mutexUnlock();
    if(st!=MCP251863_OK) return st;

    // data => rx[2..5]
    val=((uint32_t)rx[2]<<24)|((uint32_t)rx[3]<<16)|
        ((uint32_t)rx[4]<<8)|(rx[5]);
    return MCP251863_OK;
}

/***********************************************************************
 * writeRegisterCRC
 ***********************************************************************/
MCP251863_Status MCP251863::writeRegisterCRC(uint16_t addr, uint32_t val)
{
    if(!_configured) return MCP251863_FAIL;

    uint8_t data[4];
    data[0]=(val>>24)&0xFF;
    data[1]=(val>>16)&0xFF;
    data[2]=(val>>8)&0xFF;
    data[3]=(val)&0xFF;

    mutexLock();
    auto st= writeDataCRCSPI(MCP_CMD_WRITE_CRC, addr, data,4);
    if(st==MCP251863_OK){
        if(checkCRCErrorFlag()) st=MCP251863_CRC_ERROR;
    }
    mutexUnlock();

    return st;
}

/***********************************************************************
 * readRegisterCRC
 ***********************************************************************/
MCP251863_Status MCP251863::readRegisterCRC(uint16_t addr, uint32_t &val)
{
    if(!_configured) return MCP251863_FAIL;
    uint8_t data[4];

    mutexLock();
    auto st= readDataCRCSPI(MCP_CMD_READ_CRC, addr, data,4);
    if(st==MCP251863_OK){
        if(checkCRCErrorFlag()){
            st=MCP251863_CRC_ERROR;
        } else {
            val=((uint32_t)data[0]<<24)|((uint32_t)data[1]<<16)|
                ((uint32_t)data[2]<<8)| data[3];
        }
    }
    mutexUnlock();
    return st;
}

/***********************************************************************
 * configureFIFO
 ***********************************************************************/
MCP251863_Status MCP251863::configureFIFO(uint8_t fifoNum,const MCP251863_FIFO_Config &cfg)
{
    if(!_configured) return MCP251863_FAIL;
    if(fifoNum<1||fifoNum>31) return MCP251863_INVALID_PARAM;

    setConfigurationMode();

    uint16_t addr=MCP_FIFO_REG_ADDR(fifoNum,0);
    uint8_t pls=0;
    switch(cfg.payLoadSize){
      case 8:pls=0;break; case 12:pls=1;break; case 16:pls=2;break;
      case 20:pls=3;break; case 24:pls=4;break; case 32:pls=5;break;
      case 48:pls=6;break; case 64:pls=7;break;
      default:pls=0;break;
    }
    uint8_t fsz=(cfg.fifoSize>0)?(cfg.fifoSize-1):0;
    uint32_t val=0;
    // bits [31..29] => PLSIZE
    val |= ((uint32_t)pls &0x07)<<29;
    // bits [28..24] => FSIZE
    val |= ((uint32_t)fsz &0x1F)<<24;
    // bit7 => TXEN if TX
    if(cfg.isTx){
        val|=(1<<7);
        // if using restricted attempts from CiCON, set TXAT bits etc.
    } else {
        if(cfg.timeStampEnable){
            // bit5 => RXTSEN
            val|=(1<<5);
        }
    }

    return writeRegister(addr,val);
}

/***********************************************************************
 * enableTEF
 ***********************************************************************/
MCP251863_Status MCP251863::enableTEF(uint8_t tefSize, bool timeStampEnable)
{
    if(!_configured) return MCP251863_FAIL;
    setConfigurationMode();

    // bits [31..24]=FSIZE => tefSize-1
    // bit4 => TEFTSEN
    uint8_t fz=(tefSize>0)?(tefSize-1):0;
    uint32_t val=0;
    val|=((uint32_t)fz&0x1F)<<24;
    if(timeStampEnable){
      val|=(1<<4); // TEFTSEN
    }
    // optionally set other bits like TEFOVIE, TEFFIE, TEFHIE, TEFNEIE if you want
    // For simplicity, we keep them off or handle them via CiINT

    return writeRegister(REG_CiTEFCON, val);
}

/***********************************************************************
 * readTEF
 ***********************************************************************/
MCP251863_Status MCP251863::readTEF(MCP251863_TEF_Message &m)
{
    // check TEFSTA => bit0 => TEFNEIF => not empty
    uint32_t sta;
    readRegister(REG_CiTEFSTA,sta);
    if(!(sta&0x1)) return MCP251863_FAIL; // empty

    // read TEFUA => user address
    uint32_t ua;
    auto st= getTEFUserAddress(ua);
    if(st!=MCP251863_OK) return st;

    uint16_t readAddr=(uint16_t)(ua&0xFFFF);
    // we read 3 words => TE0, TE1, TE2 => total 12 bytes
    uint8_t rxBuf[15]={0}; // 2 +12 we do
    uint16_t cw=buildCmdWord(MCP_CMD_READ, readAddr);
    uint8_t txHdr[2];
    txHdr[0]=(cw>>8)&0xFF;
    txHdr[1]= (cw)&0xFF;

    mutexLock();
    spiBegin();
    writeDataSPI(txHdr,2,rxBuf,2+12);
    spiEnd();
    mutexUnlock();

    // parse
    uint32_t te0= ((uint32_t)rxBuf[2]<<24)|((uint32_t)rxBuf[3]<<16)|
                  ((uint32_t)rxBuf[4]<<8)| rxBuf[5];
    uint32_t te1= ((uint32_t)rxBuf[6]<<24)|((uint32_t)rxBuf[7]<<16)|
                  ((uint32_t)rxBuf[8]<<8)| rxBuf[9];
    uint32_t te2= ((uint32_t)rxBuf[10]<<24)|((uint32_t)rxBuf[11]<<16)|
                  ((uint32_t)rxBuf[12]<<8)| rxBuf[13];

    bool ext=((te1>>4)&1);
    m.extended=ext;
    if(ext){
      uint32_t EID=(te0 &0x1FFFF800)>>11;
      uint32_t SID= (te0 &0x7FF);
      m.id= (EID<<11)|SID;
    } else {
      m.id=(te0&0x7FF);
    }
    m.canFD=((te1>>7)&1);
    m.brs=((te1>>6)&1);
    m.esi=((te1>>8)&1);
    m.dlc=(te1&0xF);
    m.timeStamp= te2;

    // increment TEF => set UINC => bit8 in TEFCON
    uint32_t tefcon;
    readRegister(REG_CiTEFCON,tefcon);
    tefcon|=(1<<8);
    writeRegister(REG_CiTEFCON,tefcon);

    return MCP251863_OK;
}

/***********************************************************************
 * configureFilter
 ***********************************************************************/
MCP251863_Status MCP251863::configureFilter(const MCP251863_FilterConfig &fc)
{
    if(!_configured) return MCP251863_FAIL;
    if(fc.filterNum>31) return MCP251863_INVALID_PARAM;
    setConfigurationMode();

    // FilterN => base=0x1F0 => offset=8*N => FLTOBJ => +4 => MASK
    uint16_t fobjAddr= 0x1F0+(8*fc.filterNum);
    uint16_t maskAddr=fobjAddr+4;

    // minimal approach
    uint32_t fobj=0;
    uint32_t mobj=0;
    // if extended => set EXIDE => bit30 => EID in bits28..11 => SID in bits10..0
    if(fc.extended){
      uint32_t EID=(fc.id>>11)&0x3FFFF;
      uint32_t SID=(fc.id&0x7FF);
      fobj= ((1<<30) | (EID<<11) | SID);
      // mask => if we want to match extended only => set bit30 => MIDE=1
      uint32_t Emask=(fc.mask>>11)&0x3FFFF;
      uint32_t Smask=(fc.mask&0x7FF);
      mobj= ((1<<30)| (Emask<<11) | Smask);
    } else {
      // standard
      fobj= (fc.id&0x7FF);
      mobj= (fc.mask&0x7FF);
    }
    writeRegister(fobjAddr,fobj);
    writeRegister(maskAddr,mobj);

    // filter control => FLTCON0 => 0x1D0 => each 4 filters
    uint8_t wordIndex= fc.filterNum/4;
    uint8_t pos= fc.filterNum%4;
    uint16_t fltConAddr= 0x1D0+(4*wordIndex);

    uint32_t fltVal;
    readRegister(fltConAddr, fltVal);

    // each filter occupies 8 bits => shift=8*pos
    uint8_t shift= (8*pos);
    // we want FLTEN => bit7 =>1, FBP => bits4..0 => fc.targetFIFO
    // MIDE => bit6 => if extended only => optional
    uint8_t newByte= 0;
    newByte|= (1<<7); // FLTEN
    if(fc.extended){
      // MIDE => bit6 => match extended
      newByte|=(1<<6);
    }
    newByte|= (fc.targetFIFO &0x1F);

    // clear old
    fltVal &= ~(0xFF<<shift);
    fltVal |= ((uint32_t)newByte)<<shift;

    writeRegister(fltConAddr,fltVal);

    return MCP251863_OK;
}

/***********************************************************************
 * configureBitTiming
 ***********************************************************************/
MCP251863_Status MCP251863::configureBitTiming(uint32_t nominalBitrate,
                                               uint32_t dataBitrate,
                                               uint32_t sysClock)
{
    setConfigurationMode();
    // For example, 500k/2M on 40MHz
    if(nominalBitrate==500000 && dataBitrate==2000000 && sysClock==40000000){
        // e.g. NB => BRP=0, TSEG1=19, TSEG2=6, SJW=6
        uint32_t nb= ((0<<24)&0xFF000000)|((19<<16)&0x00FF0000)|
                     ((6<<8)&0x00007F00)|(6&0x7F);
        writeRegister(REG_CiNBTCFG, nb);

        // DB => BRP=0, TSEG1=9, TSEG2=2, SJW=2 => ~2Mbps
        uint32_t db= ((0<<24)&0xFF000000)|((9<<16)&0x01F0000)|
                     ((2<<8)&0x00000F00)|(2&0xF);
        writeRegister(REG_CiDBTCFG, db);
        // TDC if needed
        return MCP251863_OK;
    }
    // else let user do custom
    return MCP251863_INVALID_PARAM;
}

/***********************************************************************
 * enableECC
 ***********************************************************************/
MCP251863_Status MCP251863::enableECC()
{
    uint32_t v;
    auto st=readRegister(REG_ECCCON,v);
    if(st!=MCP251863_OK) return st;
    v|= 0x01; // ECCEN
    return writeRegister(REG_ECCCON,v);
}

/***********************************************************************
 * disableECC
 ***********************************************************************/
MCP251863_Status MCP251863::disableECC()
{
    uint32_t v;
    auto st=readRegister(REG_ECCCON,v);
    if(st!=MCP251863_OK) return st;
    v&= ~0x01;
    return writeRegister(REG_ECCCON,v);
}

/***********************************************************************
 * checkECC
 * Example: check ECCSTAT => if double-bit error => handle
 ***********************************************************************/
MCP251863_Status MCP251863::checkECC()
{
    uint32_t eccstat;
    auto st= readRegister(REG_ECCSTAT,eccstat);
    if(st!=MCP251863_OK) return st;
    bool ded=((eccstat>>2)&1);
    bool sec=((eccstat>>1)&1);
    if(ded){
      // double-bit error => uncorrectable
      return MCP251863_ECC_DBE;
    }
    if(sec){
      // single-bit corrected => might log or ignore
    }
    return MCP251863_OK;
}

/***********************************************************************
 * getBusStatus
 ***********************************************************************/
MCP251863_Status MCP251863::getBusStatus(MCP251863_BusStatus &bs)
{
    uint32_t val;
    auto st= readRegister(REG_CiTREC,val);
    if(st!=MCP251863_OK) return st;
    bs.txBusOff= ((val>>21)&1);
    bs.txErrorPassive= ((val>>20)&1);
    bs.rxErrorPassive= ((val>>19)&1);
    bool txwarn=((val>>18)&1);
    bool rxwarn=((val>>17)&1);
    bool ewarn=((val>>16)&1);
    bs.busWarning=(ewarn||txwarn||rxwarn);
    bs.tec= (val>>8)&0xFF;
    bs.rec= val &0xFF;

    return MCP251863_OK;
}

/***********************************************************************
 * transmitMessage
 ***********************************************************************/
MCP251863_Status MCP251863::transmitMessage(uint8_t txFifo, const MCP251863_MSG &m)
{
    if(!_configured) return MCP251863_FAIL;

    // get UA
    uint32_t ua;
    auto st=updateFIFOUserAddress(txFifo,ua);
    if(st!=MCP251863_OK) return st;

    // build T0,T1
    uint32_t t0=0,t1=0;
    if(m.extended){
      t1|=(1<<4); // IDE
      uint32_t EID=(m.id>>11)&0x3FFFF;
      uint32_t SID=(m.id&0x7FF);
      t0= ((EID<<11)&0x1FFFF800)|SID;
      t0|=(1<<29); 
    } else {
      t0=(m.id&0x7FF);
    }
    if(m.canFD) t1|=(1<<7);
    if(m.brs) t1|=(1<<6);
    if(m.esi) t1|=(1<<8);
    if(m.remote) t1|=(1<<5);
    t1|= (m.dlc&0xF);

    uint8_t actualLen= DLCtoSize[m.dlc<16?m.dlc:15];
    if(actualLen>64) actualLen=64;

    uint8_t outBuf[72]={0};
    outBuf[0]=(t0>>24)&0xFF;
    outBuf[1]=(t0>>16)&0xFF;
    outBuf[2]=(t0>>8)&0xFF;
    outBuf[3]=(t0)&0xFF;
    outBuf[4]=(t1>>24)&0xFF;
    outBuf[5]=(t1>>16)&0xFF;
    outBuf[6]=(t1>>8)&0xFF;
    outBuf[7]=(t1)&0xFF;
    for(int i=0;i<actualLen;i++){
      outBuf[8+i]=m.data[i];
    }
    uint16_t writeLen=8+actualLen;
    if(writeLen%4!=0) writeLen+=(4-(writeLen%4));

    mutexLock();
    {
      uint16_t cmdW= buildCmdWord(MCP_CMD_WRITE,(uint16_t)(ua&0xFFFF));
      uint8_t txHdr[2];
      txHdr[0]=(cmdW>>8)&0xFF;
      txHdr[1]=(cmdW)&0xFF;
      spiBegin();
      writeDataSPI(txHdr,2,nullptr,0);
      writeDataSPI(outBuf,writeLen,nullptr,0);
      spiEnd();
    }
    mutexUnlock();

    // set TXREQ => bit9
    uint16_t fifoCon=MCP_FIFO_REG_ADDR(txFifo,0);
    uint32_t fcVal;
    readRegister(fifoCon,fcVal);
    fcVal|=(1<<9);
    writeRegister(fifoCon,fcVal);

    return MCP251863_OK;
}

/***********************************************************************
 * receiveMessage
 ***********************************************************************/
MCP251863_Status MCP251863::receiveMessage(uint8_t rxFifo, MCP251863_MSG &m)
{
    if(!_configured) return MCP251863_FAIL;

    // check if not empty => FIFOSTA => bit0 => TFNRFNIF
    uint16_t staAddr= MCP_FIFO_REG_ADDR(rxFifo,4);
    uint32_t stVal;
    readRegister(staAddr, stVal);
    if(!(stVal&0x01)) return MCP251863_FAIL; // empty

    // UA
    uint32_t ua;
    auto st=updateFIFOUserAddress(rxFifo,ua);
    if(st!=MCP251863_OK) return st;

    uint8_t rxBuf[72]={0};
    uint16_t cmdW= buildCmdWord(MCP_CMD_READ, (uint16_t)(ua&0xFFFF));
    uint8_t txHdr[2];
    txHdr[0]=(cmdW>>8)&0xFF;
    txHdr[1]=(cmdW)&0xFF;

    mutexLock();
    spiBegin();
    writeDataSPI(txHdr,2, rxBuf,2+72);
    spiEnd();
    mutexUnlock();

    uint32_t t0= ((uint32_t)rxBuf[2]<<24)|((uint32_t)rxBuf[3]<<16)|
                 ((uint32_t)rxBuf[4]<<8)| rxBuf[5];
    uint32_t t1= ((uint32_t)rxBuf[6]<<24)|((uint32_t)rxBuf[7]<<16)|
                 ((uint32_t)rxBuf[8]<<8)| rxBuf[9];

    m.extended=((t1>>4)&1);
    m.canFD=((t1>>7)&1);
    m.brs=((t1>>6)&1);
    m.esi=((t1>>8)&1);
    m.remote=((t1>>5)&1);

    uint8_t dlc=(t1&0xF);
    m.dlc=dlc;
    uint8_t actualLen=DLCtoSize[dlc<16?dlc:15];
    if(actualLen>64) actualLen=64;

    if(m.extended){
      uint32_t EID=(t0&0x1FFFF800)>>11;
      uint32_t SID=(t0&0x7FF);
      m.id= (EID<<11)|SID;
    } else {
      m.id=(t0&0x7FF);
    }
    for(int i=0;i<actualLen;i++){
      m.data[i]= rxBuf[10+i];
    }
    m.dataLen=actualLen;
    m.timeStamp=0; // if we had RXTSEN => next word is time stamp

    // UINC => bit8
    uint16_t fifoCon=MCP_FIFO_REG_ADDR(rxFifo,0);
    uint32_t fcVal;
    readRegister(fifoCon,fcVal);
    fcVal|=(1<<8);
    writeRegister(fifoCon,fcVal);

    return MCP251863_OK;
}

/***********************************************************************
 * getInterrupts
 ***********************************************************************/
MCP251863_Status MCP251863::getInterrupts(uint32_t &iflags)
{
    return readRegister(REG_CiINT, iflags);
}

/***********************************************************************
 * clearInterrupts
 ***********************************************************************/
MCP251863_Status MCP251863::clearInterrupts(uint32_t mask)
{
    uint32_t val;
    auto st=readRegister(REG_CiINT,val);
    if(st!=MCP251863_OK) return st;
    val&= ~mask;
    return writeRegister(REG_CiINT,val);
}

/***********************************************************************
 * service
 * checks bus-off, ECC errors, etc.
 ***********************************************************************/
void MCP251863::service()
{
    // bus off
    MCP251863_BusStatus bs;
    if(getBusStatus(bs)==MCP251863_OK){
        if(bs.txBusOff){
            // reinit
            setConfigurationMode();
            // maybe wait or log
            setNormalMode();
        }
    }
    // check ECC => detect double-bit errors?
    checkECC();
}

/***********************************************************************
 * handleInterrupt
 ***********************************************************************/
void MCP251863::handleInterrupt()
{
    // minimal approach
    // typically, read CiINT => see if TEF, RX, error, etc.
    // parse, store frames in a ring buffer if you want
    // do minimal work
    uint32_t iflags;
    getInterrupts(iflags);

    // if TEF => read TEF
    // if((iflags & TEFNEIF?? ) => readTEF(...);

    // if RX => read FIFO or signal
    // if error => log?

    // then clear
    // clearInterrupts(iflags);
}

/***********************************************************************
 * updateFIFOUserAddress
 ***********************************************************************/
MCP251863_Status MCP251863::updateFIFOUserAddress(uint8_t fifoNum, uint32_t &ua)
{
    uint16_t uaAddr= MCP_FIFO_REG_ADDR(fifoNum,8);
    uint32_t val;
    auto st= readRegister(uaAddr,val);
    if(st!=MCP251863_OK) return st;
    ua=(val&0xFFFF);
    return MCP251863_OK;
}

/***********************************************************************
 * getTEFUserAddress
 ***********************************************************************/
MCP251863_Status MCP251863::getTEFUserAddress(uint32_t &ua)
{
    return readRegister(REG_CiTEFUA,ua);
}

/***********************************************************************
 * spiBegin / spiEnd
 ***********************************************************************/
void MCP251863::spiBegin()
{
    _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW);
}

void MCP251863::spiEnd()
{
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
}

/***********************************************************************
 * writeDataSPI
 ***********************************************************************/
MCP251863_Status MCP251863::writeDataSPI(uint8_t *tx, uint8_t txLen,
                                         uint8_t *rx, uint8_t rxLen)
{
    for(int i=0; i<txLen;i++){
        uint8_t rb=_spi->transfer(tx[i]);
        if(rx && i<rxLen){
          rx[i]=rb;
        }
    }
    for(int i=txLen;i<rxLen;i++){
        rx[i]=_spi->transfer(0x00);
    }
    return MCP251863_OK;
}

/***********************************************************************
 * writeDataCRCSPI
 ***********************************************************************/
MCP251863_Status MCP251863::writeDataCRCSPI(uint8_t cmd, uint16_t addr,
                                            uint8_t *data, uint16_t dataLen)
{
    if(dataLen>255) return MCP251863_INVALID_PARAM;

    uint8_t buf[270];
    uint16_t cw= buildCmdWord(cmd, addr);
    buf[0]=(cw>>8)&0xFF;
    buf[1]=(cw)&0xFF;
    buf[2]=(uint8_t)dataLen;
    memcpy(&buf[3], data, dataLen);

    // calc CRC
    uint16_t crc=0xFFFF;
    int total=3+dataLen;
    for(int i=0;i<total;i++){
      uint8_t b=buf[i];
      for(int bit=0;bit<8;bit++){
         bool d=(b&(1<<(7-bit)))!=0;
         bool c=(crc&0x8000)!=0;
         crc<<=1;
         if(d^c) crc^= CRC_POLY;
      }
    }
    buf[3+dataLen]=(crc>>8)&0xFF;
    buf[3+dataLen+1]=(crc)&0xFF;

    spiBegin();
    writeDataSPI(buf,3+dataLen+2,nullptr,0);
    spiEnd();

    return MCP251863_OK;
}

/***********************************************************************
 * readDataCRCSPI
 ***********************************************************************/
MCP251863_Status MCP251863::readDataCRCSPI(uint8_t cmd, uint16_t addr,
                                           uint8_t *data, uint16_t dataLen)
{
    if(dataLen>255) return MCP251863_INVALID_PARAM;
    uint8_t txb[3];
    uint16_t cw=buildCmdWord(cmd,addr);
    txb[0]=(cw>>8)&0xFF;
    txb[1]=(cw)&0xFF;
    txb[2]=(uint8_t)dataLen;

    uint8_t rx[3+dataLen+2]={0};

    spiBegin();
    writeDataSPI(txb,3, rx, 3+dataLen+2);
    spiEnd();

    memcpy(data, &rx[3], dataLen);
    return MCP251863_OK;
}

/***********************************************************************
 * buildCmdWord
 ***********************************************************************/
uint16_t MCP251863::buildCmdWord(uint8_t cmd, uint16_t addr)
{
    return ((cmd&0xF)<<12)|(addr&0xFFF);
}

/***********************************************************************
 * checkCRCErrorFlag
 ***********************************************************************/
bool MCP251863::checkCRCErrorFlag()
{
    uint32_t val;
    if(readRegister(REG_CRC,val)!=MCP251863_OK) return false;
    bool fErr=((val>>17)&1);
    bool cErr=((val>>16)&1);
    if(fErr||cErr){
        // clear
        uint32_t nVal= val & ~((1<<17)|(1<<16));
        writeRegister(REG_CRC,nVal);
    }
    return cErr;
}
