#include "CRSFIn.h"
#include "Arduino.h"

CRSFIn::CRSFIn(){
    this->currentIndex = 0;
    this->frame_start_time = micros();
}

void CRSFIn::begin(Uart *port){
    this->port = port;
    this->port->begin(420000);
    this->us_per_packet = 1750;
}

CRSFIn::~CRSFIn(){
    this->port->end();
}

bool CRSFIn::update(){
    int available = this->port->available();
    if(micros() - this->frame_start_time > this->us_per_packet){
            this->currentIndex = 0;
    }
    if(this->currentIndex == 0){
        this->frame_start_time = micros();
    }
    if(available > 0){
        const int fullFrameLength = this->currentIndex < 2 ? 5 : min(this->crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH, CRSF_FRAME_SIZE_MAX);
        if(this->currentIndex < fullFrameLength){
            int numBytesRead = this->port->readBytes(&this->crsfFrame.bytes[currentIndex], min(available, CRSF_FRAME_SIZE_MAX - this->currentIndex));
            this->currentIndex += numBytesRead;
            if(this->currentIndex >= fullFrameLength && fullFrameLength > 5){
                this->currentIndex = 0;
                this->frame_start_time = micros();
                if(this->crsfFrame.frame.type == 0x16 && this->crsfFrame.bytes[fullFrameLength-1] == this->crsfFrameCRC()){
                    memcpy(&this->outputFrame, &this->crsfFrame, CRSF_FRAME_SIZE_MAX);
                    this->last_frame_timestamp = millis();
                return true;
                }
            }
        }
    }
    return false;
}

void CRSFIn::updateFast(){
    uint32_t start_time = micros();
    if(start_time - this->frame_start_time > this->us_per_packet){
        this->currentIndex = 0;
        this->frame_start_time = start_time;
    }
    const int fullFrameLength =  this->currentIndex < 3 ? 5 : min(this->crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH, CRSF_FRAME_SIZE_MAX);
    if(this->currentIndex < fullFrameLength){
        this->crsfFrame.bytes[this->currentIndex++] = this->port->read();
        if(this->currentIndex >= fullFrameLength){
            this->currentIndex = 0;
            this->frame_start_time = micros();
            if(this->crsfFrame.bytes[fullFrameLength-1] == this->crsfFrameCRC() && this->crsfFrame.frame.type == 0x16){
                memcpy(&this->outputFrame, &this->crsfFrame, CRSF_FRAME_SIZE_MAX);
                this->last_frame_timestamp = millis();
            }
        }
    }
}

// Use only this or update, not both
void CRSFIn::IrqHandler() {
    this->port->IrqHandler();
    while(this->port->available()){
        this->updateFast();
    }
}

uint8_t CRSFIn::crc8_dvb_s2(uint8_t crc, unsigned char a){
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

uint8_t CRSFIn::crsfFrameCRC(){
    // CRC includes type and payload
    uint8_t crc = crc8_dvb_s2(0, this->crsfFrame.frame.type);
    for (int ii = 0; ii < this->crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; ++ii) {
        crc = crc8_dvb_s2(crc, this->crsfFrame.frame.payload[ii]);
    }
    return crc;
}

unsigned int CRSFIn::getChannelRaw(unsigned int channel){
    crsfPayloadRcChannelsPacked_t *channels = (crsfPayloadRcChannelsPacked_t*)&this->outputFrame.frame.payload;
    switch(channel){
        case 0:
            return channels->chan0;
        case 1:
            return channels->chan1;
        case 2:
            return channels->chan2;
        case 3:
            return channels->chan3;
        case 4:
            return channels->chan4;
        case 5:
            return channels->chan5;
        case 6:
            return channels->chan6;
        case 7:
            return channels->chan7;
        case 8:
            return channels->chan8;
        case 9:
            return channels->chan9;
        case 10:
            return channels->chan10;
        case 11:
            return channels->chan11;
        case 12:
            return channels->chan12;
        case 13:
            return channels->chan13;
        case 14:
            return channels->chan14;
        case 15:
            return channels->chan15;
        default:
            return 0;
    }

}

float CRSFIn::getChannelFloat(unsigned int channel){
    float val = (float)this->getChannelRaw(channel);
    return (val - 172.f) * (1.f/1637.f);
}


void CRSFIn::transmitGpsFrame(crsfGpsFrame_t &info){
    uint8_t buffer[17];
    buffer[0] = 17;
    buffer[1] = 0x02;
    writeU32BigEndian(&buffer[2], info.latitude);
    writeU32BigEndian(&buffer[6], info.longitude);
    writeU16BigEndian(&buffer[10], info.groundSpeed);
    writeU16BigEndian(&buffer[12], info.heading);
    writeU16BigEndian(&buffer[14], info.altitude);
    buffer[16] = info.satellites;
    uint8_t crc = 0;
    for(int i = 0; i<16; i++){
        crc = crc8_dvb_s2(crc, buffer[i+1]);
    }
    this->port->write(0xC8);
    this->port->write(buffer, 17);        
    this->port->write(crc);
}

void CRSFIn::transmitVoltageFrame(crsfVoltageFrame_t &info){
    uint8_t buffer[10];
    buffer[0] = 10;
    buffer[1] = 0x08;
    writeU16BigEndian(&buffer[2], info.voltage);
    writeU16BigEndian(&buffer[4], info.current);
    writeU24BigEndian(&buffer[6], info.fuel);
    buffer[9] = info.battery;   
    uint8_t crc = 0;
    for(int i = 0; i<9; i++){
        crc = crc8_dvb_s2(crc, buffer[i+1]);
    }
    this->port->write(0xC8);
    this->port->write(buffer, 10);
    this->port->write(crc);
}

void writeU32BigEndian(uint8_t *buffer, uint32_t value){
    buffer[0] = (value >> 24) & 0xFF;
    buffer[1] = (value >> 16) & 0xFF;
    buffer[2] = (value >> 8) & 0xFF;
    buffer[3] = value & 0xFF;
}

void writeU24BigEndian(uint8_t *buffer, uint32_t value){
    buffer[0] = (value >> 16) & 0xFF;
    buffer[1] = (value >> 8) & 0xFF;
    buffer[2] = value & 0xFF;
}

void writeU16BigEndian(uint8_t *buffer, uint16_t value){
    buffer[0] = (value >> 8) & 0xFF;
    buffer[1] = value & 0xFF;
}