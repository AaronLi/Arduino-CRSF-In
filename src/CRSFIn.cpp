#include "CRSFIn.h"
#include "Arduino.h"

CRSFIn::CRSFIn(){
    this->currentIndex = 0;
    this->frame_start_time = micros();
}

void CRSFIn::begin(HardwareSerial *port){
    this->port = port;
    this->port->begin(400000);
}

CRSFIn::~CRSFIn(){
    this->port->end();
}

bool CRSFIn::update(){
    if(this->port->available() > 0){
        if(micros() - this->frame_start_time > 1000){
            this->currentIndex = 0;
        }
        if(this->currentIndex == 0){
            this->frame_start_time = micros();
        }
        const int fullFrameLength = this->currentIndex < 3 ? 5 : min(this->crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH, CRSF_FRAME_SIZE_MAX);
        if(this->currentIndex < fullFrameLength){
            int numBytesAvailable = this->port->available();
            int numBytesRead = this->port->readBytes(&this->crsfFrame.bytes[currentIndex], numBytesAvailable);
            this->currentIndex += numBytesRead;
            if(this->currentIndex >= fullFrameLength){
                if(this->crsfFrame.frame.type == 0x16 && this->crsfFrame.bytes[fullFrameLength-1] == this->crsfFrameCRC()){
                crsfPayloadRcChannelsPacked_t *channels = (crsfPayloadRcChannelsPacked_t*)&this->crsfFrame.frame.payload;
                return true;
                }
            }
        }
    }
    return false;
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
    crsfPayloadRcChannelsPacked_t *channels = (crsfPayloadRcChannelsPacked_t*)&this->crsfFrame.frame.payload;
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