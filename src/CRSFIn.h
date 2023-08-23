#ifndef CRSF_IN
#define CRSF_IN

#include <Arduino.h>


#define CRSF_PAYLOAD_SIZE_MAX   32 // !!TODO needs checking
#define CRSF_FRAME_SIZE_MAX     (CRSF_PAYLOAD_SIZE_MAX + 4)


enum {
    CRSF_FRAME_GPS_PAYLOAD_SIZE = 15,
    CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
    CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE = 10,
    CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE = 22, // 11 bits per channel * 16 channels = 22 bytes.
    CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE = 6,
    CRSF_FRAME_LENGTH_ADDRESS = 1, // length of ADDRESS field
    CRSF_FRAME_LENGTH_FRAMELENGTH = 1, // length of FRAMELENGTH field
    CRSF_FRAME_LENGTH_TYPE = 1, // length of TYPE field
    CRSF_FRAME_LENGTH_CRC = 1, // length of CRC field
    CRSF_FRAME_LENGTH_TYPE_CRC = 2 // length of TYPE and CRC fields combined
};

struct crsfPayloadRcChannelsPacked_s {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
} __attribute__ ((__packed__));

typedef struct crsfPayloadRcChannelsPacked_s crsfPayloadRcChannelsPacked_t;

typedef struct crsfFrameDef_s {
    uint8_t deviceAddress;
    uint8_t frameLength;
    uint8_t type;
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1]; // +1 for CRC at end of payload
} crsfFrameDef_t;

typedef union crsfFrame_u {
    uint8_t bytes[CRSF_FRAME_SIZE_MAX];
    crsfFrameDef_t frame;
} crsfFrame_t;

typedef struct crsfGpsInfo {
    int32_t  latitude; //      Latitude ( degree / 10`000`000 )
    int32_t  longitude; //     Longitude (degree / 10`000`000 )
    uint16_t groundSpeed; //   Groundspeed ( km/h / 10 )
    uint16_t heading; //       GPS heading ( degree / 100 )
    uint16_t   altitude; //      Altitude ( meter ­1000m offset )
    uint8_t  satellites; //    Satellites in use ( counter )
} crsfGpsFrame_t;

typedef struct crsfVoltageInfo {
    uint16_t voltage; //    Voltage ( mV * 100 )
    uint16_t current; //   Current ( mA * 100 )
    uint32_t fuel; //   Fuel ( drawn mAh )
    uint8_t battery; //     Battery remaining ( percent )
} crsfVoltageFrame_t;

void writeU32BigEndian(uint8_t *buffer, uint32_t value);
void writeU24BigEndian(uint8_t *buffer, uint32_t value);
void writeU16BigEndian(uint8_t *buffer, uint16_t value);

class CRSFIn {
    public:
        uint32_t last_frame_timestamp;

        CRSFIn();
        ~CRSFIn();
        void begin(Uart *serial);
        bool update();
        unsigned int getChannelRaw(unsigned int channel);
        float getChannelFloat(unsigned int channel);
        void transmitGpsFrame(crsfGpsFrame_t &info);
        void transmitVoltageFrame(crsfVoltageFrame_t &info);
        void IrqHandler();
    private:
        Uart * port;
        crsfFrame_t crsfFrame;
        crsfFrame_t outputFrame;
        uint8_t currentIndex;
        uint32_t frame_start_time;

        
        static uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a);
        uint8_t crsfFrameCRC(void);
        void updateFast();
};

#endif
