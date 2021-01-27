#include "sbus.h"


static uint16_t _channels[18];

static uint8_t buffer[25];
static uint8_t buffer_index = 0;


uint8_t sbus_parse_byte(uint8_t inChar)
{
    if (!buffer_index && inChar != SBUS_STARTBYTE) {
        //incorrect start byte, out of sync
        return 0;
    }

    buffer[buffer_index++] = inChar;

    if (buffer_index == sizeof(buffer)) {
        buffer_index = 0;
        if (buffer[24] != SBUS_ENDBYTE) {
            //incorrect end byte, out of sync
            return 0;
        }

#if 0
        _channels[0]  = ((buffer[1]    |buffer[2]<<8)                 & 0x07FF);
        _channels[1]  = ((buffer[2]>>3 |buffer[3]<<5)                 & 0x07FF);
        _channels[2]  = ((buffer[3]>>6 |buffer[4]<<2 |buffer[5]<<10)  & 0x07FF);
        _channels[3]  = ((buffer[5]>>1 |buffer[6]<<7)                 & 0x07FF);
        _channels[4]  = ((buffer[6]>>4 |buffer[7]<<4)                 & 0x07FF);
        _channels[5]  = ((buffer[7]>>7 |buffer[8]<<1 |buffer[9]<<9)   & 0x07FF);
        _channels[6]  = ((buffer[9]>>2 |buffer[10]<<6)                & 0x07FF);
        _channels[7]  = ((buffer[10]>>5|buffer[11]<<3)                & 0x07FF);
        _channels[8]  = ((buffer[12]   |buffer[13]<<8)                & 0x07FF);
        _channels[9]  = ((buffer[13]>>3|buffer[14]<<5)                & 0x07FF);
        _channels[10] = ((buffer[14]>>6|buffer[15]<<2|buffer[16]<<10) & 0x07FF);
        _channels[11] = ((buffer[16]>>1|buffer[17]<<7)                & 0x07FF);
        _channels[12] = ((buffer[17]>>4|buffer[18]<<4)                & 0x07FF);
        _channels[13] = ((buffer[18]>>7|buffer[19]<<1|buffer[20]<<9)  & 0x07FF);
        _channels[14] = ((buffer[20]>>2|buffer[21]<<6)                & 0x07FF);
        _channels[15] = ((buffer[21]>>5|buffer[22]<<3)                & 0x07FF);

        _channels[16] = (buffer[23] & 0x1) ? 2047 : 0;
        _channels[17] = (buffer[23] & 0x2) ? 2047 : 0;
#else
        SbusChannels_t * channels = (SbusChannels_t*)&buffer[1];
        _channels[0] = channels->chan0;
        _channels[1] = channels->chan1;
        _channels[2] = channels->chan2;
        _channels[3] = channels->chan3;
        _channels[4] = channels->chan4;
        _channels[5] = channels->chan5;
        _channels[6] = channels->chan6;
        _channels[7] = channels->chan7;
        _channels[8] = channels->chan8;
        _channels[9] = channels->chan9;
        _channels[10] = channels->chan10;
        _channels[11] = channels->chan11;
        _channels[12] = channels->chan12;
        _channels[13] = channels->chan13;
        _channels[14] = channels->chan14;
        _channels[15] = channels->chan15;
#endif
    }
    return 1;
}

void sbus_get_rc_data(uint16_t * const rc_data, uint8_t len)
{
    uint8_t iter;
    for (iter = 0; iter < len && iter < ARRAY_SIZE(_channels); iter++) {
        rc_data[iter] = _channels[iter];
    }
}
