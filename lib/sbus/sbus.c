#include "sbus.h"
#include "defines.h"


static uint16_t _channels[18];

static uint8_t buffer[25];
static uint8_t buffer_index = 0;


FAST_CODE_1 uint8_t sbus_parse_byte(uint8_t inChar)
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
    }
    return 1;
}

FAST_CODE_1 void sbus_get_rc_data(uint16_t * const rc_data, uint8_t len)
{
    uint8_t iter;
    for (iter = 0; iter < len && iter < ARRAY_SIZE(_channels); iter++) {
        rc_data[iter] = MAP_U16(_channels[iter],
            SBUS_MIN, SBUS_MAX, ANALOG_MIN, ANALOG_MAX);
    }
}
