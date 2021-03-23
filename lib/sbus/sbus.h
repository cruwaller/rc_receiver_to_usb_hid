#pragma once

#include "helpers.h"
#include <stdint.h>


#define SBUS_RX_BAUDRATE 115200

#define SBUS_FAILSAFE_INACTIVE 0
#define SBUS_FAILSAFE_ACTIVE   1
#define SBUS_STARTBYTE         0x0f
#define SBUS_ENDBYTE           0x00

// OUT to flight controller
#define SBUS_MIN   172
#define SBUS_MID   992
#define SBUS_MAX   1811
#if SBUS_FAST
#define RX_BAUDRATE             200000
#else
#define RX_BAUDRATE             100000
#endif

typedef struct SbusChannels_s {
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
    uint8_t flags;
} PACKED SbusChannels_t;



uint8_t sbus_parse_byte(uint8_t inChar);
void sbus_get_rc_data(uint16_t * const rc_data, uint8_t len);
