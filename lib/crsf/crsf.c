#include "crsf.h"
#include "crc.h"
#include "defines.h"
#include <stddef.h>

static uint8_t SerialInBuffer[256];
static uint8_t CRSFframeActive;
static uint8_t SerialInCrc;
static uint8_t SerialInPacketStart;
static uint8_t SerialInPacketLen;      // length of the CRSF packet as measured
static uint8_t SerialInPacketPtr;      // index where we are reading/writing

#if PROTO_ELRS
#define MAX_CHAHNELS 12
#else
#define MAX_CHAHNELS 16
#endif
static uint16_t _channels[MAX_CHAHNELS];


static FAST_CODE_1 uint8_t crsf_check_msg(uint8_t const * const input)
{
    switch (input[0]) {
#if PROTO_ELRS
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED_ELRS: {
            elrs_channels_t * channels = (elrs_channels_t*)&input[1];
            _channels[0] = channels->analog0;
            _channels[1] = channels->analog1;
            _channels[2] = channels->analog2;
            _channels[3] = channels->analog3;
            _channels[4] = channels->aux4;
            _channels[5] = channels->aux5;
            _channels[6] = channels->aux6;
            _channels[7] = channels->aux7;
            _channels[8] = channels->aux8;
            _channels[9] = channels->aux9;
            _channels[10] = channels->aux10;
            _channels[11] = channels->aux11;
        }
#else
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: {
            crsf_channels_t * channels = (crsf_channels_t*)&input[1];
            _channels[0] = channels->ch0;
            _channels[1] = channels->ch1;
            _channels[2] = channels->ch2;
            _channels[3] = channels->ch3;
            _channels[4] = channels->ch4;
            _channels[5] = channels->ch5;
            _channels[6] = channels->ch6;
            _channels[7] = channels->ch7;
            _channels[8] = channels->ch8;
            _channels[9] = channels->ch9;
            _channels[10] = channels->ch10;
            _channels[11] = channels->ch11;
            _channels[12] = channels->ch12;
            _channels[13] = channels->ch13;
            _channels[14] = channels->ch14;
            _channels[15] = channels->ch15;
            break;
        }
#endif
        default:
            return 0;
    };
    return 1;
}


FAST_CODE_1 uint8_t crsf_parse_byte(uint8_t inChar)
{
    uint8_t ret = 0;

    if (SerialInPacketPtr >= sizeof(SerialInBuffer)) {
        // we reached the maximum allowable packet length,
        // so start again because shit fucked up hey.
        SerialInPacketPtr = 0;
        CRSFframeActive = 0;
    }

    // store byte
    SerialInBuffer[SerialInPacketPtr++] = inChar;

    // CRSF Frame:
    // | address | payload_len | payload* | crc |

    if (CRSFframeActive == 0) {
        if (inChar == CRSF_ADDRESS_CRSF_RECEIVER ||
                inChar == CRSF_ADDRESS_CRSF_TRANSMITTER ||
                inChar == CRSF_SYNC_BYTE ||
                inChar == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
            CRSFframeActive = 1;
            SerialInPacketLen = 0;
        } else {
            SerialInPacketPtr = 0;
        }
    } else {
        if (SerialInPacketLen == 0) {
            // we read the packet length and save it
            SerialInCrc = 0;
            SerialInPacketLen = inChar;
            SerialInPacketStart = SerialInPacketPtr;
            if ((SerialInPacketLen < 2) || (CRSF_FRAME_SIZE_MAX < SerialInPacketLen)) {
                // failure -> discard
                CRSFframeActive = 0;
                SerialInPacketPtr = 0;
            }
        } else {
            if ((SerialInPacketPtr - SerialInPacketStart) >= (SerialInPacketLen)) {
                /* Check packet CRC */
                if (SerialInCrc == inChar) {
                    ret = crsf_check_msg(&SerialInBuffer[SerialInPacketStart]);
                }

                // packet handled, start next
                CRSFframeActive = 0;
                SerialInPacketPtr = 0;
                SerialInPacketLen = 0;
            } else {
                // Calc crc on the fly
                SerialInCrc = CalcCRC(inChar, SerialInCrc);
            }
        }
    }

    return ret;
}


FAST_CODE_1 void crsf_get_rc_data(uint16_t * const rc_data, uint8_t len)
{
    uint8_t iter;
#if PROTO_ELRS
    for (iter = 0; iter < 4 && iter < ARRAY_SIZE(_channels); iter++) {
        // 10b -> 11b
#if (ANALOG_MAX != 2047)
        rc_data[iter] = MAP_U16(_channels[iter],
            ELRS_MIN, ELRS_MAX, ANALOG_MIN, ANALOG_MAX);
#else
        rc_data[iter] = _channels[iter] << 1;
#endif
    }
    for (; iter < len && iter < ARRAY_SIZE(_channels); iter++) {
        // 3b -> 11b
#if (ANALOG_MAX != 2047)
        rc_data[iter] = MAP_U16(_channels[iter],
            ELRS_SWITCH_MIN, ELRS_SWITCH_MAX, ANALOG_MIN, ANALOG_MAX);
#else
        rc_data[iter] = 292 * _channels[iter];
#endif
    }
#else
    for (iter = 0; iter < len && iter < ARRAY_SIZE(_channels); iter++) {
        rc_data[iter] = MAP_U16(_channels[iter],
            CRSF_MIN, CRSF_MAX, ANALOG_MIN, ANALOG_MAX);
    }
#endif
}
