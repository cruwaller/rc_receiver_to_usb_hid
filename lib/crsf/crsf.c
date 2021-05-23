#include "crsf.h"
#include "crc.h"
#include "defines.h"
#include <stddef.h>


// first 5 bits in the first byte hold the first channel packed
// remaining bits hold the channel data in 11-bit format
#define CRSF_SUBSET_RC_CHANNELS_PACKED_RESOLUTION                  11
#define CRSF_SUBSET_RC_CHANNELS_PACKED_MASK                        0x07FF
#define CRSF_SUBSET_RC_CHANNELS_PACKED_STARTING_CHANNEL_RESOLUTION 5
#define CRSF_SUBSET_RC_CHANNELS_PACKED_STARTING_CHANNEL_MASK       0x1F


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


static FAST_CODE_1 uint8_t crsf_check_msg(uint8_t const * const input, uint8_t const frameLength)
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
            // use ordinary RC frame structure (0x16)
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

        case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED: {
            // use subset RC frame structure (0x17) aka CRSFv3
            // This code is copied from Betaflight (https://github.com/betaflight/betaflight/)
            uint32_t readValue = 0;
            uint8_t iter;
            uint8_t readByte;
            uint8_t readByteIndex = 1; // skip type
            uint8_t bitsMerged = 0;
            uint8_t startChannel = UINT8_MAX;
            uint8_t const numOfChannels =
                ((frameLength - CRSF_FRAME_HEADER_BYTES) * 8 - CRSF_SUBSET_RC_CHANNELS_PACKED_STARTING_CHANNEL_RESOLUTION)
                / CRSF_SUBSET_RC_CHANNELS_PACKED_RESOLUTION;
            for (iter = 0; iter < numOfChannels; iter++) {
                while (bitsMerged < CRSF_SUBSET_RC_CHANNELS_PACKED_RESOLUTION) {
                    readByte = input[readByteIndex++];
                    if (startChannel == UINT8_MAX) {
                        // get the startChannel
                        startChannel = readByte & CRSF_SUBSET_RC_CHANNELS_PACKED_STARTING_CHANNEL_MASK;
                        readByte >>= CRSF_SUBSET_RC_CHANNELS_PACKED_STARTING_CHANNEL_RESOLUTION;
                        readValue |= ((uint32_t) readByte) << bitsMerged;
                        bitsMerged += 8 - CRSF_SUBSET_RC_CHANNELS_PACKED_STARTING_CHANNEL_RESOLUTION;
                    } else {
                        readValue |= ((uint32_t) readByte) << bitsMerged;
                        bitsMerged += 8;
                    }
                }
                _channels[startChannel + iter] = readValue & CRSF_SUBSET_RC_CHANNELS_PACKED_MASK;
                readValue >>= CRSF_SUBSET_RC_CHANNELS_PACKED_RESOLUTION;
                bitsMerged -= CRSF_SUBSET_RC_CHANNELS_PACKED_RESOLUTION;
            }
            break;
        }

        /* TODO: Handle baudrate negotiation */
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
                    ret = crsf_check_msg(&SerialInBuffer[SerialInPacketStart], SerialInPacketLen);
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
