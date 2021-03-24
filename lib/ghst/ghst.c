#include "ghst.h"
#include "crc.h"
#include "defines.h"


static uint16_t _channels[GHST_NUM_OF_CHANNELS];

static uint8_t SerialInBuffer[GHST_FRAME_SIZE_MAX*2];
static uint8_t SerialInCrc;
static uint8_t SerialInPacketStart;
static uint8_t SerialInPacketLen;
static uint8_t SerialInPacketPtr;
static uint8_t frameActive;


static FAST_CODE_1 uint8_t processPacket(uint8_t const *data)
{
    const ghstPayloadPulses_t * channels_msg = (ghstPayloadPulses_t*)&data[1];
    uint8_t switchidx = 0;
    switch (data[0]) {
        case GHST_UL_RC_CHANS_HS4_RSSI: {
            break;
        }
        case GHST_UL_RC_CHANS_HS4_5TO8: {
            switchidx = 4;
            break;
        }
        case GHST_UL_RC_CHANS_HS4_9TO12: {
            switchidx = 8;
            break;
        }
        case GHST_UL_RC_CHANS_HS4_13TO16: {
            switchidx = 12;
            break;
        }
        default:
            return 0;
    }
    _channels[0] = channels_msg->ch1to4.ch1 >> 1;
    _channels[1] = channels_msg->ch1to4.ch2 >> 1;
    _channels[2] = channels_msg->ch1to4.ch3 >> 1;
    _channels[3] = channels_msg->ch1to4.ch4 >> 1;

    if (switchidx) {
        _channels[switchidx++] = channels_msg->aux.cha;
        _channels[switchidx++] = channels_msg->aux.chb;
        _channels[switchidx++] = channels_msg->aux.chc;
        _channels[switchidx++] = channels_msg->aux.chd;
    }
    return 1;
}


FAST_CODE_1 uint8_t ghst_parse_byte(uint8_t const inChar)
{
    uint8_t frame_ok = 0;

    if (SerialInPacketPtr >= sizeof(SerialInBuffer)) {
        // we reached the maximum allowable packet length,
        // so start again because shit fucked up hey.
        SerialInPacketPtr = 0;
        frameActive = 0;
    }

    // store byte
    SerialInBuffer[SerialInPacketPtr++] = inChar;

    // GHST Frame:
    // | address | payload_len | payload* | crc |

    if (frameActive == 0) {
        if (inChar == GHST_ADDR_FC) {
            frameActive = 1;
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
            if ((SerialInPacketLen < 2) || (GHST_FRAME_SIZE_MAX < SerialInPacketLen)) {
                // failure -> discard
                frameActive = 0;
                SerialInPacketPtr = 0;
            }
        } else {
            if ((SerialInPacketPtr - SerialInPacketStart) >= (SerialInPacketLen)) {
                /* Check packet CRC */
                if (SerialInCrc == inChar) {
                    uint8_t * packet_ptr = &SerialInBuffer[SerialInPacketStart];
                    frame_ok = processPacket(packet_ptr);
                } else {
                    /* CRC FAIL */
                }

                // packet handled, start next
                frameActive = 0;
                SerialInPacketPtr = 0;
                SerialInPacketLen = 0;
            } else {
                // Calc crc on the fly
                SerialInCrc = CalcCRC(inChar, SerialInCrc);
            }
        }
    }
    return frame_ok;
}


FAST_CODE_1 void ghst_get_rc_data(uint16_t * const rc_data, uint8_t len)
{
    uint8_t iter;
    for (iter = 0; iter < 4 && iter < ARRAY_SIZE(_channels); iter++) {
        rc_data[iter] = MAP_U16(_channels[iter],
            GHST_MIN, GHST_MAX, ANALOG_MIN, ANALOG_MAX);
    }
    for (; iter < len && iter < ARRAY_SIZE(_channels); iter++) {
        rc_data[iter] = MAP_U16(_channels[iter],
            GHST_SWITCH_MIN, GHST_SWITCH_MAX, ANALOG_MIN, ANALOG_MAX);
    }
}
