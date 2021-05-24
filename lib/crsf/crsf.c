#include "crsf.h"
#include "crc.h"
#include "defines.h"
#include "uart.h"
#include <stddef.h>


#define CRSF_SUBSET_RC_STARTING_CHANNEL_BITS        5
#define CRSF_SUBSET_RC_STARTING_CHANNEL_MASK        0x1F
#define CRSF_SUBSET_RC_RES_CONFIGURATION_BITS       2
#define CRSF_SUBSET_RC_RES_CONFIGURATION_MASK       0x03
#define CRSF_SUBSET_RC_RESERVED_CONFIGURATION_BITS  1

#define CRSF_RC_CHANNEL_SCALE_LEGACY                0.62477120195241f
#define CRSF_SUBSET_RC_RES_CONF_10B                 0
#define CRSF_SUBSET_RC_RES_BITS_10B                 10
#define CRSF_SUBSET_RC_RES_MASK_10B                 0x03FF
#define CRSF_SUBSET_RC_CHANNEL_SCALE_10B            1.0f
#define CRSF_SUBSET_RC_RES_CONF_11B                 1
#define CRSF_SUBSET_RC_RES_BITS_11B                 11
#define CRSF_SUBSET_RC_RES_MASK_11B                 0x07FF
#define CRSF_SUBSET_RC_CHANNEL_SCALE_11B            0.5f
#define CRSF_SUBSET_RC_RES_CONF_12B                 2
#define CRSF_SUBSET_RC_RES_BITS_12B                 12
#define CRSF_SUBSET_RC_RES_MASK_12B                 0x0FFF
#define CRSF_SUBSET_RC_CHANNEL_SCALE_12B            0.25f
#define CRSF_SUBSET_RC_RES_CONF_13B                 3
#define CRSF_SUBSET_RC_RES_BITS_13B                 13
#define CRSF_SUBSET_RC_RES_MASK_13B                 0x1FFF
#define CRSF_SUBSET_RC_CHANNEL_SCALE_13B            0.125f


enum {
    IN_TYPE_LEGACY = 0,
    IN_TYPE_V3_10B = (1 << 10) - 1,
    IN_TYPE_V3_11B = (1 << 11) - 1,
    IN_TYPE_V3_12B = (1 << 12) - 1,
    IN_TYPE_V3_13B = (1 << 13) - 1,
};


static uint8_t SerialInBuffer[256];
static uint8_t CRSFframeActive;
static uint8_t SerialInCrc;
static uint8_t SerialInPacketStart;
static uint8_t SerialInPacketLen;      // length of the CRSF packet as measured
static uint8_t SerialInPacketPtr;      // index where we are reading/writing

static uint32_t SerialInPacketType;


#if PROTO_ELRS
#define MAX_CHAHNELS 12
#else
#define MAX_CHAHNELS 24 // old CRSF 16, CRSFv3 24
#endif
static uint16_t _channels[MAX_CHAHNELS];


typedef struct {
    // Common header fields, see crsf_header_t
    uint8_t device_addr;
    uint8_t frame_size;
    uint8_t type;
    // Extended fields
    uint8_t dest_addr;
    uint8_t orig_addr;
    // Command fields
    uint8_t command;
    uint8_t sub_command;
    // Message
    uint8_t portID;
    uint8_t status; // true / false if baud ok
    // footer
    uint8_t crc_cmd; // crc of type + payload
    uint8_t crc;
} PACKED crsf_v3_speed_control_resp_t;

typedef struct {
    // Extended fields
    uint8_t dest_addr;
    uint8_t orig_addr;
    // proposal
    uint8_t command;
    uint8_t sub_command;
    uint8_t portID;
    uint32_t baudrate;
    uint8_t crc_cmd; // crc of type + payload
    uint8_t crc;
} PACKED crsf_v3_speed_control_req_t;

static uint8_t portID;
static uint32_t baudrate;


void send_negotiate_resp(void)
{
    crsf_v3_speed_control_resp_t resp;
    resp.device_addr = CRSF_SYNC_BYTE;
    resp.frame_size = sizeof(crsf_v3_speed_control_resp_t) - CRSF_FRAME_HEADER_BYTES;
    resp.type = CRSF_FRAMETYPE_COMMAND;
    resp.dest_addr = CRSF_ADDRESS_CRSF_RECEIVER;
    resp.orig_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    resp.command = CRSF_COMMAND_SUBCMD_GENERAL;
    resp.sub_command = CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_RESPONSE;
    resp.portID = portID;
    resp.status = 1;
    resp.crc_cmd = CalcCRC8len(&resp.type, 7, 0, 0xBA);
    resp.crc = CalcCRClen(&resp.type, sizeof(resp) - 3, 0);
}


void reconfigure_uart(void)
{
    if (baudrate) {
        //uart_deinit();
        //uart_init(RX_BAUDRATE, RECEIVER_UART_RX, DBG_UART_TX);
        //send_negotiate_resp();
        baudrate = 0;
    }
}


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
            SerialInPacketType = IN_TYPE_LEGACY;
            break;
        }

        case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED: {
            // use subset RC frame structure (0x17) aka CRSFv3
            // This code is copied from Betaflight (https://github.com/betaflight/betaflight/)

            uint16_t channelMask;
            uint8_t channelBits;
            uint8_t readByteIndex = 1;
            // get the configuration byte
            uint8_t configByte = input[readByteIndex++];
            // get the channel number of start channel
            uint8_t const startChannel = configByte & CRSF_SUBSET_RC_STARTING_CHANNEL_MASK;
            configByte >>= CRSF_SUBSET_RC_STARTING_CHANNEL_BITS;

            // get the channel resolution settings
            uint8_t channelRes = configByte & CRSF_SUBSET_RC_RES_CONFIGURATION_MASK;
            //configByte >>= CRSF_SUBSET_RC_RES_CONFIGURATION_BITS;
            switch (channelRes) {
                case CRSF_SUBSET_RC_RES_CONF_10B:
                    channelBits = CRSF_SUBSET_RC_RES_BITS_10B;
                    channelMask = CRSF_SUBSET_RC_RES_MASK_10B;
                    SerialInPacketType = IN_TYPE_V3_10B;
                    break;
                default:
                case CRSF_SUBSET_RC_RES_CONF_11B:
                    channelBits = CRSF_SUBSET_RC_RES_BITS_11B;
                    channelMask = CRSF_SUBSET_RC_RES_MASK_11B;
                    SerialInPacketType = IN_TYPE_V3_11B;
                    break;
                case CRSF_SUBSET_RC_RES_CONF_12B:
                    channelBits = CRSF_SUBSET_RC_RES_BITS_12B;
                    channelMask = CRSF_SUBSET_RC_RES_MASK_12B;
                    SerialInPacketType = IN_TYPE_V3_12B;
                    break;
                case CRSF_SUBSET_RC_RES_CONF_13B:
                    channelBits = CRSF_SUBSET_RC_RES_BITS_13B;
                    channelMask = CRSF_SUBSET_RC_RES_MASK_13B;
                    SerialInPacketType = IN_TYPE_V3_13B;
                    break;
            }

            // do nothing for the reserved configuration bit
            //configByte >>= CRSF_SUBSET_RC_RESERVED_CONFIGURATION_BITS;

            // calculate the number of channels packed
            uint8_t const numOfChannels =
                ((frameLength - CRSF_FRAME_HEADER_BYTES - 1) * 8) / channelBits;

            // unpack the channel data
            uint8_t bitsMerged = 0;
            uint32_t readValue = 0;
            for (uint8_t n = 0; n < numOfChannels; n++) {
                while (bitsMerged < channelBits) {
                    uint8_t readByte = input[readByteIndex++];
                    readValue |= ((uint32_t) readByte) << bitsMerged;
                    bitsMerged += 8;
                }
                _channels[startChannel + n] = readValue & channelMask;
                readValue >>= channelBits;
                bitsMerged -= channelBits;
            }
            break;
        }

        case CRSF_FRAMETYPE_COMMAND: {
            crsf_v3_speed_control_req_t * req =
                (crsf_v3_speed_control_req_t*)&input[1];

            /* TODO: Handle baudrate negotiation */

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
                inChar == CRSF_ADDRESS_FLIGHT_CONTROLLER ||
                inChar == CRSF_ADDRESS_BROADCAST) {
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
    uint16_t _min = CRSF_MIN;
    uint16_t _max = CRSF_MAX;
    if (SerialInPacketType) {
        _min = 0;
        _max = SerialInPacketType;
    }

    reconfigure_uart();

    for (iter = 0; iter < len && iter < ARRAY_SIZE(_channels); iter++) {
        rc_data[iter] = MAP_U16(_channels[iter],
            _min, _max, ANALOG_MIN, ANALOG_MAX);
    }
#endif
}
