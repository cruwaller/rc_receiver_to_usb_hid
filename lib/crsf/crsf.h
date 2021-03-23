#pragma once

#include "helpers.h"
#include <stdint.h>


#define CRSF_RX_BAUDRATE      420000
#define CRSF_TX_BAUDRATE_FAST 400000
#define CRSF_TX_BAUDRATE_SLOW 115200
#define CRSF_NUM_CHANNELS     16         // Number of input channels

#define CRSF_SYNC_BYTE 0xC8

#define CRSF_PAYLOAD_SIZE_MAX 62
#define CRSF_FRAME_START_BYTES 2 // address + len (start of the CRSF frame, not counted to frame len)
#define CRSF_FRAME_HEADER_BYTES 2 // type + crc
#define CRSF_FRAME_SIZE(payload_size) ((payload_size) + CRSF_FRAME_HEADER_BYTES) // See crsf_header_t.frame_size
#define CRSF_EXT_FRAME_SIZE(payload_size) (CRSF_FRAME_SIZE(payload_size) + CRSF_FRAME_START_BYTES)
#define CRSF_FRAME_SIZE_MAX (CRSF_PAYLOAD_SIZE_MAX + CRSF_FRAME_START_BYTES)

#define CRSF_MSP_FRAME_HEADER_BYTES 4 // type, dest, orig, crc
#define CRSF_MSP_FRAME_SIZE(payload_size) (CRSF_FRAME_SIZE(payload_size) + CRSF_MSP_FRAME_HEADER_BYTES)


// OUT to flight controller
#define ELRS_MIN        0
#define ELRS_MAX        1023
#define ELRS_SWITCH_MIN 0
#define ELRS_SWITCH_MAX 6

#define CRSF_MIN        172
#define CRSF_MID        992
#define CRSF_MAX        1811

#if PROTO_ELRS
#define RX_BAUDRATE 691200
#else
#define RX_BAUDRATE CRSF_RX_BAUDRATE
#endif

//////////////////////////////////////////////////////////////

enum crsf_frame_type_e
{
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_LINK_STATISTICS_ELRS = 0x15,
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED_ELRS = 0x17,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
    CRSF_FRAMETYPE_COMMAND = 0x32,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
};

enum crsf_addr_e
{
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
};

typedef enum
{
    CRSF_UINT8 = 0,
    CRSF_INT8 = 1,
    CRSF_UINT16 = 2,
    CRSF_INT16 = 3,
    CRSF_UINT32 = 4,
    CRSF_INT32 = 5,
    CRSF_UINT64 = 6,
    CRSF_INT64 = 7,
    CRSF_FLOAT = 8,
    CRSF_TEXT_SELECTION = 9,
    CRSF_STRING = 10,
    CRSF_FOLDER = 11,
    CRSF_INFO = 12,
    CRSF_COMMAND = 13,
    CRSF_VTX = 15,
    CRSF_OUT_OF_RANGE = 127,
} crsf_value_type_e;

typedef struct crsf_header_s
{
    uint8_t device_addr; // from crsf_addr_e
    uint8_t frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t type;        // from crsf_frame_type_e
} PACKED crsf_header_t;

// Used by extended header frames (type in range 0x28 to 0x96)
typedef struct crsf_ext_header_s
{
    // Common header fields, see crsf_header_t
    uint8_t device_addr;
    uint8_t frame_size;
    uint8_t type;
    // Extended fields
    uint8_t dest_addr;
    uint8_t orig_addr;
} PACKED crsf_ext_header_t;

typedef struct crsf_channels_s
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} PACKED crsf_channels_t;

typedef struct elrs_channels_s {
    // 64 bits of data (4 x 10 bits + 8 x 3 bits channels) = 8 bytes.
    unsigned int analog0 : 10;
    unsigned int analog1 : 10;
    unsigned int analog2 : 10;
    unsigned int analog3 : 10;
    unsigned int aux4 : 3;
    unsigned int aux5 : 3;
    unsigned int aux6 : 3;
    unsigned int aux7 : 3;
    unsigned int aux8 : 3;
    unsigned int aux9 : 3;
    unsigned int aux10 : 3;
    unsigned int aux11 : 3;
} PACKED elrs_channels_t;

typedef struct crsf_channels_msg_s
{
    crsf_header_t header;
    crsf_channels_t data;
    uint8_t crc;
} PACKED crsf_channels_msg_t;

// Used by extended header frames (type in range 0x28 to 0x96)
typedef struct crsf_sensor_battery_s
{
    crsf_header_t header;
    uint16_t voltage;  // mv * 100
    uint16_t current;  // ma * 100
    uint32_t capacity : 24; // mah
    uint32_t remaining : 8; // %
    uint8_t crc;
} PACKED crsf_sensor_battery_t;

/*
 * 0x14 Link statistics
 * Payload:
 *
 * uint8_t Uplink RSSI Ant. 1 ( dBm * -1 )
 * uint8_t Uplink RSSI Ant. 2 ( dBm * -1 )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
 * uint8_t RF Mode ( enum 4fps = 0 , 50fps, 150hz)
 * uint8_t Downlink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW, 250mW )
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 *
 * Uplink is the connection from the ground to the UAV and downlink the opposite direction.
 * Uplink:   PILOT => UAV
 * Downlink: UAV   => PILOT
 */
typedef struct crsfLinkStatistics_s
{
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality; // this goes to opentx rssi
    int8_t uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;
} PACKED crsfLinkStatistics_t;

typedef struct crsfLinkStatisticsMsg_s
{
    crsf_header_t header;
    crsfLinkStatistics_t stats;
    uint8_t crc;
} PACKED crsfLinkStatisticsMsg_t;

typedef struct crsf_sensor_gps_s
{
    crsf_header_t header;
    int32_t latitude;
    int32_t longitude;
    uint16_t speed;
    uint16_t heading;
    uint16_t altitude;
    uint8_t satellites;
    uint8_t crc;
} PACKED crsf_sensor_gps_t;

/* MSP from radio to FC */
#define CRSF_FRAME_RX_MSP_FRAME_SIZE 8
typedef struct
{
    uint8_t flags;
    union {
        struct {
            uint8_t payloadSize;
            uint8_t function;
            uint8_t payload[CRSF_FRAME_RX_MSP_FRAME_SIZE-3];
        } hdr;
        uint8_t payload[CRSF_FRAME_RX_MSP_FRAME_SIZE-1];
    };
} PACKED mspHeaderV1_RX_t;

/* MSP from FC to radio */
#define CRSF_FRAME_TX_MSP_FRAME_SIZE 58
typedef struct
{
    uint8_t flags;
    union {
        struct {
            uint8_t payloadSize;
            uint8_t function;
            uint8_t payload[CRSF_FRAME_TX_MSP_FRAME_SIZE-4];
        } hdr;
        uint8_t payload[CRSF_FRAME_TX_MSP_FRAME_SIZE-2];
    };
    uint8_t crc_msp;
} PACKED mspHeaderV1_TX_t;

typedef struct crsf_msp_packet_fc_s
{
    crsf_ext_header_t header;
    mspHeaderV1_RX_t msp;
    uint8_t crc;
} PACKED crsf_msp_packet_fc_t;

typedef struct crsf_msp_packet_radio_s
{
    crsf_ext_header_t header;
    mspHeaderV1_TX_t msp;
    uint8_t crc;
} PACKED crsf_msp_packet_radio_t;



uint8_t crsf_parse_byte(uint8_t inChar);
void crsf_get_rc_data(uint16_t * const rc_data, uint8_t len);
