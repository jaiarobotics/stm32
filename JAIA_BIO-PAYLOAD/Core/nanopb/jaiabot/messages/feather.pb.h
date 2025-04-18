/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.1 */

#ifndef PB_JAIABOT_PROTOBUF_NANOPB_JAIABOT_MESSAGES_FEATHER_PB_H_INCLUDED
#define PB_JAIABOT_PROTOBUF_NANOPB_JAIABOT_MESSAGES_FEATHER_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _jaiabot_protobuf_LoRaMessage_MessageType {
    jaiabot_protobuf_LoRaMessage_MessageType_LORA_DATA = 1,
    jaiabot_protobuf_LoRaMessage_MessageType_SET_PARAMETERS = 2,
    jaiabot_protobuf_LoRaMessage_MessageType_PARAMETERS_ACCEPTED = 3,
    jaiabot_protobuf_LoRaMessage_MessageType_PARAMETERS_REJECTED = 4,
    jaiabot_protobuf_LoRaMessage_MessageType_FEATHER_READY = 5,
    jaiabot_protobuf_LoRaMessage_MessageType_TRANSMIT_RESULT = 6,
    jaiabot_protobuf_LoRaMessage_MessageType_LOW_CONTROL = 50,
    jaiabot_protobuf_LoRaMessage_MessageType_DEBUG_MESSAGE = 100
} jaiabot_protobuf_LoRaMessage_MessageType;

typedef enum _jaiabot_protobuf_LoRaMessage_ModemConfigChoice {
    jaiabot_protobuf_LoRaMessage_ModemConfigChoice_Bw125Cr45Sf128 = 1,
    jaiabot_protobuf_LoRaMessage_ModemConfigChoice_Bw500Cr45Sf128 = 2,
    jaiabot_protobuf_LoRaMessage_ModemConfigChoice_Bw31_25Cr48Sf512 = 3,
    jaiabot_protobuf_LoRaMessage_ModemConfigChoice_Bw125Cr48Sf4096 = 4,
    jaiabot_protobuf_LoRaMessage_ModemConfigChoice_Bw125Cr45Sf2048 = 5
} jaiabot_protobuf_LoRaMessage_ModemConfigChoice;

/* Struct definitions */
typedef struct _jaiabot_protobuf_LoRaMessage_ControlSurfaces {
    int32_t motor;
    int32_t port_elevator;
    int32_t stbd_elevator;
    int32_t rudder;
} jaiabot_protobuf_LoRaMessage_ControlSurfaces;

typedef PB_BYTES_ARRAY_T(251) jaiabot_protobuf_LoRaMessage_data_t;
typedef struct _jaiabot_protobuf_LoRaMessage {
    int32_t src;
    int32_t dest;
    bool has_data;
    jaiabot_protobuf_LoRaMessage_data_t data;
    jaiabot_protobuf_LoRaMessage_MessageType type;
    bool has_id;
    int32_t id;
    bool has_flags;
    int32_t flags;
    bool has_rssi;
    int32_t rssi;
    bool has_transmit_successful;
    bool transmit_successful;
    bool has_modem_config;
    jaiabot_protobuf_LoRaMessage_ModemConfigChoice modem_config;
    bool has_tx_power;
    int32_t tx_power;
    bool has_control;
    jaiabot_protobuf_LoRaMessage_ControlSurfaces control;
} jaiabot_protobuf_LoRaMessage;


/* Helper constants for enums */
#define _jaiabot_protobuf_LoRaMessage_MessageType_MIN jaiabot_protobuf_LoRaMessage_MessageType_LORA_DATA
#define _jaiabot_protobuf_LoRaMessage_MessageType_MAX jaiabot_protobuf_LoRaMessage_MessageType_DEBUG_MESSAGE
#define _jaiabot_protobuf_LoRaMessage_MessageType_ARRAYSIZE ((jaiabot_protobuf_LoRaMessage_MessageType)(jaiabot_protobuf_LoRaMessage_MessageType_DEBUG_MESSAGE+1))

#define _jaiabot_protobuf_LoRaMessage_ModemConfigChoice_MIN jaiabot_protobuf_LoRaMessage_ModemConfigChoice_Bw125Cr45Sf128
#define _jaiabot_protobuf_LoRaMessage_ModemConfigChoice_MAX jaiabot_protobuf_LoRaMessage_ModemConfigChoice_Bw125Cr45Sf2048
#define _jaiabot_protobuf_LoRaMessage_ModemConfigChoice_ARRAYSIZE ((jaiabot_protobuf_LoRaMessage_ModemConfigChoice)(jaiabot_protobuf_LoRaMessage_ModemConfigChoice_Bw125Cr45Sf2048+1))


/* Initializer values for message structs */
#define jaiabot_protobuf_LoRaMessage_init_default {0, 0, false, {0, {0}}, jaiabot_protobuf_LoRaMessage_MessageType_LORA_DATA, false, 0, false, 0, false, 0, false, 0, false, jaiabot_protobuf_LoRaMessage_ModemConfigChoice_Bw125Cr45Sf128, false, 13, false, jaiabot_protobuf_LoRaMessage_ControlSurfaces_init_default}
#define jaiabot_protobuf_LoRaMessage_ControlSurfaces_init_default {0, 0, 0, 0}
#define jaiabot_protobuf_LoRaMessage_init_zero   {0, 0, false, {0, {0}}, _jaiabot_protobuf_LoRaMessage_MessageType_MIN, false, 0, false, 0, false, 0, false, 0, false, _jaiabot_protobuf_LoRaMessage_ModemConfigChoice_MIN, false, 0, false, jaiabot_protobuf_LoRaMessage_ControlSurfaces_init_zero}
#define jaiabot_protobuf_LoRaMessage_ControlSurfaces_init_zero {0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define jaiabot_protobuf_LoRaMessage_ControlSurfaces_motor_tag 1
#define jaiabot_protobuf_LoRaMessage_ControlSurfaces_port_elevator_tag 2
#define jaiabot_protobuf_LoRaMessage_ControlSurfaces_stbd_elevator_tag 3
#define jaiabot_protobuf_LoRaMessage_ControlSurfaces_rudder_tag 4
#define jaiabot_protobuf_LoRaMessage_src_tag     1
#define jaiabot_protobuf_LoRaMessage_dest_tag    2
#define jaiabot_protobuf_LoRaMessage_data_tag    3
#define jaiabot_protobuf_LoRaMessage_type_tag    4
#define jaiabot_protobuf_LoRaMessage_id_tag      5
#define jaiabot_protobuf_LoRaMessage_flags_tag   6
#define jaiabot_protobuf_LoRaMessage_rssi_tag    7
#define jaiabot_protobuf_LoRaMessage_transmit_successful_tag 10
#define jaiabot_protobuf_LoRaMessage_modem_config_tag 20
#define jaiabot_protobuf_LoRaMessage_tx_power_tag 21
#define jaiabot_protobuf_LoRaMessage_control_tag 30

/* Struct field encoding specification for nanopb */
#define jaiabot_protobuf_LoRaMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, INT32,    src,               1) \
X(a, STATIC,   REQUIRED, INT32,    dest,              2) \
X(a, STATIC,   OPTIONAL, BYTES,    data,              3) \
X(a, STATIC,   REQUIRED, UENUM,    type,              4) \
X(a, STATIC,   OPTIONAL, INT32,    id,                5) \
X(a, STATIC,   OPTIONAL, INT32,    flags,             6) \
X(a, STATIC,   OPTIONAL, SINT32,   rssi,              7) \
X(a, STATIC,   OPTIONAL, BOOL,     transmit_successful,  10) \
X(a, STATIC,   OPTIONAL, UENUM,    modem_config,     20) \
X(a, STATIC,   OPTIONAL, INT32,    tx_power,         21) \
X(a, STATIC,   OPTIONAL, MESSAGE,  control,          30)
#define jaiabot_protobuf_LoRaMessage_CALLBACK NULL
#define jaiabot_protobuf_LoRaMessage_DEFAULT (const pb_byte_t*)"\x20\x01\xa0\x01\x01\xa8\x01\x0d\x00"
#define jaiabot_protobuf_LoRaMessage_control_MSGTYPE jaiabot_protobuf_LoRaMessage_ControlSurfaces

#define jaiabot_protobuf_LoRaMessage_ControlSurfaces_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, SINT32,   motor,             1) \
X(a, STATIC,   REQUIRED, SINT32,   port_elevator,     2) \
X(a, STATIC,   REQUIRED, SINT32,   stbd_elevator,     3) \
X(a, STATIC,   REQUIRED, SINT32,   rudder,            4)
#define jaiabot_protobuf_LoRaMessage_ControlSurfaces_CALLBACK NULL
#define jaiabot_protobuf_LoRaMessage_ControlSurfaces_DEFAULT NULL

extern const pb_msgdesc_t jaiabot_protobuf_LoRaMessage_msg;
extern const pb_msgdesc_t jaiabot_protobuf_LoRaMessage_ControlSurfaces_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define jaiabot_protobuf_LoRaMessage_fields &jaiabot_protobuf_LoRaMessage_msg
#define jaiabot_protobuf_LoRaMessage_ControlSurfaces_fields &jaiabot_protobuf_LoRaMessage_ControlSurfaces_msg

/* Maximum encoded size of messages (where known) */
#define jaiabot_protobuf_LoRaMessage_size        350
#define jaiabot_protobuf_LoRaMessage_ControlSurfaces_size 24

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
