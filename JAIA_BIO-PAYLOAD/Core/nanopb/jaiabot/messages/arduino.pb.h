/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.1 */

#ifndef PB_JAIABOT_PROTOBUF_NANOPB_JAIABOT_MESSAGES_ARDUINO_PB_H_INCLUDED
#define PB_JAIABOT_PROTOBUF_NANOPB_JAIABOT_MESSAGES_ARDUINO_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _jaiabot_protobuf_ArduinoStatusCode {
    jaiabot_protobuf_ArduinoStatusCode_STARTUP = 0,
    jaiabot_protobuf_ArduinoStatusCode_ACK = 1,
    jaiabot_protobuf_ArduinoStatusCode_TIMEOUT = 2,
    jaiabot_protobuf_ArduinoStatusCode_PREFIX_READ_ERROR = 3,
    jaiabot_protobuf_ArduinoStatusCode_MAGIC_WRONG = 4,
    jaiabot_protobuf_ArduinoStatusCode_MESSAGE_TOO_BIG = 5,
    jaiabot_protobuf_ArduinoStatusCode_MESSAGE_WRONG_SIZE = 6,
    jaiabot_protobuf_ArduinoStatusCode_MESSAGE_DECODE_ERROR = 7,
    jaiabot_protobuf_ArduinoStatusCode_CRC_ERROR = 8,
    jaiabot_protobuf_ArduinoStatusCode_SETTINGS = 9
} jaiabot_protobuf_ArduinoStatusCode;

/* Struct definitions */
typedef struct _jaiabot_protobuf_ArduinoActuators {
    int32_t motor;
    int32_t port_elevator;
    int32_t stbd_elevator;
    int32_t rudder;
    int32_t timeout;
    bool led_switch_on;
} jaiabot_protobuf_ArduinoActuators;

typedef struct _jaiabot_protobuf_ArduinoDebug {
    bool has_arduino_restarted;
    bool arduino_restarted;
    bool has_arduino_not_responding;
    bool arduino_not_responding;
} jaiabot_protobuf_ArduinoDebug;

typedef struct _jaiabot_protobuf_ArduinoResponse {
    jaiabot_protobuf_ArduinoStatusCode status_code;
    bool has_thermocouple_temperature_C;
    float thermocouple_temperature_C;
    bool has_vccvoltage;
    float vccvoltage;
    bool has_vcccurrent;
    float vcccurrent;
    bool has_vvcurrent;
    float vvcurrent;
    bool has_motor;
    int32_t motor;
    bool has_thermistor_voltage;
    float thermistor_voltage;
    bool has_crc;
    uint32_t crc;
    bool has_calculated_crc;
    uint32_t calculated_crc;
    uint32_t version;
} jaiabot_protobuf_ArduinoResponse;

typedef struct _jaiabot_protobuf_ArduinoSettings {
    int32_t forward_start;
    int32_t reverse_start;
} jaiabot_protobuf_ArduinoSettings;

typedef struct _jaiabot_protobuf_ArduinoCommand {
    bool has_settings;
    jaiabot_protobuf_ArduinoSettings settings;
    bool has_actuators;
    jaiabot_protobuf_ArduinoActuators actuators;
} jaiabot_protobuf_ArduinoCommand;


/* Helper constants for enums */
#define _jaiabot_protobuf_ArduinoStatusCode_MIN jaiabot_protobuf_ArduinoStatusCode_STARTUP
#define _jaiabot_protobuf_ArduinoStatusCode_MAX jaiabot_protobuf_ArduinoStatusCode_SETTINGS
#define _jaiabot_protobuf_ArduinoStatusCode_ARRAYSIZE ((jaiabot_protobuf_ArduinoStatusCode)(jaiabot_protobuf_ArduinoStatusCode_SETTINGS+1))


/* Initializer values for message structs */
#define jaiabot_protobuf_ArduinoSettings_init_default {0, 0}
#define jaiabot_protobuf_ArduinoActuators_init_default {0, 0, 0, 0, 0, 0}
#define jaiabot_protobuf_ArduinoCommand_init_default {false, jaiabot_protobuf_ArduinoSettings_init_default, false, jaiabot_protobuf_ArduinoActuators_init_default}
#define jaiabot_protobuf_ArduinoResponse_init_default {_jaiabot_protobuf_ArduinoStatusCode_MIN, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, 0u}
#define jaiabot_protobuf_ArduinoDebug_init_default {false, false, false, false}
#define jaiabot_protobuf_ArduinoSettings_init_zero {0, 0}
#define jaiabot_protobuf_ArduinoActuators_init_zero {0, 0, 0, 0, 0, 0}
#define jaiabot_protobuf_ArduinoCommand_init_zero {false, jaiabot_protobuf_ArduinoSettings_init_zero, false, jaiabot_protobuf_ArduinoActuators_init_zero}
#define jaiabot_protobuf_ArduinoResponse_init_zero {_jaiabot_protobuf_ArduinoStatusCode_MIN, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, 0}
#define jaiabot_protobuf_ArduinoDebug_init_zero  {false, 0, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define jaiabot_protobuf_ArduinoActuators_motor_tag 1
#define jaiabot_protobuf_ArduinoActuators_port_elevator_tag 2
#define jaiabot_protobuf_ArduinoActuators_stbd_elevator_tag 3
#define jaiabot_protobuf_ArduinoActuators_rudder_tag 4
#define jaiabot_protobuf_ArduinoActuators_timeout_tag 5
#define jaiabot_protobuf_ArduinoActuators_led_switch_on_tag 6
#define jaiabot_protobuf_ArduinoDebug_arduino_restarted_tag 1
#define jaiabot_protobuf_ArduinoDebug_arduino_not_responding_tag 2
#define jaiabot_protobuf_ArduinoResponse_status_code_tag 1
#define jaiabot_protobuf_ArduinoResponse_thermocouple_temperature_C_tag 2
#define jaiabot_protobuf_ArduinoResponse_vccvoltage_tag 3
#define jaiabot_protobuf_ArduinoResponse_vcccurrent_tag 4
#define jaiabot_protobuf_ArduinoResponse_vvcurrent_tag 5
#define jaiabot_protobuf_ArduinoResponse_motor_tag 6
#define jaiabot_protobuf_ArduinoResponse_thermistor_voltage_tag 7
#define jaiabot_protobuf_ArduinoResponse_crc_tag 50
#define jaiabot_protobuf_ArduinoResponse_calculated_crc_tag 51
#define jaiabot_protobuf_ArduinoResponse_version_tag 52
#define jaiabot_protobuf_ArduinoSettings_forward_start_tag 1
#define jaiabot_protobuf_ArduinoSettings_reverse_start_tag 2
#define jaiabot_protobuf_ArduinoCommand_settings_tag 1
#define jaiabot_protobuf_ArduinoCommand_actuators_tag 2

/* Struct field encoding specification for nanopb */
#define jaiabot_protobuf_ArduinoSettings_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, SINT32,   forward_start,     1) \
X(a, STATIC,   REQUIRED, SINT32,   reverse_start,     2)
#define jaiabot_protobuf_ArduinoSettings_CALLBACK NULL
#define jaiabot_protobuf_ArduinoSettings_DEFAULT NULL

#define jaiabot_protobuf_ArduinoActuators_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, SINT32,   motor,             1) \
X(a, STATIC,   REQUIRED, SINT32,   port_elevator,     2) \
X(a, STATIC,   REQUIRED, SINT32,   stbd_elevator,     3) \
X(a, STATIC,   REQUIRED, SINT32,   rudder,            4) \
X(a, STATIC,   REQUIRED, SINT32,   timeout,           5) \
X(a, STATIC,   REQUIRED, BOOL,     led_switch_on,     6)
#define jaiabot_protobuf_ArduinoActuators_CALLBACK NULL
#define jaiabot_protobuf_ArduinoActuators_DEFAULT NULL

#define jaiabot_protobuf_ArduinoCommand_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  settings,          1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  actuators,         2)
#define jaiabot_protobuf_ArduinoCommand_CALLBACK NULL
#define jaiabot_protobuf_ArduinoCommand_DEFAULT NULL
#define jaiabot_protobuf_ArduinoCommand_settings_MSGTYPE jaiabot_protobuf_ArduinoSettings
#define jaiabot_protobuf_ArduinoCommand_actuators_MSGTYPE jaiabot_protobuf_ArduinoActuators

#define jaiabot_protobuf_ArduinoResponse_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UENUM,    status_code,       1) \
X(a, STATIC,   OPTIONAL, FLOAT,    thermocouple_temperature_C,   2) \
X(a, STATIC,   OPTIONAL, FLOAT,    vccvoltage,        3) \
X(a, STATIC,   OPTIONAL, FLOAT,    vcccurrent,        4) \
X(a, STATIC,   OPTIONAL, FLOAT,    vvcurrent,         5) \
X(a, STATIC,   OPTIONAL, INT32,    motor,             6) \
X(a, STATIC,   OPTIONAL, FLOAT,    thermistor_voltage,   7) \
X(a, STATIC,   OPTIONAL, UINT32,   crc,              50) \
X(a, STATIC,   OPTIONAL, UINT32,   calculated_crc,   51) \
X(a, STATIC,   REQUIRED, UINT32,   version,          52)
#define jaiabot_protobuf_ArduinoResponse_CALLBACK NULL
#define jaiabot_protobuf_ArduinoResponse_DEFAULT (const pb_byte_t*)"\xa0\x03\x00\x00"

#define jaiabot_protobuf_ArduinoDebug_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, BOOL,     arduino_restarted,   1) \
X(a, STATIC,   OPTIONAL, BOOL,     arduino_not_responding,   2)
#define jaiabot_protobuf_ArduinoDebug_CALLBACK NULL
#define jaiabot_protobuf_ArduinoDebug_DEFAULT (const pb_byte_t*)"\x08\x00\x10\x00\x00"

extern const pb_msgdesc_t jaiabot_protobuf_ArduinoSettings_msg;
extern const pb_msgdesc_t jaiabot_protobuf_ArduinoActuators_msg;
extern const pb_msgdesc_t jaiabot_protobuf_ArduinoCommand_msg;
extern const pb_msgdesc_t jaiabot_protobuf_ArduinoResponse_msg;
extern const pb_msgdesc_t jaiabot_protobuf_ArduinoDebug_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define jaiabot_protobuf_ArduinoSettings_fields &jaiabot_protobuf_ArduinoSettings_msg
#define jaiabot_protobuf_ArduinoActuators_fields &jaiabot_protobuf_ArduinoActuators_msg
#define jaiabot_protobuf_ArduinoCommand_fields &jaiabot_protobuf_ArduinoCommand_msg
#define jaiabot_protobuf_ArduinoResponse_fields &jaiabot_protobuf_ArduinoResponse_msg
#define jaiabot_protobuf_ArduinoDebug_fields &jaiabot_protobuf_ArduinoDebug_msg

/* Maximum encoded size of messages (where known) */
#define jaiabot_protobuf_ArduinoSettings_size    12
#define jaiabot_protobuf_ArduinoActuators_size   32
#define jaiabot_protobuf_ArduinoCommand_size     48
#define jaiabot_protobuf_ArduinoResponse_size    59
#define jaiabot_protobuf_ArduinoDebug_size       4

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
