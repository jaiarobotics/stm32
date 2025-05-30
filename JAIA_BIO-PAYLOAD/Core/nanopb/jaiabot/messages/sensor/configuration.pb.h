/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.1 */

#ifndef PB_JAIABOT_SENSOR_PROTOBUF_NANOPB_JAIABOT_MESSAGES_SENSOR_CONFIGURATION_PB_H_INCLUDED
#define PB_JAIABOT_SENSOR_PROTOBUF_NANOPB_JAIABOT_MESSAGES_SENSOR_CONFIGURATION_PB_H_INCLUDED
#include <pb.h>
#include "jaiabot/messages/sensor/catalog.pb.h"
#include "dccl/option_extensions.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _jaiabot_sensor_protobuf_Configuration_Cfg {
    char key[16];
    char value[64];
} jaiabot_sensor_protobuf_Configuration_Cfg;

typedef struct _jaiabot_sensor_protobuf_Configuration {
    jaiabot_sensor_protobuf_Sensor sensor;
    bool has_sample_freq;
    double sample_freq;
    pb_size_t cfg_count;
    jaiabot_sensor_protobuf_Configuration_Cfg cfg[16];
} jaiabot_sensor_protobuf_Configuration;


/* Initializer values for message structs */
#define jaiabot_sensor_protobuf_Configuration_init_default {_jaiabot_sensor_protobuf_Sensor_MIN, false, 0, 0, {jaiabot_sensor_protobuf_Configuration_Cfg_init_default, jaiabot_sensor_protobuf_Configuration_Cfg_init_default, jaiabot_sensor_protobuf_Configuration_Cfg_init_default, jaiabot_sensor_protobuf_Configuration_Cfg_init_default, jaiabot_sensor_protobuf_Configuration_Cfg_init_default, jaiabot_sensor_protobuf_Configuration_Cfg_init_default, jaiabot_sensor_protobuf_Configuration_Cfg_init_default, jaiabot_sensor_protobuf_Configuration_Cfg_init_default, jaiabot_sensor_protobuf_Configuration_Cfg_init_default, jaiabot_sensor_protobuf_Configuration_Cfg_init_default, jaiabot_sensor_protobuf_Configuration_Cfg_init_default, jaiabot_sensor_protobuf_Configuration_Cfg_init_default, jaiabot_sensor_protobuf_Configuration_Cfg_init_default, jaiabot_sensor_protobuf_Configuration_Cfg_init_default, jaiabot_sensor_protobuf_Configuration_Cfg_init_default, jaiabot_sensor_protobuf_Configuration_Cfg_init_default}}
#define jaiabot_sensor_protobuf_Configuration_Cfg_init_default {"", ""}
#define jaiabot_sensor_protobuf_Configuration_init_zero {_jaiabot_sensor_protobuf_Sensor_MIN, false, 0, 0, {jaiabot_sensor_protobuf_Configuration_Cfg_init_zero, jaiabot_sensor_protobuf_Configuration_Cfg_init_zero, jaiabot_sensor_protobuf_Configuration_Cfg_init_zero, jaiabot_sensor_protobuf_Configuration_Cfg_init_zero, jaiabot_sensor_protobuf_Configuration_Cfg_init_zero, jaiabot_sensor_protobuf_Configuration_Cfg_init_zero, jaiabot_sensor_protobuf_Configuration_Cfg_init_zero, jaiabot_sensor_protobuf_Configuration_Cfg_init_zero, jaiabot_sensor_protobuf_Configuration_Cfg_init_zero, jaiabot_sensor_protobuf_Configuration_Cfg_init_zero, jaiabot_sensor_protobuf_Configuration_Cfg_init_zero, jaiabot_sensor_protobuf_Configuration_Cfg_init_zero, jaiabot_sensor_protobuf_Configuration_Cfg_init_zero, jaiabot_sensor_protobuf_Configuration_Cfg_init_zero, jaiabot_sensor_protobuf_Configuration_Cfg_init_zero, jaiabot_sensor_protobuf_Configuration_Cfg_init_zero}}
#define jaiabot_sensor_protobuf_Configuration_Cfg_init_zero {"", ""}

/* Field tags (for use in manual encoding/decoding) */
#define jaiabot_sensor_protobuf_Configuration_Cfg_key_tag 1
#define jaiabot_sensor_protobuf_Configuration_Cfg_value_tag 2
#define jaiabot_sensor_protobuf_Configuration_sensor_tag 1
#define jaiabot_sensor_protobuf_Configuration_sample_freq_tag 2
#define jaiabot_sensor_protobuf_Configuration_cfg_tag 3

/* Struct field encoding specification for nanopb */
#define jaiabot_sensor_protobuf_Configuration_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UENUM,    sensor,            1) \
X(a, STATIC,   OPTIONAL, DOUBLE,   sample_freq,       2) \
X(a, STATIC,   REPEATED, MESSAGE,  cfg,               3)
#define jaiabot_sensor_protobuf_Configuration_CALLBACK NULL
#define jaiabot_sensor_protobuf_Configuration_DEFAULT NULL
#define jaiabot_sensor_protobuf_Configuration_cfg_MSGTYPE jaiabot_sensor_protobuf_Configuration_Cfg

#define jaiabot_sensor_protobuf_Configuration_Cfg_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, STRING,   key,               1) \
X(a, STATIC,   REQUIRED, STRING,   value,             2)
#define jaiabot_sensor_protobuf_Configuration_Cfg_CALLBACK NULL
#define jaiabot_sensor_protobuf_Configuration_Cfg_DEFAULT NULL

extern const pb_msgdesc_t jaiabot_sensor_protobuf_Configuration_msg;
extern const pb_msgdesc_t jaiabot_sensor_protobuf_Configuration_Cfg_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define jaiabot_sensor_protobuf_Configuration_fields &jaiabot_sensor_protobuf_Configuration_msg
#define jaiabot_sensor_protobuf_Configuration_Cfg_fields &jaiabot_sensor_protobuf_Configuration_Cfg_msg

/* Maximum encoded size of messages (where known) */
#define jaiabot_sensor_protobuf_Configuration_size 1355
#define jaiabot_sensor_protobuf_Configuration_Cfg_size 82

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
