/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.1 */

#ifndef PB_JAIABOT_SENSOR_PROTOBUF_NANOPB_JAIABOT_MESSAGES_SENSOR_BLUE_ROBOTICS__BAR30_PB_H_INCLUDED
#define PB_JAIABOT_SENSOR_PROTOBUF_NANOPB_JAIABOT_MESSAGES_SENSOR_BLUE_ROBOTICS__BAR30_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _jaiabot_sensor_protobuf_BlueRoboticsBar30 {
    bool has_pressure;
    double pressure;
    bool has_temperature;
    double temperature;
} jaiabot_sensor_protobuf_BlueRoboticsBar30;


/* Initializer values for message structs */
#define jaiabot_sensor_protobuf_BlueRoboticsBar30_init_default {false, 0, false, 0}
#define jaiabot_sensor_protobuf_BlueRoboticsBar30_init_zero {false, 0, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define jaiabot_sensor_protobuf_BlueRoboticsBar30_pressure_tag 1
#define jaiabot_sensor_protobuf_BlueRoboticsBar30_temperature_tag 2

/* Struct field encoding specification for nanopb */
#define jaiabot_sensor_protobuf_BlueRoboticsBar30_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, DOUBLE,   pressure,          1) \
X(a, STATIC,   OPTIONAL, DOUBLE,   temperature,       2)
#define jaiabot_sensor_protobuf_BlueRoboticsBar30_CALLBACK NULL
#define jaiabot_sensor_protobuf_BlueRoboticsBar30_DEFAULT NULL

extern const pb_msgdesc_t jaiabot_sensor_protobuf_BlueRoboticsBar30_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define jaiabot_sensor_protobuf_BlueRoboticsBar30_fields &jaiabot_sensor_protobuf_BlueRoboticsBar30_msg

/* Maximum encoded size of messages (where known) */
#define jaiabot_sensor_protobuf_BlueRoboticsBar30_size 18

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
