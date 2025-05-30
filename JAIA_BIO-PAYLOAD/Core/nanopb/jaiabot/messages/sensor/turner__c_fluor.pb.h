/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.1 */

#ifndef PB_JAIABOT_SENSOR_PROTOBUF_NANOPB_JAIABOT_MESSAGES_SENSOR_TURNER__C_FLUOR_PB_H_INCLUDED
#define PB_JAIABOT_SENSOR_PROTOBUF_NANOPB_JAIABOT_MESSAGES_SENSOR_TURNER__C_FLUOR_PB_H_INCLUDED
#include <pb.h>
#include "dccl/option_extensions.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _jaiabot_sensor_protobuf_TurnerCFluor {
    bool has_concentration;
    double concentration;
    bool has_concentration_voltage;
    double concentration_voltage;
} jaiabot_sensor_protobuf_TurnerCFluor;


/* Initializer values for message structs */
#define jaiabot_sensor_protobuf_TurnerCFluor_init_default {false, 0, false, 0}
#define jaiabot_sensor_protobuf_TurnerCFluor_init_zero {false, 0, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define jaiabot_sensor_protobuf_TurnerCFluor_concentration_tag 1
#define jaiabot_sensor_protobuf_TurnerCFluor_concentration_voltage_tag 2

/* Struct field encoding specification for nanopb */
#define jaiabot_sensor_protobuf_TurnerCFluor_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, DOUBLE,   concentration,     1) \
X(a, STATIC,   OPTIONAL, DOUBLE,   concentration_voltage,   2)
#define jaiabot_sensor_protobuf_TurnerCFluor_CALLBACK NULL
#define jaiabot_sensor_protobuf_TurnerCFluor_DEFAULT NULL

extern const pb_msgdesc_t jaiabot_sensor_protobuf_TurnerCFluor_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define jaiabot_sensor_protobuf_TurnerCFluor_fields &jaiabot_sensor_protobuf_TurnerCFluor_msg

/* Maximum encoded size of messages (where known) */
#define jaiabot_sensor_protobuf_TurnerCFluor_size 18

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
