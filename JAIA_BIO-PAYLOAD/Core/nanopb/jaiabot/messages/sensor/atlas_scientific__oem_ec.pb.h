/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.1 */

#ifndef PB_JAIABOT_SENSOR_PROTOBUF_NANOPB_JAIABOT_MESSAGES_SENSOR_ATLAS_SCIENTIFIC__OEM_EC_PB_H_INCLUDED
#define PB_JAIABOT_SENSOR_PROTOBUF_NANOPB_JAIABOT_MESSAGES_SENSOR_ATLAS_SCIENTIFIC__OEM_EC_PB_H_INCLUDED
#include <pb.h>
#include "dccl/option_extensions.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _jaiabot_sensor_protobuf_AtlasScientificOEMEC {
    bool has_conductivity_raw;
    double conductivity_raw;
    bool has_conductivity;
    double conductivity;
    bool has_total_dissolved_solids;
    double total_dissolved_solids;
    bool has_salinity_raw;
    double salinity_raw;
    bool has_salinity;
    double salinity;
} jaiabot_sensor_protobuf_AtlasScientificOEMEC;


/* Initializer values for message structs */
#define jaiabot_sensor_protobuf_AtlasScientificOEMEC_init_default {false, 0, false, 0, false, 0, false, 0, false, 0}
#define jaiabot_sensor_protobuf_AtlasScientificOEMEC_init_zero {false, 0, false, 0, false, 0, false, 0, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define jaiabot_sensor_protobuf_AtlasScientificOEMEC_conductivity_raw_tag 1
#define jaiabot_sensor_protobuf_AtlasScientificOEMEC_conductivity_tag 2
#define jaiabot_sensor_protobuf_AtlasScientificOEMEC_total_dissolved_solids_tag 3
#define jaiabot_sensor_protobuf_AtlasScientificOEMEC_salinity_raw_tag 4
#define jaiabot_sensor_protobuf_AtlasScientificOEMEC_salinity_tag 5

/* Struct field encoding specification for nanopb */
#define jaiabot_sensor_protobuf_AtlasScientificOEMEC_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, DOUBLE,   conductivity_raw,   1) \
X(a, STATIC,   OPTIONAL, DOUBLE,   conductivity,      2) \
X(a, STATIC,   OPTIONAL, DOUBLE,   total_dissolved_solids,   3) \
X(a, STATIC,   OPTIONAL, DOUBLE,   salinity_raw,      4) \
X(a, STATIC,   OPTIONAL, DOUBLE,   salinity,          5)
#define jaiabot_sensor_protobuf_AtlasScientificOEMEC_CALLBACK NULL
#define jaiabot_sensor_protobuf_AtlasScientificOEMEC_DEFAULT NULL

extern const pb_msgdesc_t jaiabot_sensor_protobuf_AtlasScientificOEMEC_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define jaiabot_sensor_protobuf_AtlasScientificOEMEC_fields &jaiabot_sensor_protobuf_AtlasScientificOEMEC_msg

/* Maximum encoded size of messages (where known) */
#define jaiabot_sensor_protobuf_AtlasScientificOEMEC_size 45

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
