/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.5 */

#ifndef PB_JAIABOT_SENSOR_PROTOBUF_NANOPB_JAIABOT_MESSAGES_SENSOR_SENSOR_CORE_PB_H_INCLUDED
#define PB_JAIABOT_SENSOR_PROTOBUF_NANOPB_JAIABOT_MESSAGES_SENSOR_SENSOR_CORE_PB_H_INCLUDED
#include <pb.h>
#include "dccl/option_extensions.pb.h"
#include "jaiabot/messages/sensor/metadata.pb.h"
#include "jaiabot/messages/sensor/configuration.pb.h"
#include "jaiabot/messages/sensor/atlas_scientific__oem_ec.pb.h"
#include "jaiabot/messages/sensor/atlas_scientific__oem_do.pb.h"
#include "jaiabot/messages/sensor/atlas_scientific__oem_ph.pb.h"
#include "jaiabot/messages/sensor/turner__c_fluor.pb.h"
#include "jaiabot/messages/sensor/blue_robotics__bar30.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _jaiabot_sensor_protobuf_MCUCommand { 
    jaiabot_sensor_protobuf_MCUCommand_ENTER_BOOTLOADER_MODE = 1 
} jaiabot_sensor_protobuf_MCUCommand;

typedef enum _jaiabot_sensor_protobuf_CalibrationCommand { 
    jaiabot_sensor_protobuf_CalibrationCommand_START_EC_CALIBRATION = 1, 
    jaiabot_sensor_protobuf_CalibrationCommand_CALIBRATE_EC_DRY = 2, 
    jaiabot_sensor_protobuf_CalibrationCommand_CALIBRATE_EC_LOW = 3, 
    jaiabot_sensor_protobuf_CalibrationCommand_CALIBRATE_EC_HIGH = 4, 
    jaiabot_sensor_protobuf_CalibrationCommand_START_PH_CALIBRATION = 5, 
    jaiabot_sensor_protobuf_CalibrationCommand_CALIBRATE_PH_LOW = 6, 
    jaiabot_sensor_protobuf_CalibrationCommand_CALIBRATE_PH_MID = 7, 
    jaiabot_sensor_protobuf_CalibrationCommand_CALIBRATE_PH_HIGH = 8, 
    jaiabot_sensor_protobuf_CalibrationCommand_START_DO_CALIBRATION = 9, 
    jaiabot_sensor_protobuf_CalibrationCommand_CALIBRATE_DO_LOW = 10, 
    jaiabot_sensor_protobuf_CalibrationCommand_CALIBRATE_DO_HIGH = 11, 
    jaiabot_sensor_protobuf_CalibrationCommand_STOP_CALIBRATION = 12 
} jaiabot_sensor_protobuf_CalibrationCommand;

/* Struct definitions */
typedef struct _jaiabot_sensor_protobuf_SensorData { 
    uint64_t time; 
    pb_size_t which_data;
    union {
        jaiabot_sensor_protobuf_Metadata metadata;
        jaiabot_sensor_protobuf_AtlasScientificOEMEC oem_ec;
        jaiabot_sensor_protobuf_BlueRoboticsBar30 bar30;
        jaiabot_sensor_protobuf_AtlasScientificOEMpH oem_ph;
        jaiabot_sensor_protobuf_AtlasScientificOEMDO oem_do;
        jaiabot_sensor_protobuf_TurnerCFluor c_fluor;
    } data; 
} jaiabot_sensor_protobuf_SensorData;

typedef struct _jaiabot_sensor_protobuf_SensorRequest { 
    uint64_t time; 
    pb_size_t which_request_data;
    union {
        bool request_metadata;
        jaiabot_sensor_protobuf_Configuration cfg;
    } request_data; 
    bool has_mcu_command;
    jaiabot_sensor_protobuf_MCUCommand mcu_command; 
    bool has_calibration_command;
    jaiabot_sensor_protobuf_CalibrationCommand calibration_command; 
} jaiabot_sensor_protobuf_SensorRequest;

typedef struct _jaiabot_sensor_protobuf_SensorThreadConfig { 
    bool has_metadata;
    jaiabot_sensor_protobuf_Metadata metadata; 
    bool has_sample_rate;
    int32_t sample_rate; 
} jaiabot_sensor_protobuf_SensorThreadConfig;


/* Helper constants for enums */
#define _jaiabot_sensor_protobuf_MCUCommand_MIN jaiabot_sensor_protobuf_MCUCommand_ENTER_BOOTLOADER_MODE
#define _jaiabot_sensor_protobuf_MCUCommand_MAX jaiabot_sensor_protobuf_MCUCommand_ENTER_BOOTLOADER_MODE
#define _jaiabot_sensor_protobuf_MCUCommand_ARRAYSIZE ((jaiabot_sensor_protobuf_MCUCommand)(jaiabot_sensor_protobuf_MCUCommand_ENTER_BOOTLOADER_MODE+1))

#define _jaiabot_sensor_protobuf_CalibrationCommand_MIN jaiabot_sensor_protobuf_CalibrationCommand_START_EC_CALIBRATION
#define _jaiabot_sensor_protobuf_CalibrationCommand_MAX jaiabot_sensor_protobuf_CalibrationCommand_STOP_CALIBRATION
#define _jaiabot_sensor_protobuf_CalibrationCommand_ARRAYSIZE ((jaiabot_sensor_protobuf_CalibrationCommand)(jaiabot_sensor_protobuf_CalibrationCommand_STOP_CALIBRATION+1))


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define jaiabot_sensor_protobuf_SensorRequest_init_default {0, 0, {0}, false, _jaiabot_sensor_protobuf_MCUCommand_MIN, false, _jaiabot_sensor_protobuf_CalibrationCommand_MIN}
#define jaiabot_sensor_protobuf_SensorData_init_default {0, 0, {jaiabot_sensor_protobuf_Metadata_init_default}}
#define jaiabot_sensor_protobuf_SensorThreadConfig_init_default {false, jaiabot_sensor_protobuf_Metadata_init_default, false, 0}
#define jaiabot_sensor_protobuf_SensorRequest_init_zero {0, 0, {0}, false, _jaiabot_sensor_protobuf_MCUCommand_MIN, false, _jaiabot_sensor_protobuf_CalibrationCommand_MIN}
#define jaiabot_sensor_protobuf_SensorData_init_zero {0, 0, {jaiabot_sensor_protobuf_Metadata_init_zero}}
#define jaiabot_sensor_protobuf_SensorThreadConfig_init_zero {false, jaiabot_sensor_protobuf_Metadata_init_zero, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define jaiabot_sensor_protobuf_SensorData_time_tag 1
#define jaiabot_sensor_protobuf_SensorData_metadata_tag 11
#define jaiabot_sensor_protobuf_SensorData_oem_ec_tag 12
#define jaiabot_sensor_protobuf_SensorData_bar30_tag 13
#define jaiabot_sensor_protobuf_SensorData_oem_ph_tag 14
#define jaiabot_sensor_protobuf_SensorData_oem_do_tag 15
#define jaiabot_sensor_protobuf_SensorData_c_fluor_tag 16
#define jaiabot_sensor_protobuf_SensorRequest_time_tag 1
#define jaiabot_sensor_protobuf_SensorRequest_request_metadata_tag 11
#define jaiabot_sensor_protobuf_SensorRequest_cfg_tag 12
#define jaiabot_sensor_protobuf_SensorRequest_mcu_command_tag 20
#define jaiabot_sensor_protobuf_SensorRequest_calibration_command_tag 21
#define jaiabot_sensor_protobuf_SensorThreadConfig_metadata_tag 1
#define jaiabot_sensor_protobuf_SensorThreadConfig_sample_rate_tag 2

/* Struct field encoding specification for nanopb */
#define jaiabot_sensor_protobuf_SensorRequest_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT64,   time,              1) \
X(a, STATIC,   ONEOF,    BOOL,     (request_data,request_metadata,request_data.request_metadata),  11) \
X(a, STATIC,   ONEOF,    MESSAGE,  (request_data,cfg,request_data.cfg),  12) \
X(a, STATIC,   OPTIONAL, UENUM,    mcu_command,      20) \
X(a, STATIC,   OPTIONAL, UENUM,    calibration_command,  21)
#define jaiabot_sensor_protobuf_SensorRequest_CALLBACK NULL
#define jaiabot_sensor_protobuf_SensorRequest_DEFAULT (const pb_byte_t*)"\xa0\x01\x01\xa8\x01\x01\x00"
#define jaiabot_sensor_protobuf_SensorRequest_request_data_cfg_MSGTYPE jaiabot_sensor_protobuf_Configuration

#define jaiabot_sensor_protobuf_SensorData_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT64,   time,              1) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,metadata,data.metadata),  11) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,oem_ec,data.oem_ec),  12) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,bar30,data.bar30),  13) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,oem_ph,data.oem_ph),  14) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,oem_do,data.oem_do),  15) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,c_fluor,data.c_fluor),  16)
#define jaiabot_sensor_protobuf_SensorData_CALLBACK NULL
#define jaiabot_sensor_protobuf_SensorData_DEFAULT NULL
#define jaiabot_sensor_protobuf_SensorData_data_metadata_MSGTYPE jaiabot_sensor_protobuf_Metadata
#define jaiabot_sensor_protobuf_SensorData_data_oem_ec_MSGTYPE jaiabot_sensor_protobuf_AtlasScientificOEMEC
#define jaiabot_sensor_protobuf_SensorData_data_bar30_MSGTYPE jaiabot_sensor_protobuf_BlueRoboticsBar30
#define jaiabot_sensor_protobuf_SensorData_data_oem_ph_MSGTYPE jaiabot_sensor_protobuf_AtlasScientificOEMpH
#define jaiabot_sensor_protobuf_SensorData_data_oem_do_MSGTYPE jaiabot_sensor_protobuf_AtlasScientificOEMDO
#define jaiabot_sensor_protobuf_SensorData_data_c_fluor_MSGTYPE jaiabot_sensor_protobuf_TurnerCFluor

#define jaiabot_sensor_protobuf_SensorThreadConfig_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  metadata,          1) \
X(a, STATIC,   OPTIONAL, INT32,    sample_rate,       2)
#define jaiabot_sensor_protobuf_SensorThreadConfig_CALLBACK NULL
#define jaiabot_sensor_protobuf_SensorThreadConfig_DEFAULT NULL
#define jaiabot_sensor_protobuf_SensorThreadConfig_metadata_MSGTYPE jaiabot_sensor_protobuf_Metadata

extern const pb_msgdesc_t jaiabot_sensor_protobuf_SensorRequest_msg;
extern const pb_msgdesc_t jaiabot_sensor_protobuf_SensorData_msg;
extern const pb_msgdesc_t jaiabot_sensor_protobuf_SensorThreadConfig_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define jaiabot_sensor_protobuf_SensorRequest_fields &jaiabot_sensor_protobuf_SensorRequest_msg
#define jaiabot_sensor_protobuf_SensorData_fields &jaiabot_sensor_protobuf_SensorData_msg
#define jaiabot_sensor_protobuf_SensorThreadConfig_fields &jaiabot_sensor_protobuf_SensorThreadConfig_msg

/* Maximum encoded size of messages (where known) */
#define jaiabot_sensor_protobuf_SensorData_size  1868
#define jaiabot_sensor_protobuf_SensorRequest_size 1375
#define jaiabot_sensor_protobuf_SensorThreadConfig_size 1868

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
