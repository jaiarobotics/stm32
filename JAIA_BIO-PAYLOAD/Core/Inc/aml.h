#ifndef INC_AML_H_
#define INC_AML_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "nanopb/jaiabot/messages/sensor/sensor_core.pb.h"

/**
 * AML sensor metadata parsed from the @Z version/identification block.
 * Example: @Z 451814 11/26/2024 0.008 11/26/2024 0.002 CT.X 54
 */
typedef struct
{
    uint32_t serial_number;       /* e.g. 451814 */
    char date1[16];               /* e.g. "11/26/2024" */
    double value1;                /* e.g. 0.008 */
    char date2[16];                /* e.g. "11/26/2024" */
    double value2;                /* e.g. 0.002 */
    char sensor_type[16];         /* e.g. "CT.X" */
    int version_id;               /* e.g. 54 */
    bool valid;                   /* true if metadata has been parsed at least once */
} AML_Metadata;

typedef struct
{
    double conductivity;
    double temperature;
    bool is_valid; 
} AML_Reading;

#define AML_UART_BUFFER_MAX 256

/**
 * Parse AML sensor output from UART1 buffer.
 * Handles @Z metadata lines and " conductivity temperature" data lines.
 * Call this when new data is available in the buffer (e.g. from UART RX callback).
 * @return true if at least one line (metadata or data) was successfully parsed
 */
bool parseAMLData(const uint8_t *in_buffer, uint16_t len);

/**
 * Fill metadata from the last parsed @ block.
 * Returns true if metadata has been parsed at least once, false otherwise.
 */
bool getAmlMetadata(AML_Metadata *out);

/**
 * Get last parsed conductivity (mS/cm). Only meaningful if data has been parsed.
 */
double getAMLConductivity(void);

/**
 * Get last parsed temperature (°C). Only meaningful if data has been parsed.
 */
double getAMLTemp(void);

/**
 * Returns true if at least one conductivity/temperature data line has been parsed.
 */
HAL_StatusTypeDef getAMLDataValid(void);

void transmit_aml_data(void);

#endif /* INC_AML_H_ */
