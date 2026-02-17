/**
 * AML (conductivity/temperature) sensor parser and getters.
 * Parses UART output: @Z metadata lines and " conductivity temperature" data lines.
 */

#include "aml.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* Internal state */
static double aml_conductivity = 0.0;
static double aml_temperature = 0.0;
static bool aml_data_valid = false;
static AML_Metadata aml_metadata = {0};

void parse_aml_uart1_buffer(const uint8_t *buf, uint16_t len)
{
  if (buf == NULL || len == 0)
    return;

  uint8_t copy[AML_UART_BUFFER_MAX];
  if (len >= sizeof(copy))
    len = (uint16_t)(sizeof(copy) - 1);
  memcpy(copy, buf, len);
  copy[len] = '\0';

  const char *p = (const char *)copy;
  const char *end = (const char *)copy + len;

  while (p < end)
  {
    while (p < end && (*p == '\r' || *p == '\n'))
      p++;
    if (p >= end)
      break;

    const char *line_start = p;

    while (p < end && *p != '\r' && *p != '\n')
      p++;

    const char *q = line_start;
    while (q < p && (*q == ' ' || *q == '\t'))
      q++;
    if (q >= p)
      continue;

    if (*q == '@')
    {
      /* Parse @Z metadata: @Z 451814 11/26/2024 0.008 11/26/2024 0.002 CT.X 54 */
      unsigned int serial = 0;
      char d1[16] = {0}, d2[16] = {0}, st[16] = {0};
      double v1 = 0.0, v2 = 0.0;
      int ver = 0;
      if (sscanf(line_start, " @Z %u %15s %lf %15s %lf %15s %d",
                 &serial, d1, &v1, d2, &v2, st, &ver) >= 7)
      {
        aml_metadata.serial_number = (uint32_t)serial;
        (void)strncpy(aml_metadata.date1, d1, sizeof(aml_metadata.date1) - 1);
        aml_metadata.date1[sizeof(aml_metadata.date1) - 1] = '\0';
        aml_metadata.value1 = v1;
        (void)strncpy(aml_metadata.date2, d2, sizeof(aml_metadata.date2) - 1);
        aml_metadata.date2[sizeof(aml_metadata.date2) - 1] = '\0';
        aml_metadata.value2 = v2;
        (void)strncpy(aml_metadata.sensor_type, st, sizeof(aml_metadata.sensor_type) - 1);
        aml_metadata.sensor_type[sizeof(aml_metadata.sensor_type) - 1] = '\0';
        aml_metadata.version_id = ver;
        aml_metadata.valid = true;
      }
      continue;
    }

    /* Data line: " conductivity temperature" */
    double c = 0.0, t = 0.0;
    if (sscanf(line_start, " %lf %lf", &c, &t) == 2)
    {
      aml_conductivity = c;
      aml_temperature = t;
      aml_data_valid = true;
    }
  }
}

bool getAmlMetadata(AML_Metadata *out)
{
  if (out == NULL || !aml_metadata.valid)
    return false;
  *out = aml_metadata;
  return true;
}

double getAMLConductivity(void)
{
  return aml_conductivity;
}

double getAMLTemp(void)
{
  return aml_temperature;
}

bool getAMLDataValid(void)
{
  return aml_data_valid;
}
