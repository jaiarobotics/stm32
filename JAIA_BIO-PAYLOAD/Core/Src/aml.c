/**
 * AML (conductivity/temperature) sensor parser and getters.
 * Parses UART output: @Z metadata lines and conductivity/temperature data lines.
 * Uses line accumulation to handle fragmented UART receives.
 */

#include "aml.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

void parse_line(const char *buffer, AML_Reading* reading)
{
  if (sscanf(buffer, "%lf %lf", &reading->conductivity, &reading->temperature) == 2)
  {
    reading->is_valid = true;
  } else {
    reading->is_valid = false;
  }
}


