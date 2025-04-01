#include "cfluor.h"

CFluor sFluorometer;

float readFluorometer(void) {
    return calculateConcentration();
}

float getFluorometerVoltage(void) {
    return sFluorometer.voltage;
}

float getFluorometerConcentration(void) {
    return (sFluorometer.voltage - sFluorometer.offset) * sFluorometer.cal_coefficient;
}