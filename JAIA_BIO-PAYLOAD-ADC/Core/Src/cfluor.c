#include "cfluor.h"

CFluor sFluorometer;

float readFluorometer(void) {
    return calculateConcentration();
}

float calculateConcentration(void) {
    return (sFluorometer.voltage - sFluorometer.offset) * sFluorometer.cal_coefficient;
}

float getFluorometerVoltage(void) {
    return sFluorometer.voltage;
}