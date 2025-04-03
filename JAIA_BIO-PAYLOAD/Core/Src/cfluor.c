#include "cfluor.h"

CFluor sFluorometer;

float getFluorometerVoltage(void) {
    return sFluorometer.voltage;
}

void getFluorometerConcentration(void) {
    sFluorometer.concentration = (sFluorometer.voltage - sFluorometer.offset) * sFluorometer.cal_coefficient;
}

int readFluorometer(void) {
    getFluorometerConcentration();

    return 0;
}