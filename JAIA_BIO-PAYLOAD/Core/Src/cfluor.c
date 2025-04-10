#include "cfluor.h"

CFluor sFluorometer;

int readCFluor()
{
    float voltage = convert_3_3_to_5_0(adc_voltage1);
    sFluorometer.concentration = (voltage - sFluorometer.offset) * sFluorometer.cal_coefficient;

    return 0;
}

float convert_3_3_to_5_0(float voltage)
{
    return voltage * (5.0f / 3.3f);
}

void initCFluor()
{
    sFluorometer.offset = 0.0f;
    sFluorometer.cal_coefficient = 1.0f;
}

void setOffset(float offset)
{
    sFluorometer.offset = offset;
}

void setCalCoefficient(float cal_coefficient)
{
    sFluorometer.cal_coefficient = cal_coefficient;
}

float getConcentration()
{
    return sFluorometer.concentration;
}
