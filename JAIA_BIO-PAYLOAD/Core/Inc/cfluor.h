#ifndef INC_CFluor_H_
#define INC_CFluor_H_

#include "main.h"

typedef struct CFluor
{
    I2C_HandleTypeDef* pi2c;
    float offset;
    float cal_coefficient;
    float concentration;
    float concentration_voltage;
} CFluor;

extern CFluor sFluorometer;  

int readCFluor(void);
void initCFluor(void);
void set_CFluorOffset(float offset);
void set_CFluorCalCoefficient(float cal_coefficient);
float getConcentration(void);
float convert_3_3_to_5_0(float voltage);
float getConcentrationVoltage(void);

#endif /* INC_CFluor_H_ */
