#ifndef INC_CFluor_H_
#define INC_CFluor_H_

#include "main.h"

typedef struct CFluor
{
    I2C_HandleTypeDef* pi2c;
    float offset;
    float cal_coefficient;
    float concentration;

} CFluor;

extern CFluor sFluorometer;  

int readCFluor(void);
void initCFluor(void);
void setOffset(float offset);
void setCalCoefficient(float cal_coefficient);
float getConcentration(void);
float convert_3_3_to_5_0(float voltage);

#endif /* INC_CFluor_H_ */
