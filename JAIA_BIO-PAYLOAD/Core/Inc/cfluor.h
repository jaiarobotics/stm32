#ifndef INC_CFLUOR_H_
#define INC_CFLUOR_H_

#include "main.h"
#include "stdint.h"

typedef struct {
    float voltage;
    float concentration;    
    float offset;
    float cal_coefficient;
    uint32_t sn;
} CFluor;

extern CFluor sFluorometer;

float readFluorometer(void); // 
float calculateConcentration(void);

float getFluorometerVoltage(void);
float getFluorometerConcentration(void);
void setFluorometerOffset(float offset);
void setFluorometerCalCoefficient(float cal_coefficient);
void setFluorometerSN(uint32_t sn);


#endif /* INC_CFLUOR_H_ */