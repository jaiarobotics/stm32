/*
 * MS5837.h
 *
 *  Created on: Mar 4, 2025
 *      Author: ColinVincent
 */

#ifndef INC_MS5837_H_
#define INC_MS5837_H_

#include "stdint.h"

#include "main.h"

typedef struct MS5837
{
  int fluidDensity;
  I2C_HandleTypeDef* pi2c;
  int model;
  int pressure;
  int temp;
} sMS5837;

int readMS5837(void);
int initMS5837(I2C_HandleTypeDef* i2cHandle);

void setModel(uint8_t model);
uint8_t getModel(void);

void setFluidDensity(float density);
float getPressure(float conversion);

float getTemp(void);
float getAltitude(void);
float getDepth(void);

extern sMS5837 sDepth;

#endif /* INC_MS5837_H_ */
