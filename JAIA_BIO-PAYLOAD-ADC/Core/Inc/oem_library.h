#ifndef INC_OEM_LIBRARY_H_
#define INC_OEM_LIBRARY_H_

#include "stm32l4xx_hal.h" /* Needed for I2C */
#include "stdint.h"

#include <string.h>


/* I2C ADDRESSES */
#define EC_OEM_I2C_ADDR                     (0x64 << 1)
#define PH_OEM_I2C_ADDR                     (0x65 << 1)
#define DO_OEM_I2C_ADDR                     (0x67 << 1)


/* REGISTER ADDRESS ENUMS */

    /* Universal register addresses */
#define OEM_REG_DEV_TYPE                    0x00
#define OEM_REG_LED                         0x05
#define OEM_REG_ACTIVATE                    0x06    // Send command 0x01 to activate, 0x00 to hibernate.

    /* EC Chip register addresses */ 
typedef enum {
    EC_REG_OEM_DEV_TYPE  = 0x04,            // EC device type
    EC_REG_OEM_CAL       = 0x0A,            // EC Calibration MSB (4 bytes wide, 0x0A-0x0D)
    EC_REG_OEM_CAL_REQ   = 0x0E,            // EC Calibration Request (1 byte wide, 0x0E)
    EC_REG_OEM_CAL_CONF  = 0x0F,            // EC Calibration Configuration (1 byte wide, 0x0F) 
    EC_REG_OEM_TEMP_COMP = 0x10,            // EC Temperature Compensation (4 bytes wide, 0x10-0x13)
    EC_REG_OEM_TEMP_CONF = 0x14,            // EC Temperature Configuration (4 bytes wide, 0x14-0x17)
    EC_REG_OEM_EC        = 0x18,            // EC Most Significant Byte (4 bytes wide, 0x18-0x1B)
} EC_Registers;

    /* pH Chip register addresses */
typedef enum {
    PH_REG_OEM_DEV_TYPE = 0x01,
    PH_REG_OEM_REG_PH = 0x16,
} PH_Registers;

    /* DO Chip register addresses */
typedef enum {
    DO_REG_OEM_DEV_TYPE = 0x03,
    DO_REG_OEM_DO = 0x22,
    DO_REG_OEM_DO_SAT = 0x26,
} DO_Registers;

/* SENSOR STRUCT */
typedef struct {
    /* I2C handle */
    I2C_HandleTypeDef *i2cHandle;

    /* Reading coming from Atlas Scientific sensor */
    uint32_t reading;

    /* Temperature reading coming from Atlas Scientific DO or pH sensors */
    float temperature;

    /* Device type */
    uint8_t devType;

    /* Device I2C address*/
    uint8_t devAddr;
} OEM_CHIP;

typedef enum {
    PH      = 0x01,
    DO      = 0x03,
    EC      = 0x04,
} OEM_devTypeEnum;


/* INITIALIZATION */
HAL_StatusTypeDef OEM_Init(OEM_CHIP *dev, I2C_HandleTypeDef *i2cHandle);
HAL_StatusTypeDef OEM_Activate(OEM_CHIP *dev);
HAL_StatusTypeDef OEM_Hibernate(OEM_CHIP *dev);


/* COLLECT DATA */
HAL_StatusTypeDef OEM_ReadAllChips(OEM_CHIP *ec, OEM_CHIP *ph, OEM_CHIP *dOxy);   // Read data from all chips with one function call
HAL_StatusTypeDef OEM_ReadData(OEM_CHIP *dev);                                  // Universal read function for OEM chips
HAL_StatusTypeDef OEM_GetDeviceType(OEM_CHIP *dev);
float calc_oem_temp(float temperature_resistance);


/* CALIBRATION */
HAL_StatusTypeDef OEM_SetCalibration(OEM_CHIP *dev);
HAL_StatusTypeDef OEM_GetCalibration(OEM_CHIP *dev);


/* LOW-LEVEL FUNCTIONS */
HAL_StatusTypeDef OEM_ReadRegister(OEM_CHIP *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef OEM_ReadRegisters(OEM_CHIP *dev, uint8_t reg, uint8_t *data, uint8_t len);
HAL_StatusTypeDef OEM_WriteRegister(OEM_CHIP *dev, uint8_t reg, uint8_t *data);

#endif