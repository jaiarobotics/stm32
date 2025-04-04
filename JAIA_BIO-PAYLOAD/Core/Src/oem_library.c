#include "oem_library.h"

/* INITIALIZATION */
int initAtlasScientificEC()
{
  // Power on the EC sensor
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
  
  HAL_Delay(20);
  ec.devAddr = EC_OEM_I2C_ADDR;
  ec.devType = EC_OEM_DEV_TYPE;

  HAL_Delay(20);
  HAL_StatusTypeDef status = OEM_Activate(ec.i2cHandle, ec.devAddr);

  if (status != HAL_OK) 
  {
    Error_Handler();
  }

  return 0;
}

int initAtlasScientificDO()
{
  // Power on the DO sensor
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);

  HAL_Delay(20);
  dOxy.devAddr = DO_OEM_I2C_ADDR;
  dOxy.devType = DO_OEM_DEV_TYPE;
  
  HAL_Delay(20);
  HAL_StatusTypeDef status = OEM_Activate(dOxy.i2cHandle, dOxy.devAddr);
  
  if (status != HAL_OK) 
  {
    Error_Handler();
  }

  return 0;
}

int initAtlasScientificPH()
{
  // Power on the pH sensor
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);

  HAL_Delay(20);
  ph.devAddr = PH_OEM_I2C_ADDR;
  ph.devType = PH_OEM_DEV_TYPE;

  HAL_Delay(20);
  HAL_StatusTypeDef status = OEM_Activate(ph.i2cHandle, ph.devAddr);
  
  if (status != HAL_OK) 
  {
    Error_Handler();
  }

  return 0;
}

HAL_StatusTypeDef OEM_Activate(I2C_HandleTypeDef *i2cHandle, uint8_t *devAddr) {
    uint8_t activate_command = 0x01;
    return OEM_WriteRegister(i2cHandle, devAddr, OEM_REG_ACTIVATE, &activate_command);
}

HAL_StatusTypeDef OEM_Hibernate(I2C_HandleTypeDef *i2cHandle, uint8_t *devAddr) {
    uint8_t hibernate_command = 0x00;
    return OEM_WriteRegister(i2cHandle, devAddr, OEM_REG_ACTIVATE, &hibernate_command);
}


// /* COLLECT DATA */

HAL_StatusTypeDef get_ECReading() {
    uint8_t regData[4];
    float divFactor = 100.0f;
    HAL_StatusTypeDef status = HAL_OK;

    status = OEM_ReadRegisters(ec.i2cHandle, ec.devAddr, EC_REG_OEM_EC, &regData[0], 4);
    
    uint32_t regReading = (regData[0] << 24) | (regData[1] << 16) | (regData[2] << 8) | regData[3];
    ec.conductivity = (float)regReading / divFactor;

    return status;
}

HAL_StatusTypeDef get_DOReading() {
    uint8_t regData[4];
    unsigned long regReading;
    float divFactor = 100.0f;
    HAL_StatusTypeDef status = HAL_OK;

    status = OEM_ReadRegisters(dOxy.i2cHandle, dOxy.devAddr, DO_OEM_REG_DO, &regData[0], 4);
    regReading = ((uint32_t)regData[0] << 24) | ((uint32_t)regData[1] << 16) | ((uint32_t)regData[2] << 8) | regData[3];
    dOxy.dissolved_oxygen = (float)regReading / divFactor;

    status = OEM_ReadRegisters(dOxy.i2cHandle, dOxy.devAddr, DO_OEM_REG_DO_SATURATION, &regData[0], 4);
    regReading = ((uint32_t)regData[0] << 24) | ((uint32_t)regData[1] << 16) | ((uint32_t)regData[2] << 8) | regData[3];
    dOxy.dissolved_oxygen_saturation = (float)regReading / divFactor;

    // TODO: Get temperature from ADC branch, but for now use a constant 25 °C
    dOxy.temperature = 25.0f;

    return status;
}

HAL_StatusTypeDef get_PHReading() {
    uint8_t regData[4];
    unsigned long regReading;
    float divFactor = 1000.0f;
    HAL_StatusTypeDef status = HAL_OK;

    status = OEM_ReadRegisters(ph.i2cHandle, ph.devAddr, PH_OEM_REG_PH, &regData[0], 4);
    regReading = (regData[0] << 24) | (regData[1] << 16) | (regData[2] << 8) | regData[3];
    ph.ph = (float)regReading / divFactor;

    return status;
}


// /* 
//  * CALIBRATION DATA
//  */

// HAL_StatusTypeDef OEM_SetCalibration(OEM_CHIP *dev) {
//     // return
// }

// HAL_StatusTypeDef OEM_GetCalibration(OEM_CHIP *dev) {
//     // return
// }


// /* LOW-LEVEL FUNCTIONS */
HAL_StatusTypeDef OEM_ReadRegister(I2C_HandleTypeDef *i2cHandle, uint8_t devAddr, uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Read(i2cHandle, devAddr, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef OEM_ReadRegisters(I2C_HandleTypeDef *i2cHandle, uint8_t devAddr, uint8_t reg, uint8_t *data, uint8_t len) {
    return HAL_I2C_Mem_Read(i2cHandle, devAddr, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef OEM_WriteRegister(I2C_HandleTypeDef *i2cHandle, uint8_t devAddr, uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Write(i2cHandle, devAddr, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}


