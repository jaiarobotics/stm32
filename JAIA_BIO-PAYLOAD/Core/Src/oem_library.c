#include "oem_library.h"

/* INITIALIZATION */
int initAtlasScientificEC()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  HAL_Delay(20);
  ec.devAddr = EC_OEM_I2C_ADDR;
  ec.devType = OEM_ReadRegister(ec.i2cHandle, ec.devAddr, OEM_REG_DEV_TYPE, &ec.devType);

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

  HAL_Delay(20);
  dOxy.devAddr = DO_OEM_I2C_ADDR;
  dOxy.devType = OEM_ReadRegister(dOxy.i2cHandle, dOxy.devAddr, OEM_REG_DEV_TYPE, &dOxy.devType);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  HAL_Delay(20);
  ph.devAddr = PH_OEM_I2C_ADDR;
  ph.devType = OEM_ReadRegister(ph.i2cHandle, ph.devAddr, OEM_REG_DEV_TYPE, &ph.devType);

  HAL_Delay(20);
  HAL_StatusTypeDef status = OEM_Activate(ph.i2cHandle, ph.devAddr);

  if (status != HAL_OK)
  {
    Error_Handler();
  }

  return 0;
}

HAL_StatusTypeDef OEM_Activate(I2C_HandleTypeDef *i2cHandle, uint8_t *devAddr)
{
  uint8_t activate_command = 0x01;
  return OEM_WriteRegister(i2cHandle, devAddr, OEM_REG_ACTIVATE, &activate_command);
}

HAL_StatusTypeDef OEM_Hibernate(I2C_HandleTypeDef *i2cHandle, uint8_t *devAddr)
{
  uint8_t hibernate_command = 0x00;
  return OEM_WriteRegister(i2cHandle, devAddr, OEM_REG_ACTIVATE, &hibernate_command);
}

/* COLLECT DATA */
HAL_StatusTypeDef get_ECReading()
{
  uint8_t regData[4];
  float divFactor = 100.0f;
  HAL_StatusTypeDef status = HAL_OK;

  status = OEM_ReadRegisters(ec.i2cHandle, ec.devAddr, EC_REG_OEM_EC, &regData[0], 4);

  uint32_t regReading = (regData[0] << 24) | (regData[1] << 16) | (regData[2] << 8) | regData[3];
  ec.conductivity = (float)regReading / divFactor;

  return status;
}

HAL_StatusTypeDef get_DOReading()
{
  uint8_t regData[4];
  float divFactor = 100.0f;
  HAL_StatusTypeDef status = HAL_OK;

  // Dissovled Oxygen
  status = OEM_ReadRegisters(dOxy.i2cHandle, dOxy.devAddr, DO_REG_OEM_DO, &regData[0], 4);
  uint32_t regReading = (regData[0] << 24) | (regData[1] << 16) | (regData[2] << 8) | regData[3];
  dOxy.dissolved_oxygen = (float)regReading / divFactor;

  // Temperature
  dOxy.temperature = OEM_ConvertVoltageToTemperature(adc_voltage5);

  return status;
}

HAL_StatusTypeDef get_PHReading()
{
  uint8_t regData[4];
  float divFactor = 1000.0f;
  HAL_StatusTypeDef status = HAL_OK;

  // pH
  status = OEM_ReadRegisters(ph.i2cHandle, ph.devAddr, PH_REG_OEM_PH, &regData[0], 4);
  uint32_t regReading = (regData[0] << 24) | (regData[1] << 16) | (regData[2] << 8) | regData[3];
  ph.ph = (float)regReading / divFactor;

  // Temperature
  ph.temperature = OEM_ConvertVoltageToTemperature(adc_voltage4);

  return status;
}

double OEM_ConvertVoltageToTemperature(double voltage)
{
    double Rt = 10000 / ((3.3 / voltage) - 1);
    double temp = -(sqrt(-0.00232 * Rt + 17.59246) - 3.908) / 0.00116;

    return temp;
}

/* GETTERS*/
double getConductivity() { return ec.conductivity; }
double getTDS() { return ec.total_dissolved_solids; }
double getSalinity() { return ec.salinity; }
double getDO() { return dOxy.temperature; }
double getDOTemperature() { return dOxy.temperature; }
double getPH() { return ph.ph; }
double getPHTemperature() { return ph.temperature; }

/* 
 * CALIBRATION DATA
 */

// HAL_StatusTypeDef OEM_SetCalibration(OEM_CHIP *dev) {
//     return
// }

// HAL_StatusTypeDef OEM_GetCalibration(OEM_CHIP *dev) {
//     return
// }


/* LOW-LEVEL FUNCTIONS */
HAL_StatusTypeDef OEM_ReadRegister(I2C_HandleTypeDef *i2cHandle, uint8_t devAddr, uint8_t reg, uint8_t *data)
{
  return HAL_I2C_Mem_Read(i2cHandle, devAddr, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef OEM_ReadRegisters(I2C_HandleTypeDef *i2cHandle, uint8_t devAddr, uint8_t reg, uint8_t *data, uint8_t len)
{
  return HAL_I2C_Mem_Read(i2cHandle, devAddr, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef OEM_WriteRegister(I2C_HandleTypeDef *i2cHandle, uint8_t devAddr, uint8_t reg, uint8_t *data)
{
  return HAL_I2C_Mem_Write(i2cHandle, devAddr, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
