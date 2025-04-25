/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef jaiabot_sensor_protobuf_SensorData SensorData;
typedef jaiabot_sensor_protobuf_SensorRequest SensorRequest;
typedef jaiabot_sensor_protobuf_Sensor Sensor;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SWO_ENABLED 0  // Set to 1 to enable SWO debugging
#define ITM_PORT 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
OEM_EC_CHIP ec;
OEM_DO_CHIP dOxy;
OEM_PH_CHIP ph;

ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

int Sensors[_jaiabot_sensor_protobuf_Sensor_ARRAYSIZE] = {0};
// Sample rates expressed in milliseconds to match HAL_GetTick output
int SensorSampleRates[_jaiabot_sensor_protobuf_Sensor_ARRAYSIZE] = {0};

#define SOFTWARE_VERSION 3
#define MAX_MSG_SIZE 256
#define SENSOR_REQUEST_SAMPLE_RATE 1000
#define MILLISECONDS_FACTOR 1000

uint8_t uartrxbuff[MAX_MSG_SIZE] __attribute__((aligned(4)));
uint8_t uarttxbuff[MAX_MSG_SIZE] __attribute__((aligned(4)));

extern uint32_t _s_ramfunc, _e_ramfunc, _s_ramfunc_load;

// ADC Variables
uint16_t adc_value1; // Fluorometer
uint16_t adc_value2; //
uint16_t adc_value3; //
uint16_t adc_value4; // pH temperature  
uint16_t adc_value5; // DO temperature

float adc_voltage1;
float adc_voltage2;
float adc_voltage3;
float adc_voltage4;
float adc_voltage5;

uint32_t adc_counter;
uint16_t adc_buffer[5];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

void jumpToBootloader(void);
void startCalibration(jaiabot_sensor_protobuf_Sensor sensor);
void stopCalibration(void);

// Initialize Sensors
void init_blue_robotics_bar30();
void init_atlas_scientific_EC();
void init_atlas_scientific_DO();
void init_atlas_scientific_pH();
void init_CFluor();

// Transmit Data
void process_sensor_request(SensorRequest *sensor_request);
void transmit_sensor_data(SensorData *sensor_data);
void transmit_metadata();
void transmit_atlas_scientific_ec_data();
void transmit_atlas_scientific_do_data();
void transmit_atlas_scientific_ph_data();
void transmit_blue_robotics_bar30_data();
void transmit_turner_c_fluor_data();

// Utility
int hz_to_ms(int hz);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  MX_IWDG_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  // Initialize Sensors
  init_atlas_scientific_EC();
  init_atlas_scientific_DO();
  init_atlas_scientific_pH();
  init_blue_robotics_bar30();
  
  // Hardcoded offset and cal coefficient for Turner CFluor 
  init_CFluor();
  set_CFluorOffset(0.0997);
  set_CFluorCalCoefficient(44.608f);

  // Must be called before computing CRC32
  init_crc32_table();

  // Set up UART RX interrupt
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)uartrxbuff, sizeof(uartrxbuff));

  // Calibrate the ADC
  if (HAL_ADCEx_Calibration_Start(&hadc1, LL_ADC_SINGLE_ENDED) != HAL_OK)
  {
      Error_Handler();
  }

  // Start the timer for ADC Transfers at 100ms
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  double time = 0;
  double ec_target_send_time = 0;
  double do_target_send_time = 0;
  double ph_target_send_time = 0;
  double bar30_target_send_time = 0;
  double turner_c_fluor_target_send_time = 0;
  double sensor_request_target_check_time = 0;

  while (1)
  {
    // Refresh watchdog
    HAL_IWDG_Refresh(&hiwdg);

    // Loop Frequency: 100 Hz
    HAL_Delay(10);

    // Sensor Request
    if (time >= sensor_request_target_check_time)
    {
      sensor_request_target_check_time = time + SENSOR_REQUEST_SAMPLE_RATE;
      SensorRequest sensor_request = process_cmd();
      process_sensor_request(&sensor_request);
    }

    // Sensor Data
    if (Sensors[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_EC] == REQUESTED && time >= ec_target_send_time)
    {
      ec_target_send_time = time + SensorSampleRates[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_EC];
      transmit_atlas_scientific_ec_data();
    }

    if (Sensors[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_DO] == REQUESTED && time >= do_target_send_time)
    {
      do_target_send_time = time + SensorSampleRates[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_DO];
      transmit_atlas_scientific_do_data();
    }

    if (Sensors[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_PH] == REQUESTED && time >= ph_target_send_time)
    {
      ph_target_send_time = time + SensorSampleRates[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_PH];
      transmit_atlas_scientific_ph_data();
    }

    if (Sensors[jaiabot_sensor_protobuf_Sensor_BLUE_ROBOTICS__BAR30] == REQUESTED && time >= bar30_target_send_time)
    {
      bar30_target_send_time = time + SensorSampleRates[jaiabot_sensor_protobuf_Sensor_BLUE_ROBOTICS__BAR30];
      transmit_blue_robotics_bar30_data();
    }

    if (Sensors[jaiabot_sensor_protobuf_Sensor_TURNER__C_FLUOR] == REQUESTED && time >= turner_c_fluor_target_send_time)
    {
      turner_c_fluor_target_send_time = time + SensorSampleRates[jaiabot_sensor_protobuf_Sensor_TURNER__C_FLUOR];
      transmit_turner_c_fluor_data();
    }

    time = HAL_GetTick();
  }

  return 0;
}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

// Sensor Initialization Functions
void init_atlas_scientific_EC()
{
  ec.i2cHandle = &hi2c2;
  int res = initAtlasScientificEC();

  if (res == 0)
  {
    Sensors[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_EC] = INITIALIZED;
  }
  else
  {
	  Sensors[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_EC] = FAILED;
  }
}

void init_atlas_scientific_DO()
{
  dOxy.i2cHandle = &hi2c2;
  int res = initAtlasScientificDO();

  if (res == 0)
  {
    Sensors[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_DO] = INITIALIZED;
  }
  else
  {
	  Sensors[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_DO] = FAILED;
  }
}

void init_atlas_scientific_pH()
{
  ph.i2cHandle = &hi2c2;
  int res = initAtlasScientificPH();

  if (res == 0)
  {
    Sensors[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_PH] = INITIALIZED;
  }
  else
  {
	Sensors[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_PH] = FAILED;
  }
}

void init_blue_robotics_bar30()
{
  int res = initMS5837(&hi2c3, MS5837_30BA);

  if (res == 0)
  {
    // Forward LED
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_10);
    Sensors[jaiabot_sensor_protobuf_Sensor_BLUE_ROBOTICS__BAR30] = INITIALIZED;
  }
  else
  {
	Sensors[jaiabot_sensor_protobuf_Sensor_BLUE_ROBOTICS__BAR30] = FAILED;
  }
}

void init_CFluor()
{
  Sensors[jaiabot_sensor_protobuf_Sensor_TURNER__C_FLUOR] = INITIALIZED;
}

void process_sensor_request(SensorRequest *sensor_request)
{
  if (sensor_request->which_request_data == jaiabot_sensor_protobuf_SensorRequest_request_metadata_tag)
  {
    if (sensor_request->request_data.request_metadata)
    {
      transmit_metadata();
    }
  }
  else if (sensor_request->which_request_data == jaiabot_sensor_protobuf_SensorRequest_cfg_tag)
  {
    if (sensor_request->request_data.cfg.sensor == jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_EC && Sensors[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_EC] != STOPPED)
    {
      SensorSampleRates[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_EC] = hz_to_ms(sensor_request->request_data.cfg.sample_freq);
      Sensors[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_EC] = REQUESTED;
    }

    if (sensor_request->request_data.cfg.sensor == jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_DO && Sensors[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_DO] != STOPPED)
    {
      SensorSampleRates[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_DO] = hz_to_ms(sensor_request->request_data.cfg.sample_freq);
      Sensors[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_DO] = REQUESTED;
    }

    if (sensor_request->request_data.cfg.sensor == jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_PH && Sensors[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_PH] != STOPPED)
    {
      SensorSampleRates[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_PH] = hz_to_ms(sensor_request->request_data.cfg.sample_freq);
      Sensors[jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_PH] = REQUESTED;
    }

    if (sensor_request->request_data.cfg.sensor == jaiabot_sensor_protobuf_Sensor_BLUE_ROBOTICS__BAR30 && Sensors[jaiabot_sensor_protobuf_Sensor_BLUE_ROBOTICS__BAR30] != STOPPED)
    {
      SensorSampleRates[jaiabot_sensor_protobuf_Sensor_BLUE_ROBOTICS__BAR30] = hz_to_ms(sensor_request->request_data.cfg.sample_freq);
      Sensors[jaiabot_sensor_protobuf_Sensor_BLUE_ROBOTICS__BAR30] = REQUESTED;
    }

    if (sensor_request->request_data.cfg.sensor == jaiabot_sensor_protobuf_Sensor_TURNER__C_FLUOR && Sensors[jaiabot_sensor_protobuf_Sensor_TURNER__C_FLUOR] != STOPPED)
    {
      SensorSampleRates[jaiabot_sensor_protobuf_Sensor_TURNER__C_FLUOR] = hz_to_ms(sensor_request->request_data.cfg.sample_freq);
      Sensors[jaiabot_sensor_protobuf_Sensor_TURNER__C_FLUOR] = REQUESTED;
    }
  }

  if (sensor_request->has_mcu_command && sensor_request->mcu_command == jaiabot_sensor_protobuf_MCUCommand_ENTER_BOOTLOADER_MODE)
  {
    jumpToBootloader();
  }

  if (sensor_request->has_calibration_type)
  {
    switch (sensor_request->calibration_type)
    {
      case jaiabot_sensor_protobuf_CalibrationType_START_EC_CALIBRATION:
        startCalibration(jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_EC);
        break;
      case jaiabot_sensor_protobuf_CalibrationType_CALIBRATE_EC_DRY:
        calibrateEC(0.000000, 2);
        break;
      case jaiabot_sensor_protobuf_CalibrationType_CALIBRATE_EC_LOW:
        calibrateEC(sensor_request->calibration_value * 100.0, 4);
        break;
      case jaiabot_sensor_protobuf_CalibrationType_CALIBRATE_EC_HIGH:
        calibrateEC(sensor_request->calibration_value * 100.0, 5);
        break;
      case jaiabot_sensor_protobuf_CalibrationType_CLEAR_EC_CALIBRATION:
        clearCalibration(jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_EC);
        break;
      case jaiabot_sensor_protobuf_CalibrationType_START_DO_CALIBRATION:
        startCalibration(jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_DO);
        break;
      case jaiabot_sensor_protobuf_CalibrationType_CALIBRATE_DO_LOW:
        calibrateDO(3);
        break;
      case jaiabot_sensor_protobuf_CalibrationType_CALIBRATE_DO_HIGH:
        calibrateDO(2);
        break;
      case jaiabot_sensor_protobuf_CalibrationType_CLEAR_DO_CALIBRATION:
        clearCalibration(jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_DO);
        break;
      case jaiabot_sensor_protobuf_CalibrationType_START_PH_CALIBRATION:
        startCalibration(jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_PH);
        break;
      case jaiabot_sensor_protobuf_CalibrationType_CALIBRATE_PH_LOW:
        calibratePH(sensor_request->calibration_value * 1000.0, 2);
        break;
      case jaiabot_sensor_protobuf_CalibrationType_CALIBRATE_PH_MID:
        calibratePH(sensor_request->calibration_value * 1000.0, 3);
        break;
      case jaiabot_sensor_protobuf_CalibrationType_CALIBRATE_PH_HIGH:
        calibratePH(sensor_request->calibration_value * 1000.0, 4);
        break;
      case jaiabot_sensor_protobuf_CalibrationType_CLEAR_PH_CALIBRATION:
        clearCalibration(jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_PH);
        break;
      case jaiabot_sensor_protobuf_CalibrationType_STOP_CALIBRATION:
        stopCalibration();
        break;
      default:
        break;
    }

    transmit_metadata();
  }

  if (sensor_request->has_compensation_type)
  {
    switch (sensor_request->compensation_type)
    {
      case jaiabot_sensor_protobuf_CompensationType_SET_EC_TEMPERATURE_COMPENSATION:
        setECTempCompensation(sensor_request->compensation_value * 100.0);
        break;
      case jaiabot_sensor_protobuf_CompensationType_SET_DO_SALINITY_COMPENSATION:
        setDOSalinityCompensation(sensor_request->compensation_value * 100.0);
        break;
      case jaiabot_sensor_protobuf_CompensationType_SET_DO_PRESSURE_COMPENSATION:
        setDOPressureCompensation(sensor_request->compensation_value * 100.0);
        break;
      case jaiabot_sensor_protobuf_CompensationType_SET_DO_TEMPERATURE_COMPENSATION:
        setDOTempCompensation(sensor_request->compensation_value * 100.0);
        break;
      case jaiabot_sensor_protobuf_CompensationType_SET_PH_TEMPERATURE_COMPENSATION:
        setPHTempCompensation(sensor_request->compensation_value * 100.0);
        break;
      default:
        break;
    }
  }
}

void startCalibration(jaiabot_sensor_protobuf_Sensor sensor)
{
  for (int i = 0; i < _jaiabot_sensor_protobuf_Sensor_ARRAYSIZE; i++)
  {
    // Set all of our sensors to UNITIALIZED besides the one we're calibrating. Set the sample rate to 1 Hz for the sensor we're calibrating
    if (i != sensor)
    {
      Sensors[i] = STOPPED;
    }
    else 
    {
      SensorSampleRates[i] = hz_to_ms(1);
      Sensors[i] = REQUESTED;
    }
    printf("SENSOR: %d\r\n", i, Sensors[i]);
  }
}

void stopCalibration()
{
  // Set all of our sensors to REQUESTED. Set the sample rate to 10 Hz for all sensors
  for (int i = 0; i < _jaiabot_sensor_protobuf_Sensor_ARRAYSIZE; i++)
  {
    SensorSampleRates[i] = hz_to_ms(10);
    Sensors[i] = REQUESTED;
  }
}

void transmit_sensor_data(SensorData *sensor_data)
{
  uint8_t buffer[MAX_MSG_SIZE] = {0};
  uint8_t buffer_cobs[MAX_MSG_SIZE] = {0};
  size_t message_length;

  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  bool status = pb_encode(&stream, jaiabot_sensor_protobuf_SensorData_fields, sensor_data);

  if (!status)
  {
    return;
  }

  message_length = stream.bytes_written;

  uint32_t calculated_crc = compute_crc32(buffer, message_length);
  uint8_t bits_in_byte = 8;
  uint8_t bytes_in_crc32 = 4;

  uint8_t counter = 0;
  for (int i = bytes_in_crc32 - 1; i >= 0; --i)
  {
    buffer[counter + message_length] = (calculated_crc >> (i * bits_in_byte)) & 0xFF;
    counter++;
  }

  COBSStuffData(buffer, message_length + bytes_in_crc32, buffer_cobs);

  uint8_t len_cobs = {0};
  for (int i = 0; i < sizeof(buffer_cobs); i++)
  {
    len_cobs = len_cobs + 1;
    if (buffer_cobs[i] == 0)
    {
      break;
    }
  }

  HAL_StatusTypeDef transmit_status = HAL_UART_Transmit(&huart2, buffer_cobs, len_cobs, HAL_MAX_DELAY);

  // Middle LED
  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_11);
  HAL_Delay(10);
}

void transmit_metadata()
{
  for (int sensor_index = 1; sensor_index < _jaiabot_sensor_protobuf_Sensor_ARRAYSIZE; sensor_index++)
  {

    Metadata metadata = jaiabot_sensor_protobuf_Metadata_init_zero;
    metadata.sensor = sensor_index;
    metadata.has_payload_board_version = true;
    metadata.payload_board_version = SOFTWARE_VERSION;
    
    metadata.has_calibration = true;
    
    // Sensor calibration information
    switch (sensor_index)
    {
      case jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_EC:
        metadata.calibration.has_confirmation = true;
        metadata.calibration.confirmation = ec.calibration_confirmation;
        break;
      case jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_DO:
        metadata.calibration.has_confirmation = true;
        metadata.calibration.confirmation = dOxy.calibration_confirmation;
        break;
      case jaiabot_sensor_protobuf_Sensor_ATLAS_SCIENTIFIC__OEM_PH:
        metadata.calibration.has_confirmation = true;
        metadata.calibration.confirmation = ph.calibration_confirmation;
        break;
      default:
        break;
    }

    if (Sensors[sensor_index] == UNINITIALIZED)
    {
      continue;
    }

    if (Sensors[sensor_index] == FAILED)
    {
    	metadata.has_init_failed = true;
    	metadata.init_failed = true;
    }

    SensorData sensor_data = jaiabot_sensor_protobuf_SensorData_init_zero;
    sensor_data.time = HAL_GetTick();
    sensor_data.which_data = jaiabot_sensor_protobuf_SensorData_metadata_tag;
    sensor_data.data.metadata = metadata;

    transmit_sensor_data(&sensor_data);
  }
}

// Data Transmit Functions
void transmit_atlas_scientific_ec_data()
{
  SensorData sensor_data = jaiabot_sensor_protobuf_SensorData_init_zero;
  sensor_data.time = HAL_GetTick();
  sensor_data.which_data = jaiabot_sensor_protobuf_SensorData_oem_ec_tag;
  AtlasScientificOEMEC oem_ec = jaiabot_sensor_protobuf_AtlasScientificOEMEC_init_zero;

  if (get_ECReading() == HAL_OK)
  {
    oem_ec.has_conductivity = true;
    oem_ec.conductivity = getConductivity();
    oem_ec.has_total_dissolved_solids = true;
    oem_ec.total_dissolved_solids = getTDS();
    oem_ec.has_salinity = true;
    oem_ec.salinity = getSalinity();
  }

  sensor_data.data.oem_ec = oem_ec;
  transmit_sensor_data(&sensor_data);
}

void transmit_atlas_scientific_do_data()
{
  SensorData sensor_data = jaiabot_sensor_protobuf_SensorData_init_zero;
  sensor_data.time = HAL_GetTick();
  sensor_data.which_data = jaiabot_sensor_protobuf_SensorData_oem_do_tag;
  AtlasScientificOEMDO oem_do = jaiabot_sensor_protobuf_AtlasScientificOEMDO_init_zero;

  if (get_DOReading() == HAL_OK)
  {
    oem_do.has_dissolved_oxygen = true;
    oem_do.dissolved_oxygen = getDO();
    oem_do.has_temperature = true;
    oem_do.temperature = getDOTemperature();
    oem_do.has_temperature_voltage = true;
    oem_do.temperature_voltage = getDOTemperatureVoltage();
  }

  sensor_data.data.oem_do = oem_do;
  transmit_sensor_data(&sensor_data);
}

void transmit_atlas_scientific_ph_data()
{
  SensorData sensor_data = jaiabot_sensor_protobuf_SensorData_init_zero;
  sensor_data.time = HAL_GetTick();
  sensor_data.which_data = jaiabot_sensor_protobuf_SensorData_oem_ph_tag;
  AtlasScientificOEMPH oem_ph = jaiabot_sensor_protobuf_AtlasScientificOEMpH_init_zero;

  if (get_PHReading() == HAL_OK)
  {
    oem_ph.has_ph = true;
    oem_ph.ph = getPH();
    oem_ph.has_temperature = true;
    oem_ph.temperature = getPHTemperature();
    oem_ph.has_temperature_voltage = true;
    oem_ph.temperature_voltage = getPHTemperatureVoltage();
  }

  sensor_data.data.oem_ph = oem_ph;
  transmit_sensor_data(&sensor_data);
}

void transmit_blue_robotics_bar30_data()
{
  SensorData sensor_data = jaiabot_sensor_protobuf_SensorData_init_zero;
  sensor_data.time = HAL_GetTick();
  sensor_data.which_data = jaiabot_sensor_protobuf_SensorData_bar30_tag;
  BlueRoboticsBar30 bar30 = jaiabot_sensor_protobuf_BlueRoboticsBar30_init_zero;

  if (readMS5837() == 0)
  {
    bar30.has_pressure = true;
    bar30.pressure = getDepth();
    bar30.has_temperature = true;
    bar30.temperature = getTemperature();
  }

  sensor_data.data.bar30 = bar30;
  transmit_sensor_data(&sensor_data);
}

void transmit_turner_c_fluor_data()
{
  SensorData sensor_data = jaiabot_sensor_protobuf_SensorData_init_zero;
  sensor_data.time = HAL_GetTick();
  sensor_data.which_data = jaiabot_sensor_protobuf_SensorData_c_fluor_tag;
  TurnerCFluor c_fluor = jaiabot_sensor_protobuf_TurnerCFluor_init_zero;

  if (readCFluor() == 0)
  {
    c_fluor.has_concentration = true;
    c_fluor.concentration = getConcentration();
    c_fluor.has_concentration_voltage = true;
    c_fluor.concentration_voltage = getConcentrationVoltage();
  }

  sensor_data.data.c_fluor = c_fluor;
  transmit_sensor_data(&sensor_data);
}

int hz_to_ms(int hz)
{
  return 1.0f / hz * MILLISECONDS_FACTOR;
}
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  // Aft LED
  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_12);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10D19CE4;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 159;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim16, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PDIS_PH_EN_GPIO_Port, PDIS_PH_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PDIS_DO_EN_Pin|PDIS_EC_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PDIS_PH_EN_Pin */
  GPIO_InitStruct.Pin = PDIS_PH_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PDIS_PH_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OC2_Pin */
  GPIO_InitStruct.Pin = OC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(OC2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SC1_Pin OC1_Pin WC_EN_Pin */
  GPIO_InitStruct.Pin = SC1_Pin|OC1_Pin|WC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PDIS_DO_EN_Pin PDIS_EC_EN_Pin */
  GPIO_InitStruct.Pin = PDIS_DO_EN_Pin|PDIS_EC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RS232_INV_Pin */
  GPIO_InitStruct.Pin = RS232_INV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RS232_INV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SC2_Pin */
  GPIO_InitStruct.Pin = SC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SC2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PPS_Pin */
  GPIO_InitStruct.Pin = PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PPS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *data, int len) {
#if SWO_ENABLED
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar(data[i]); // Send each character over SWO
    }
    return len;
#else
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t *)data, len, HAL_MAX_DELAY);
    return (status == HAL_OK) ? len : -1;
#endif
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  // NOTE: This gets called on HT and FT by default
  if (Size > 1)
  {
    uartrxbuff[Size] = '\0';

    // All '$' messages are added to queue to be processed
    // Add message to the queue if there's enough room
    if (uQueue.msgCount < UART_QUEUE_SIZE)
    {
      uQueue.msgCount++;

      if (uQueue.wIndex > UART_QUEUE_SIZE - 1)
      {
        uQueue.wIndex = 0;
      }

      // Copy Message into message queue!
      strcpy(uQueue.msgQueue[uQueue.wIndex], uartrxbuff);

      uQueue.wIndex++;
    }
    else
    {
      // Error UART queue full!
      printf("UART Queue full!\r\n");
    }
  }

  // Set up next DMA Reception!
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)uartrxbuff, sizeof(uartrxbuff));
  //__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_value1 = adc_buffer[0];
        adc_value2 = adc_buffer[1];
        adc_value3 = adc_buffer[2];
        adc_value4 = adc_buffer[3];
        adc_value5 = adc_buffer[4];

        adc_voltage1 = adc_buffer[0] * 3.3f / 4096.0f;
        adc_voltage2 = adc_buffer[1] * 3.3f / 4096.0f;
        adc_voltage3 = adc_buffer[2] * 3.3f / 4096.0f;
        adc_voltage4 = adc_buffer[3] * 3.3f / 4096.0f;
        adc_voltage5 = adc_buffer[4] * 3.3f / 4096.0f;

        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,0);

        adc_counter++;
    }
}

void jumpToBootloader(void)
{

  HAL_FLASH_Unlock();

  // Flash Erase Configuration
  FLASH_EraseInitTypeDef eraseInitStruct = {0};
  uint32_t pageError = 0;

  eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES; // Page erase
  eraseInitStruct.Banks = FLASH_BANK_1;              // Specify Bank 1
  eraseInitStruct.Page = 0;                          // Page number to erase (0 = first page)
  eraseInitStruct.NbPages = 1;                       // Number of pages to erase

  // Perform the erase operation
  if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK)
  {
    // Handle error
    uint32_t errorCode = HAL_FLASH_GetError();
    while (1)
      ;
  }

  uint32_t address = 0x08000000;
  uint64_t data_to_write = 0xFFFFFFFFFFFFFFFF;

  // Program the flash (64-bit aligned)
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data_to_write) != HAL_OK)
  {
    uint32_t errorCode = HAL_FLASH_GetError();
    while (1)
      ;
  }

  HAL_FLASH_Lock(); // Lock the flash to prevent accidental writes

  /* Disable all interrupts */
  __disable_irq();

  /* Disable Systick timer */
  SysTick->CTRL = 0;

  /* Set the clock to the default state */
  HAL_RCC_DeInit();

  /* Clear Interrupt Enable Register & Interrupt Pending Register */
  for (uint8_t i = 0; i < (MCU_IRQS + 31u) / 32; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  /* Re-enable all interrupts */
  __enable_irq();

  // Set the MSP
  __set_MSP(BOOTVTAB->Initial_SP);

  // Jump to app firmware
  BOOTVTAB->Reset_Handler();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
