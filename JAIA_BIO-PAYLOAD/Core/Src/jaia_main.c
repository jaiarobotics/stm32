#include "jaia_main.h"

OEM_CHIP ec;
OEM_CHIP dOxy;
OEM_CHIP ph;

//uint8_t tx_buff[];
//uint8_t rx_buff[10];

/*ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;*/

/*void jumpToBootloader(void)
{
  // Set RAM to our flag (random number) and reset the MCU
  #define BOOT_ADDR 0x1FFF0000  // my MCU boot code base address
  #define MCU_IRQS  70u // no. of NVIC IRQ inputs

  #define BOOTVTAB  ((struct boot_vectable_ *)BOOT_ADDR)

  HAL_FLASH_Unlock();

  // Flash Erase Configuration
  FLASH_EraseInitTypeDef eraseInitStruct = {0};
  uint32_t pageError = 0;

  eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES; // Page erase
  eraseInitStruct.Banks = FLASH_BANK_1;             // Specify Bank 1
  eraseInitStruct.Page = 0;                         // Page number to erase (0 = first page)
  eraseInitStruct.NbPages = 1;                      // Number of pages to erase

  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_10);

  // Perform the erase operation
  if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK)
  {
      // Handle error
      uint32_t errorCode = HAL_FLASH_GetError();
      while (1); // Debug: Loop indefinitely on error
  }

  uint32_t address = 0x08000000; 
  uint64_t data_to_write = 0xFFFFFFFFFFFFFFFF;

  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_11);

  // Program the flash (64-bit aligned)
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data_to_write) != HAL_OK)
  {
      uint32_t errorCode = HAL_FLASH_GetError();
      while (1);
  }

  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_12);

  HAL_FLASH_Lock(); // Lock the flash to prevent accidental writes

  // Disable all interrupts
  __disable_irq();

  //Disable Systick timer
  SysTick->CTRL = 0;

  // Set the clock to the default state
  HAL_RCC_DeInit();

  // Clear Interrupt Enable Register & Interrupt Pending Register 
  for (uint8_t i = 0; i < (MCU_IRQS + 31u) / 32; i++)
  {
    NVIC->ICER[i]=0xFFFFFFFF;
    NVIC->ICPR[i]=0xFFFFFFFF;
  }

  // Re-enable all interrupts
  __enable_irq();

  // Set the MSP
  __set_MSP(BOOTVTAB->Initial_SP);

  // Jump to app firmware
  BOOTVTAB->Reset_Handler();
}*/


// Loop through memory addresses to find addresses with I2C devices on them
/*void I2C_Scan(void) {
  uint8_t Buffer[25] = {0};
  uint8_t Space[] = " - ";
  uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
  uint8_t EndMSG[] = "\r\n\r\n Done! \r\n\r\n";

  uint8_t i = 0, ret;

  HAL_UART_Transmit(&huart2, StartMSG, sizeof(StartMSG), HAL_MAX_DELAY);
  for(i=1; i<128; i++)
  {
      ret = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1), 3, 5);
      if (ret != HAL_OK) // No ACK Received At That Address
      {
        HAL_UART_Transmit(&huart2, Space, sizeof(Space), HAL_MAX_DELAY);
      } else if (ret == HAL_OK) {
        sprintf(Buffer, "0x%X", i);
        HAL_UART_Transmit(&huart2, Buffer, sizeof(Buffer), HAL_MAX_DELAY);
      }
  }
  HAL_UART_Transmit(&huart2, EndMSG, sizeof(EndMSG), HAL_MAX_DELAY);
  // [ Scanning Done ]

  return;
}*/


/*void startAtlasChips(void) {
  
  // Turn on Atlas Sensors
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); // pH
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET); // DO
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET); // EC  

  // Look for powered-up I2C devices on i2c bus 2
  I2C_Scan();

  // Assign the I2C address of each Atlas Scientific chip to its respective object
  ec.devAddr = EC_OEM_I2C_ADDR;
  ph.devAddr = PH_OEM_I2C_ADDR;
  dOxy.devAddr = DO_OEM_I2C_ADDR;

  // Activate our Atlas Scientific chips
  HAL_StatusTypeDef ec_init_status = OEM_Init(&ec, &hi2c2);
  HAL_StatusTypeDef do_init_status = OEM_Init(&dOxy, &hi2c2);
  HAL_StatusTypeDef ph_init_status = OEM_Init(&ph, &hi2c2);

  sprintf(tx_buff, "EC Init Status: 0x%02X\r\nDO Init Status: 0x%02X\r\npH Init Status: 0x%02X\r\n\r\n", ec_init_status, ph_init_status);
  HAL_UART_Transmit(&huart2, tx_buff, strlen(tx_buff), HAL_MAX_DELAY);

}*/

/*int jaia_main() {

    // Initialize and Activate the Atlas Scientific chips
    startAtlasChips();

    while (1) {
        HAL_Delay(1000); // Delay for 1 second
        
        // LEDs 
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_10);
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_11);
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_12);
        
        // Read the Atlas Scientific chips
        HAL_StatusTypeDef ecReadStatus = OEM_ReadData(&ec); 
        HAL_StatusTypeDef doReadStatus = OEM_ReadData(&dOxy);
        HAL_StatusTypeDef phReadStatus = OEM_ReadData(&ph);

        uint8_t buffer[128];
        size_t message_length;
        bool status;

        if (rx_buff[0] == 1) {
          HAL_UART_Transmit(&huart2, "Found it\r\n", sizeof("Found it\r\n"), HAL_MAX_DELAY);
          break;
        }
        
      
        // Encode sensor data into nanopb message
        {
            SensorData message = SensorData_init_zero;
            AtlasScientificOEMEC ec_message = AtlasScientificOEMEC_init_zero;
            AtlasScientificOEMDO do_message = AtlasScientificOEMDO_init_zero;
            AtlasScientificOEMpH ph_message = AtlasScientificOEMpH_init_zero;

            pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

            message.data.oem_ec.conductivity = ec.reading;
            message.data.oem_do.dissolved_oxygen = dOxy.reading;
            message.data.oem_ph.ph = ph.reading;

            status = pb_encode(&stream, SensorData_fields, &message);
            if (!status) {
                //HAL_UART_Transmit(&huart2, "Failed to encode message\r\n", sizeof("Failed to encode message\r\n"), HAL_MAX_DELAY);
            } else {
                message_length = stream.bytes_written;
                //HAL_UART_Transmit(&huart2, buffer, message_length, HAL_MAX_DELAY);
            }
        }



    } 
    return 0;
}*/




