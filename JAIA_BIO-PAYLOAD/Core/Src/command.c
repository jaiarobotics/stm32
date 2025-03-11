/*
 * command.c
 *
 *  Created on: Mar 5, 2025
 *      Author: ColinVincent
 */
#include "command.h"

#include "stdio.h"
#include "string.h"
#include "stdbool.h"

#include "main.h"

// Command Processing
UART_QUEUE uQueue;
uint8_t msg[128];

// Commands
char* BOOT = "$boot\n";

void process_cmd(void)
{
  if (uQueue.msgCount > 0)
  {
    // First calculate which message we need to process from the queue (0 - 16). wIndex - msgCount
    uQueue.rIndex = uQueue.wIndex - uQueue.msgCount;
    uQueue.rIndex = uQueue.rIndex < 0 ? uQueue.rIndex - UART_QUEUE_SIZE : uQueue.rIndex;

    uQueue.msgCount--;

    printf("Processing Command! : %s\r\n",uQueue.msgQueue[uQueue.rIndex]);

    strcpy(msg,uQueue.msgQueue[uQueue.rIndex]);

    // Need to revisit comparison
    // Right now $b will match with BOOT
    // Not using == 0 at the moment because ending characters are not known
    if (strcmp(msg, BOOT))
    {
    	printf("%s\r\n", "BOOT");
    	JumpToBootloader();
    }
  }
}

void JumpToBootloader(void)
{

  HAL_FLASH_Unlock();

  // Flash Erase Configuration
  FLASH_EraseInitTypeDef eraseInitStruct = {0};
  uint32_t pageError = 0;

  eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES; // Page erase
  eraseInitStruct.Banks = FLASH_BANK_1;             // Specify Bank 1
  eraseInitStruct.Page = 0;                         // Page number to erase (0 = first page)
  eraseInitStruct.NbPages = 1;                      // Number of pages to erase

  // Perform the erase operation
  if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK)
  {
      // Handle error
      uint32_t errorCode = HAL_FLASH_GetError();
      while (1);
  }

  uint32_t address = 0x08000000;
  uint64_t data_to_write = 0xFFFFFFFFFFFFFFFF;

  // Program the flash (64-bit aligned)
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data_to_write) != HAL_OK)
  {
      uint32_t errorCode = HAL_FLASH_GetError();
      while (1);
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
    NVIC->ICER[i]=0xFFFFFFFF;
    NVIC->ICPR[i]=0xFFFFFFFF;
  }

  /* Re-enable all interrupts */
  __enable_irq();

  // Set the MSP
  __set_MSP(BOOTVTAB->Initial_SP);

  // Jump to app firmware
  BOOTVTAB->Reset_Handler();
}

