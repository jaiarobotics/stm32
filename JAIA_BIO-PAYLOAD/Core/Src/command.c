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

void process_cmd(void)
{
  if (uQueue.msgCount > 0)
  {
    // First calculate which message we need to process from the queue (0 - 16). wIndex - msgCount
    uQueue.rIndex = uQueue.wIndex - uQueue.msgCount;
    uQueue.rIndex = uQueue.rIndex < 0 ? uQueue.rIndex - UART_QUEUE_SIZE : uQueue.rIndex;

    uQueue.msgCount--;

    printf("Processing Command! : %s\r\n",uQueue.msgQueue[uQueue.rIndex]);
    
    // Toggle LEDs if the STM32 receives a command message
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_10);
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_11);
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_12);

    strcpy(msg,uQueue.msgQueue[uQueue.rIndex]);

  }
}
