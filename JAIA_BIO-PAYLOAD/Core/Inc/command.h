/*
 * command.h
 *
 *  Created on: Mar 5, 2025
 *      Author: ColinVincent
 */

#ifndef INC_COMMAND_H_
#define INC_COMMAND_H_

#include "stdint.h"
#include "crc32.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"

#define UART_QUEUE_SIZE 32
#define UART_MAX_LEN 256

typedef struct tUartQueue
{
  uint8_t msgQueue[UART_QUEUE_SIZE][UART_MAX_LEN];        // {msg1,msg2,msg3...,msg128} length * width
  uint16_t msgCount;
  uint8_t wIndex;
  int8_t rIndex;
} UART_QUEUE;

extern UART_QUEUE uQueue;

void process_cmd(void);

#endif /* INC_COMMAND_H_ */
