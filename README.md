# stm32

## UART Pin Swap
- USART2 Default pins PA2 and PA3 are required for bootloader
- PA3 was assigned to LPUART1_RX and PA15 was assigned to USART2_RX
- PA3 was jumped to USART2_RX via add wire from J7-Pin4 to U6-Pin7
- Trace was cut next to U6-Pin7