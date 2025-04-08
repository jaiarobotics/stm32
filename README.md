# stm32

## UART Pin Swap
- USART2 Default pins PA2 and PA3 are required for bootloader
- PA3 was assigned to LPUART1_RX and PA15 was assigned to USART2_RX
- PA3 was jumped to USART2_RX via add wire from J7-Pin4 to U6-Pin7
- Trace was cut next to U6-Pin7

## ADC Updates
- Verified ADC1 Channels 2/3 via function generator 0-5V triangle wave
- Set up 10Hz triggered 5-CH scan for ADC1
- Fixed .ioc pinout error. Differential inputs assigned wrong on J13. Work as single ended only
- Initialized IWDG + Added IWDG reset to main
- Added printf support for SWV
- Set up Debug config for SWV Real time plotting
- Added systick flag for Depth sampling in main() loop
 
## IWDG
- LSI = 32kHz
- Timeout = (Reload + 1) x Prescaler / LSI_FREQ
- 2seconds ~ PRESCALER_256 and Reload = 2500
- Make sure to enable IWDG while debugging in Debug Config

## ADC
- Scan Conversion mode enabled
- DMA Continous Requests enabled
- End of Conversion set to End of Sequence
- 5 Conversion
- Trigger set to TIM6
- Set each Rank to channels in incrementing order
- Set each Channel to 247.5 sample time
- Both IRQs enabled
- DMA1 Channel 1 Assigned
- DMA set to circular mode

## TIMER
- TIM6 used for ADC conversion trigger
- Trigger event = Update Event
- Prescaler 159, Counter Period = 49999 ==> 100msec

## SWV Config
- Debug Configurations >> JAIA_BIO-PAYLOAD-ADC Debug >> Debugger
- Enable SWV
- Set clock to be equal to micro clock speed 80MHz (See clock config page) 
- Set "Suspend watchdog counters" to enable

## SWV Plotting
- Enable SWV Live Data Plotting
- Open Window > Show View > SWV > SWV Data Trace Timeline
- Click Configure Trace
- Enable PC Sample and Data Trace
- Add the global variable you want to monitor
- Click Start Trace