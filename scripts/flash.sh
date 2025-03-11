#!/bin/bash

## Currently this script will only run from F0B2 stm32-deploy dir

# You must stop the jaiabot services to prevent the arduino 
# from binding to /dev/ttyUSB0

## You need to move the elf file into the directory that you will be executing this script

# Exit immediately if a command exits with non-zero status
set -e

# Configure device connected to BIO Payload Board
stty -F /dev/ttyUSB0 115200

# Send character to enter bootloader mode
echo "Sending bootloader command"
printf "\x24boot" > /dev/ttyUSB0
sleep 2

# Copy this file from the STM32CubeIDE
ELF="JAIA_BIO-PAYLOAD.elf"
# Name of file to be created
BIN="JAIA_BIO-PAYLOAD.bin"

echo "Converting ELF to BIN..."
rm "$BIN"
arm-none-eabi-objcopy -O binary "$ELF" "$BIN"

echo "Resetting STM32..."
echo '0' | sudo tee /sys/bus/usb/devices/usb1/authorized
sleep 1
echo '1' | sudo tee /sys/bus/usb/devices/usb1/authorized
sleep 1

stm32flash-0.7/stm32flash -w JAIA_BIO-PAYLOAD.bin -v -b "115200" -g "0x08000000" "/dev/ttyUSB0"

echo "Resetting STM32..."
echo '0' | sudo tee /sys/bus/usb/devices/usb1/authorized
sleep 1
echo '1' | sudo tee /sys/bus/usb/devices/usb1/authorized
sleep 1
