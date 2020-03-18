#!/bin/sh

# Enable Pin 32 / PWM0
echo "Setup Pin 32 as PWM"
busybox devmem 0x700031fc 32 0x45
busybox devmem 0x6000d504 32 0x2

# Enable Pin 33 / PWM2
echo "Setup Pin 33 as PWM"
busybox devmem 0x70003248 32 0x46
busybox devmem 0x6000d100 32 0x00
