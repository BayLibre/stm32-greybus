# stm32-greybus

STM32F4Discovery Board Greybus Experiments

The greybus protocol can be used to control an external microcontrol using a simple and well-designed protocol.

The protocol already defines I2C, GPIO, LED or HID transports, this project aims to implement the Greybus protocol based on the gbsim project using the physical interfaces of the STM32.

Currently, the project build with the gcc-arm-none-eabi-5_2-2015q4 toolchain.

Simply run :

$ make PREFIX=/path/to/gcc-arm-none-eabi-5_2-2015q4/bin/arm-none-eabi-
