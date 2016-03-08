# stm32-greybus

STM32F4Discovery Board Greybus Experiments

The greybus protocol can be used to control an external microcontroller using a simple and well-designed protocol, see : https://github.com/projectara/greybus for Linux kernel host over USB implementation

The protocol already defines I2C, GPIO, LED or HID transports, this project aims to implement the Greybus protocol based on the gbsim project using the physical interfaces of the STM32.
The GBSIM original sources can be found at : https://github.com/projectara/gbsim

Currently, the project build with the gcc-arm-none-eabi-5_2-2015q4 toolchain :
https://launchpad.net/gcc-arm-embedded/5.0/5-2015-q4-major/+download/gcc-arm-none-eabi-5_2-2015q4-20151219-linux.tar.bz2

Simply run :
```
$ make PREFIX=/path/to/gcc-arm-none-eabi-5_2-2015q4/bin/arm-none-eabi-
```
