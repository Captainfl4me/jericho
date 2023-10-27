# Jericho

Project of active guided model rocket.

## Onboard computer

This flight computer is a RP2040 program with [pico C/C++ sdk](https://github.com/raspberrypi/pico-sdk). This computer use an IMU, a magnetometer, a barometer and a SD card reader to get relative altitude and attitude of the rocket.

Library use:

- [no-OS-FatFS-SD-SPI-RPi-Pico](https://github.com/carlk3/no-OS-FatFS-SD-SPI-RPi-Pico)
- [Simple WS2812 LED](https://github.com/ForsakenNGS/Pico_WS2812)

## Debug LED

The built-in adressable LED in the waveshare RP2040 zero board is use to check status of computer.

LED | status
--|--
RED | Start program
RED blink | Running initialization
GREEN blink | Running start-up tests
GREEN 1s | All tests are OK
PURPLE | At least one test failed!
YELLOW | Main loop running
BLUE | Main loop exit, mission finish successfully
