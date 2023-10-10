/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"

#include "WS2812.hpp"
#include "MPU6050.hpp"

#define LED_PIN 16
#define LED_LENGTH 1


void core1_main() {
    WS2812 built_in_led(LED_PIN, LED_LENGTH, pio0, 0, WS2812::FORMAT_GRB);
    while (true) {    
        for (int i = 0; i < 255; i++) {
            built_in_led.fill(WS2812::RGB(0, i, 0));
            built_in_led.show();
            sleep_ms(5);
        }
        for (int i = 255; i > 0; i--) {
            built_in_led.fill(WS2812::RGB(0, i, 0));
            built_in_led.show();
            sleep_ms(5);
        }
    } 
}

int main() {
    // Enable UART so we can print status output
    stdio_init_all();

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/bus_scan example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else

    printf("Initialize RP2040");
    WS2812 built_in_led(LED_PIN, LED_LENGTH, pio0, 0, WS2812::FORMAT_GRB);
    built_in_led.fill(WS2812::RGB(255, 0, 0));
    built_in_led.show();

    // Init I2C 0 with 400KHz
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    sleep_ms(2000);

    MPU6050 mpu6050(0x68);
    printf("MPU6050 initialized\n");
    if (mpu6050.testConnection()) {
        printf("MPU6050 connection successful\n");
    } else {
        printf("MPU6050 connection failed\n");
        return 1;
    }

    sleep_ms(2000);
    
    built_in_led.fill(WS2812::RGB(100, 100, 0));
    built_in_led.show();

    multicore_launch_core1(core1_main);
    while(true) {
#ifdef DEBUG
        uint32_t startTime = time_us_32();
#endif

        mpu6050.updateData();

#ifdef DEBUG
        uint32_t executionTime = time_us_32() - startTime;
        //printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%f\n", executionTime, mpu6050.raw_acc[0], mpu6050.raw_acc[1], mpu6050.raw_acc[2], mpu6050.raw_gyro[0], mpu6050.raw_gyro[1], mpu6050.raw_gyro[2], mpu6050.temp);
        printf("%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", executionTime, mpu6050.acc[0], mpu6050.acc[1], mpu6050.acc[2], mpu6050.gyro[0], mpu6050.gyro[1], mpu6050.gyro[2], mpu6050.temp);
#endif
    }

    printf("Done.\n");
    built_in_led.fill(WS2812::RGB(0, 0, 100));
    built_in_led.show();

    return 0;
#endif
}
