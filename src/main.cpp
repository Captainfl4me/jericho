/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"

#include "WS2812.hpp"
#include "MPU6050.hpp"
#include "BMP280.hpp"
#include "logger.hpp"

#define LED_PIN 16
#define LED_LENGTH 1

#define WRITE_LOG 0x0001
#define WRITE_DATA 0x0002
#define SHUTDOWN_CORE 0xf003

void start_blink(WS2812* built_in_led, uint8_t red, uint8_t green, uint8_t blue, uint32_t delay_ms) {
    while (true) {
        built_in_led->fill(WS2812::RGB(red, green, blue));
        built_in_led->show();
        sleep_ms(delay_ms);
        built_in_led->fill(WS2812::RGB(red, green, blue));
        built_in_led->show();
        sleep_ms(delay_ms);
    }
}

void start_blink_red() {
    WS2812 built_in_led(LED_PIN, LED_LENGTH, pio0, 0, WS2812::FORMAT_GRB);
    start_blink(&built_in_led, 255, 0, 0, 100);
}

void start_blink_green() {
    WS2812 built_in_led(LED_PIN, LED_LENGTH, pio0, 0, WS2812::FORMAT_GRB);
    start_blink(&built_in_led, 0, 255, 0, 100);
}

void core1_main() {
    uint32_t command;
    while(true) {
        if (!Logger::logger->is_fifo_empty()) {
            Logger::logger->write_all_data_from_fifo();
        }

        if(multicore_fifo_rvalid()){
            command = multicore_fifo_pop_blocking();
            if (command == SHUTDOWN_CORE) {        
                if (!Logger::logger->is_fifo_empty()) {
                    Logger::logger->write_all_data_from_fifo();
                }
                Logger::logger->write_log("Shutingdown core1...");
                multicore_fifo_push_blocking(SHUTDOWN_CORE);
                break;
            }
        }
    }

    sleep_ms(100);
    return;
}

int main() {
    // Enable UART so we can print status output
    stdio_init_all();
    Logger::logger = new Logger(0, 1, 2, 3, 12500 * 1000, spi0);
    Logger::logger->write_log("RP2040 log start!");

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/bus_scan example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    WS2812 built_in_led(LED_PIN, LED_LENGTH, pio0, 0, WS2812::FORMAT_GRB);
    built_in_led.fill(WS2812::RGB(255, 0, 0));
    built_in_led.show();
    multicore_launch_core1(start_blink_red);

    // Init I2C 0 with 400KHz
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    MPU6050 mpu6050(0x68, (uint8_t)1000);
    mpu6050.set_accel_range(mpu_6050_range::MPU6050_RANGE_16G);
    mpu6050.set_gyro_scale(mpu_6050_scale::MPU6050_SCALE_1000DPS);
    BMP280 bmp280(0x76);

    mpu6050.calibrate(1000);
    multicore_reset_core1();

    multicore_launch_core1(start_blink_green);
    Logger::logger->write_log("Initialized");
    if (mpu6050.test_connection()) Logger::logger->write_log("MPU6050 connection successful");
    else {
        Logger::logger->write_error("MPU6050 connection failed");
        built_in_led.fill(WS2812::RGB(100, 0, 100));
        built_in_led.show();
        return 1;
    }
    if (bmp280.test_connection()) Logger::logger->write_log("HW611 connection successful");
    else {
        Logger::logger->write_error("HW611 connection failed");
        built_in_led.fill(WS2812::RGB(100, 0, 100));
        built_in_led.show();
        return 1;
    }
    if (Logger::logger->test_connection()) Logger::logger->write_log("SD card connection successful");
    else {
        Logger::logger->write_error("SD card connection failed");
        built_in_led.fill(WS2812::RGB(100, 0, 100));
        built_in_led.show();
        return 1;
    }
    multicore_reset_core1();

    // If all tests are sucessfull wait 1 second and start main loop
    built_in_led.fill(WS2812::RGB(0, 100, 0));
    built_in_led.show();
    sleep_ms(1000);
    
    built_in_led.fill(WS2812::RGB(100, 100, 0));
    built_in_led.show();
    Logger::logger->write_log("Initialize finish starting core1...");
    multicore_launch_core1(core1_main);
    multicore_fifo_drain();
    Logger::logger->write_log("Starting loop...");

    data_t data;
    while(true) {
#ifdef DEBUG
        uint32_t startTime = time_us_32();
#endif

        mpu6050.update();
        bmp280.update();

        data.time = time_us_32();
        data.acc.x = mpu6050.data.acc.x;
        data.acc.y = mpu6050.data.acc.y;
        data.acc.z = mpu6050.data.acc.z;
        data.gyro.x = mpu6050.data.gyro.x;
        data.gyro.y = mpu6050.data.gyro.y;
        data.gyro.z = mpu6050.data.gyro.z;
        data.pressure = bmp280.data.pressure;

        Logger::logger->push_data_to_fifo(&data);

        // if(time_us_32() > 30 * 1000000) {
        //     multicore_fifo_push_blocking(SHUTDOWN_CORE);
        //     break;
        // }

#ifdef DEBUG
        uint32_t executionTime = time_us_32() - startTime;
        //printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%f\n", executionTime, mpu6050.raw_acc[0], mpu6050.raw_acc[1], mpu6050.raw_acc[2], mpu6050.raw_gyro[0], mpu6050.raw_gyro[1], mpu6050.raw_gyro[2], mpu6050.temp);
        printf("%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%.3f\n", executionTime, mpu6050.data.acc.x, mpu6050.data.acc.y, mpu6050.data.acc.z, mpu6050.data.gyro.x, mpu6050.data.gyro.y, mpu6050.data.gyro.z, bmp280.data.temp, bmp280.data.pressure);
#endif
    }
    sleep_ms(100);
    Logger::logger->write_log("Waiting for core1 to finish...");

    // Wait for core 1 to shutdown
    while(!multicore_fifo_rvalid()){
        sleep_ms(1);
    }
    uint32_t command = multicore_fifo_pop_blocking();
    if (command == SHUTDOWN_CORE) {
        Logger::logger->write_log("Confirm Shutdown core1");
    }else {
        Logger::logger->write_error("Core1 shutdown failed");
        start_blink_red();
    }

    Logger::logger->write_log("Shutdown core 0.\n");
    built_in_led.fill(WS2812::RGB(0, 0, 200));
    built_in_led.show();

    return 0;
#endif
}
