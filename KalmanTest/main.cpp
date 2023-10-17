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
#include "HW611.hpp"
#include "logger.hpp"

#define LED_PIN 16
#define LED_LENGTH 1

#define WRITE_LOG 0x0001
#define WRITE_DATA 0x0002
#define SHUTDOWN_CORE 0xf003

void panic_blink_freeze(WS2812* built_in_led) {
    while (true) {
        built_in_led->fill(WS2812::RGB(255, 0, 0));
        built_in_led->show();
        sleep_ms(100);
        built_in_led->fill(WS2812::RGB(0, 0, 0));
        built_in_led->show();
        sleep_ms(100);
    }
}

void apply_fifo_command(uint32_t command) {
    if (command == WRITE_DATA) {
        uint32_t time = multicore_fifo_pop_blocking();
        int16_t acc_x = multicore_fifo_pop_blocking();
        int16_t acc_y = multicore_fifo_pop_blocking();
        int16_t acc_z = multicore_fifo_pop_blocking();
        int16_t gyro_x = multicore_fifo_pop_blocking();
        int16_t gyro_y = multicore_fifo_pop_blocking();
        int16_t gyro_z = multicore_fifo_pop_blocking();
            
        Logger::logger->write_data(time, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
    }else if (command == SHUTDOWN_CORE) {
        while(multicore_fifo_rvalid()) {
            command = multicore_fifo_pop_blocking();
            printf("Command: %d\n", command);
            apply_fifo_command(command);
        }
    }
}

void core1_main() {
    WS2812 built_in_led(LED_PIN, LED_LENGTH, pio0, 0, WS2812::FORMAT_GRB);

    //Init finished
    built_in_led.fill(WS2812::RGB(0, 255, 0));
    built_in_led.show();

    uint32_t command;
    while(true) {
        command = multicore_fifo_pop_blocking();
        printf("Command: %d\n", command);
        apply_fifo_command(command);

        if (command == SHUTDOWN_CORE) {
            Logger::logger->write_log("Shutingdown core1...");
            multicore_fifo_push_blocking(SHUTDOWN_CORE);
            break;
        }
    }

    sleep_ms(100);
    return;
}

int main() {
    // Enable UART so we can print status output
    stdio_init_all();
    Logger::logger = new Logger(0, 1, 2, 3, 12500 * 1000, spi0);
    Logger::logger->write_log("RP2040 log start!\n");

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/bus_scan example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
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
    Logger::logger->write_log("I2C initialized");

    MPU6050 mpu6050(0x68);
    HW611 hw611(0x76);

    if (mpu6050.testConnection()) Logger::logger->write_log("MPU6050 connection successful");
    else {
        Logger::logger->write_error("MPU6050 connection failed");
        return 1;
    }
    if (hw611.testConnection()) Logger::logger->write_log("HW611 connection successful");
    else {
        Logger::logger->write_error("HW611 connection failed");
        return 1;
    }

    sleep_ms(1000);
    built_in_led.fill(WS2812::RGB(100, 100, 0));
    built_in_led.show();
    Logger::logger->write_log("Initialize finish starting core1...");
    
    multicore_launch_core1(core1_main);
    multicore_fifo_drain();
    Logger::logger->write_log("Starting loop...");
    while(true) {
#ifdef DEBUG
        uint32_t startTime = time_us_32();
#endif

        mpu6050.updateData();
        hw611.updateData();

        if (multicore_fifo_wready()) {
            printf("Write to fifo\n");
            multicore_fifo_push_blocking(WRITE_DATA);
            multicore_fifo_push_blocking(startTime);
            multicore_fifo_push_blocking(mpu6050.raw_acc[0]);
            multicore_fifo_push_blocking(mpu6050.raw_acc[1]);
            multicore_fifo_push_blocking(mpu6050.raw_acc[2]);
            multicore_fifo_push_blocking(mpu6050.raw_gyro[0]);
            multicore_fifo_push_blocking(mpu6050.raw_gyro[1]);
            multicore_fifo_push_blocking(mpu6050.raw_gyro[2]);
        } else {
            printf("FIFO not ready\n");
        }

        if(time_us_32() > 10 * 1000000) {
            multicore_fifo_push_blocking(SHUTDOWN_CORE);
            break;
        }

#ifdef DEBUG
        uint32_t executionTime = time_us_32() - startTime;
        //printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%f\n", executionTime, mpu6050.raw_acc[0], mpu6050.raw_acc[1], mpu6050.raw_acc[2], mpu6050.raw_gyro[0], mpu6050.raw_gyro[1], mpu6050.raw_gyro[2], mpu6050.temp);
        printf("%d\t%f\t%f\t%f\t%f\t%.3f\n", executionTime, mpu6050.gyro[0], mpu6050.gyro[1], mpu6050.gyro[2], hw611.temp, hw611.pressure);
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
        panic_blink_freeze(&built_in_led);
    }

    Logger::logger->write_log("Shutdown core 0.\n");
    built_in_led.fill(WS2812::RGB(0, 0, 200));
    built_in_led.show();

    return 0;
#endif
}
