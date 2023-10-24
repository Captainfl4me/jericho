#include <stdio.h>
#include <bits/stdc++.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"

#include "WS2812.hpp"

#define LED_PIN 16
#define LED_LENGTH 1

struct RGB{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

RGB HSVtoRGB(float H, float S,float V){
    if(H>360 || H<0 || S>100 || S<0 || V>100 || V<0){
        return RGB{0,0,0};
    }
    float s = S/100;
    float v = V/100;
    float C = s*v;
    float X = C*(1-std::abs(std::fmod(H/60.0, 2)-1));
    float m = v-C;
    float r,g,b;
    if(H >= 0 && H < 60){
        r = C,g = X,b = 0;
    }
    else if(H >= 60 && H < 120){
        r = X,g = C,b = 0;
    }
    else if(H >= 120 && H < 180){
        r = 0,g = C,b = X;
    }
    else if(H >= 180 && H < 240){
        r = 0,g = X,b = C;
    }
    else if(H >= 240 && H < 300){
        r = X,g = 0,b = C;
    }
    else{
        r = C,g = 0,b = X;
    }
    int R = (r+m)*255;
    int G = (g+m)*255;
    int B = (b+m)*255;

    return RGB{(uint8_t)R,(uint8_t)G,(uint8_t)B};
}

void core1_main() {
    WS2812 built_in_led(LED_PIN, LED_LENGTH, pio0, 0, WS2812::FORMAT_GRB);
    while (true) {    
        for (int i = 0; i < 360; i++) {
            RGB rgb = HSVtoRGB(i,100,40);
            built_in_led.fill(WS2812::RGB(rgb.r, rgb.g, rgb.b));
            built_in_led.show();
            sleep_ms(5);
        }
    }

}


// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int main() {
    // Enable UART so we can print status output
    stdio_init_all();

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/bus_scan example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    multicore_launch_core1(core1_main);
    while(true) {
        printf("------- Start scan ------\n");
        for (int addr = 0; addr < (1 << 7); ++addr) {
            // Perform a 1-byte dummy read from the probe address. If a slave
            // acknowledges this address, the function returns the number of bytes
            // transferred. If the address byte is ignored, the function returns
            // -1.

            // Skip over any reserved addresses.
            int ret;
            uint8_t rxdata;
            if (reserved_addr(addr))
                ret = PICO_ERROR_GENERIC;
            else
                ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);
            
            if (ret >= 0) {
                printf("Found: 0x%02x\n", addr);
            }
        }
        printf("Done.\n");
        sleep_ms(1000);
    }
    return 0;
#endif
}