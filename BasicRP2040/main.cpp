#include <stdio.h>
#include <cmath>
#include "pico/stdlib.h"
#include "WS2812.hpp"

#define LED_PIN 16
#define LED_LENGTH 1 

int main() {
    stdio_init_all();

    // 0. Initialize LED strip
    printf("0. Initialize LED strip");
    WS2812 built_in_led(LED_PIN, LED_LENGTH, pio0, 0, WS2812::FORMAT_RGB);

    while(true) {
        for (int i = 0; i < 255; i++) {
            built_in_led.fill(WS2812::RGB(i, 0, 0));
            built_in_led.show();
            sleep_ms(10);
        }
        for (int i = 0; i < 255; i++) {
            built_in_led.fill(WS2812::RGB(0, i, 0));
            built_in_led.show();
            sleep_ms(10);
        }
        for (int i = 0; i < 255; i++) {
            built_in_led.fill(WS2812::RGB(0, 0, i));
            built_in_led.show();
            sleep_ms(10);
        }
    }

    return 0;
}
