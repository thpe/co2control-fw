/**
 * Copyright (c) 2024 Thomas Petig
 */

#include "pico/stdlib.h"

int main() {
    const uint LED1_PIN = 20;
    const uint LED2_PIN = 21;
    gpio_init(LED1_PIN);
    gpio_init(LED2_PIN);
    gpio_set_dir(LED1_PIN, GPIO_OUT);
    gpio_set_dir(LED2_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED1_PIN, 1);
        gpio_put(LED2_PIN, 0);
        sleep_ms(250);
        gpio_put(LED1_PIN, 0);
        gpio_put(LED2_PIN, 1);
        sleep_ms(250);
    }
}
