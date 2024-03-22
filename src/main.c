/**
 * Copyright (c) 2024 Thomas Petig
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define UART_ID uart0
#define BAUD_RATE 115200

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1
#define UART0_DE_PIN 2
#define UART0_RE_PIN 3



int main() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);

    const uint LED1_PIN = 20;
    const uint LED2_PIN = 21;
    gpio_init(LED1_PIN);
    gpio_init(LED2_PIN);
    gpio_set_dir(LED1_PIN, GPIO_OUT);
    gpio_set_dir(LED2_PIN, GPIO_OUT);
    gpio_set_dir(UART0_DE_PIN, GPIO_OUT);
    gpio_set_dir(UART0_RE_PIN, GPIO_OUT);
    while (true) {
        uart_puts(UART_ID, " Hello, UART!\n");
        gpio_put(UART0_DE_PIN, 1);
        gpio_put(UART0_RE_PIN, 0);
        gpio_put(LED1_PIN, 1);
        gpio_put(LED2_PIN, 0);
        sleep_ms(1000);
        gpio_put(LED1_PIN, 0);
        gpio_put(LED2_PIN, 1);
        gpio_put(UART0_DE_PIN, 0);
        gpio_put(UART0_RE_PIN, 1);
        sleep_ms(1000);
    }
}
