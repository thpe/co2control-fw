/**
 * Copyright (c) 2024 Thomas Petig
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "scd30.hpp"
#include "pico/util/queue.h"
#include "pico/multicore.h"


// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART0_ID uart0
#define UART0_BAUD 115200
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1
#define UART0_DE_PIN 2
#define UART0_RE_PIN 3

// UART1
#define UART1_ID uart1
#define UART1_BAUD 19200
#define UART1_TX_PIN 4
#define UART1_RX_PIN 5


typedef struct
{
  float co2;
  float temp;
  float hum;
  float v_sys;
  float v_cell;
} measurement_t;

queue_t res_queue;

void core1_entry() {
  while(1) {
    measurement_t meas;
    queue_remove_blocking(&res_queue, &meas);
  }
}

void modbus_rx_enable() {
  gpio_put(UART0_DE_PIN, 0);
  gpio_put(UART0_RE_PIN, 0);
}

void modbus_all_disable() {
  gpio_put(UART0_DE_PIN, 0);
  gpio_put(UART0_RE_PIN, 1);
}

void modbus_tx_enable() {
  gpio_put(UART0_RE_PIN, 1);
  gpio_put(UART0_DE_PIN, 1);
}

int main() {

//    stdio_init_all();
//    uart_init(UART0_ID, UART0_BAUD);
//    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
//    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);
    stdio_uart_init_full(UART0_ID, UART0_BAUD, UART0_TX_PIN, UART0_RX_PIN);
    printf("REBOOT\r\n");
    gpio_init(UART0_RE_PIN);
    gpio_set_dir(UART0_RE_PIN, GPIO_OUT);
    gpio_put(UART0_RE_PIN, 1);
    gpio_init(UART0_DE_PIN);
    gpio_set_dir(UART0_DE_PIN, GPIO_OUT);
    gpio_put(UART0_DE_PIN, 1);

    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(29);


    const uint LED1_PIN = 20;
    const uint LED2_PIN = 21;
    gpio_init(LED1_PIN);
    gpio_init(LED2_PIN);
    gpio_set_dir(LED1_PIN, GPIO_OUT);
    gpio_set_dir(LED2_PIN, GPIO_OUT);


    queue_init(&res_queue, sizeof(measurement_t), 2);


    SCD30<1, UART1_TX_PIN, UART1_RX_PIN> scd;
    scd.init();

    printf("UART1 initialised\r\n");


    sleep_ms(1000);
    scd.txReset();
    scd.check_resp();

    sleep_ms(1000);
    scd.txContStart();
    scd.check_resp();
    printf("continous measurement startet\r\n");

    const float conversion_factor = 3.3f / (1 << 12) * 2.0;
    while (true) {
        gpio_put(LED1_PIN, 1);
        gpio_put(LED2_PIN, 0);
        scd.txMeas();
        gpio_put(LED1_PIN, 0);
        gpio_put(LED2_PIN, 1);
        sleep_ms(2000);
        adc_select_input(0);
        uint16_t result1 = adc_read();
        adc_select_input(3);
        uint16_t result2 = adc_read();
        modbus_tx_enable();
        printf("DATA,%f,", result1 * conversion_factor);
        printf("%f,", result2 * conversion_factor);
        scd.printMeas(true);
        printf("\r\n");
        sleep_ms(10);
        modbus_rx_enable();
//        scd.check_resp();
    }
}
