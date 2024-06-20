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

#include "modbus.hpp"

// UART1
#define UART1_ID uart1
#define UART1_BAUD 19200
#define UART1_TX_PIN 4
#define UART1_RX_PIN 5

#define LED0_PIN 19
#define LED1_PIN 20
#define LED2_PIN 21
#define LED3_PIN 22


queue_t res_queue;

uint led_toggle(uint led)
{
  uint val = gpio_get(led);
  val = (~val & 0x1);
  gpio_put(led, val);
  return val;
}

void core1_entry() {
  gpio_init(LED0_PIN);
  gpio_set_dir(LED0_PIN, GPIO_OUT);
  printf("booting core 1\r\n");
  gpio_put(LED0_PIN, 1);
  sleep_ms(1000);
  gpio_put(LED0_PIN, 0);
  measurement_t meas;
  while(1) {
    queue_remove_blocking(&res_queue, &meas);
    led_toggle(LED0_PIN);
    modbus_tx_enable();
    print_meas(meas);
    printf("\n");
    sleep_ms(10);
    modbus_rx_enable();
  }
}


int main() {

//    stdio_init_all();
//    uart_init(UART0_ID, UART0_BAUD);
//    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
//    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);

    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(29);

    modbus_init();
    printf("REBOOT\r\n");

    gpio_init(LED1_PIN);
    gpio_init(LED2_PIN);
    gpio_init(LED3_PIN);
    gpio_set_dir(LED1_PIN, GPIO_OUT);
    gpio_set_dir(LED2_PIN, GPIO_OUT);
    gpio_set_dir(LED3_PIN, GPIO_OUT);



    queue_init(&res_queue, sizeof(measurement_t), 2);

    SCD30<1, UART1_TX_PIN, UART1_RX_PIN> scd;
    scd.init();
    gpio_put(LED1_PIN, 1);

    printf("UART1 initialised\r\n");


    sleep_ms(1000);
    scd.txReset();
    scd.check_resp();
    gpio_put(LED2_PIN, 1);

    sleep_ms(1000);
    scd.txContStart();
    scd.check_resp();
    printf("continous measurement startet\r\n");
    gpio_put(LED3_PIN, 1);

    measurement_t measurement;
    multicore_reset_core1();
    multicore_launch_core1(&core1_entry);

    const float conversion_factor = 3.3f / (1 << 12) * 2.0;
    while (true) {
        gpio_put(LED1_PIN, 1);
        gpio_put(LED2_PIN, 0);
        scd.txMeas();
        gpio_put(LED1_PIN, 0);
        gpio_put(LED2_PIN, 1);
        led_toggle(LED3_PIN);
        sleep_ms(2000);
        adc_select_input(0);
        uint16_t result1 = adc_read();
        adc_select_input(3);
        uint16_t result2 = adc_read();
//      printf("DATA,%f,", result1 * conversion_factor);
//      printf("%f,", result2 * conversion_factor);
//        scd.printMeas(false);//true);
        scd.printStatus();
        sleep_ms(1000);
        scd.getMeas(measurement);
        measurement.v_cell = result1 * conversion_factor;
        measurement.v_sys  = result2 * conversion_factor;
        queue_try_add(&res_queue, &measurement);
//        printf("\r\n");
//        scd.check_resp();
    }
}
