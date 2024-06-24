/**
 * Copyright (c) 2024 Thomas Petig
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "scd30.hpp"
#include "pico/util/queue.h"
#include "pico/multicore.h"

#include "led.hpp"
#include "adc.hpp"

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


queue_t res_queue;

void core1_entry() {
  watchdog_enable(5000,1);
  led_init(LED0_PIN);
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
    watchdog_update();
    //modbus_rx_enable();
  }
}


int main() {

    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(29);

    modbus_init();
    printf("REBOOT\r\n");

    led_init(LED1_PIN);
    led_init(LED2_PIN);
    led_init(LED3_PIN);



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

    while (true) {
        led_toggle(LED2_PIN);
        gpio_put(LED1_PIN, 1);
        scd.txMeas();
        gpio_put(LED1_PIN, 0);
        led_toggle(LED3_PIN);
        sleep_ms(2000);
//        scd.printMeas(false);//true);
        //scd.printStatus();
        if (scd.check_resp()) {
          scd.getMeas(measurement);
          measurement.v_cell = adc_read_V(ADC_CELL);
          measurement.v_sys  = adc_read_V(ADC_SYS);
          queue_try_add(&res_queue, &measurement);
        } else {
          printf("ERROR\r\n");
          scd.printStatus();
        }
    }
}

