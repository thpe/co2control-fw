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
#include "modbusslave.hpp"

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART0_ID uart0
#define UART0_BAUD 19200
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


uint16_t hton(uint16_t val)
{
  return (val & 0xFF) << 8 | (val & 0xFF00) >> 8;
}

queue_t res_queue;

void core1_entry() {
  gpio_put(LED3_PIN, 1);
  watchdog_enable(5000,1);
  led_init(LED0_PIN);
  gpio_put(LED0_PIN, 1);
  sleep_ms(1000);
  gpio_put(LED0_PIN, 0);
  measurement_t meas;
  MODBUSSlave::Slave< 0, UART0_TX_PIN, UART0_RX_PIN, UART0_DE_PIN, UART0_RE_PIN> slave;
  slave.init();
  slave.clear();
  gpio_put(LED3_PIN, 0);
  uint16_t reg[32];
  for (int i = 0; i < 32; i++) {
    reg[i] = 0;
  }
  while(1) {
    if(queue_try_remove(&res_queue, &meas)) {
       led_toggle(LED1_PIN);
       reg[0x10] = hton(meas.co2);
       reg[0x11] = hton(meas.temp*10);
       reg[0x12] = hton(meas.hum);
       reg[0x13] = hton(meas.v_sys * 1000);
       reg[0x14] = hton(meas.v_cell * 1000);
//      print_meas(meas);
      watchdog_update();
    }
    MODBUSSlave::Request r;
    if (queue_try_remove(&MODBUSSlave::request_queue, &r)) {
      gpio_put(LED0_PIN, 1);
      sleep_ms(1);
      slave.clear();
      if (r.size > 15) {
        r.size = 15;
      }
      slave.send_holding(r.func, r.size*2, (uint8_t*)(reg+r.start));
      gpio_put(LED0_PIN, 0);
    }
  }
}


int main() {

    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(29);

  //  modbus_init();

    led_init(LED1_PIN);
    led_init(LED2_PIN);
    led_init(LED3_PIN);



    queue_init(&res_queue, sizeof(measurement_t), 2);

    SCD30<1, UART1_TX_PIN, UART1_RX_PIN> scd;
    scd.init();
    gpio_put(LED1_PIN, 1);



    sleep_ms(1000);
    scd.txReset();
    scd.check_resp();
    gpio_put(LED2_PIN, 1);

    sleep_ms(1000);
    scd.txContStart();
    scd.check_resp();

    measurement_t measurement;
    multicore_reset_core1();
    multicore_launch_core1(&core1_entry);

    while (true) {
        gpio_put(LED2_PIN, 1);
        scd.txMeas();
        gpio_put(LED2_PIN, 0);
        sleep_ms(2000);
//        scd.printMeas(false);//true);
        //scd.printStatus();
        if (scd.check_resp()) {
          scd.getMeas(measurement);
          measurement.v_cell = adc_read_V(ADC_CELL);
          measurement.v_sys  = adc_read_V(ADC_SYS);
          queue_try_add(&res_queue, &measurement);
        } else {
          //scd.printStatus();
        }
    }
}

