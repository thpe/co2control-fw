#pragma once

#include "hardware/uart.h"
#include "crc.h"
#include "measurement.hpp"
#include "uart.hpp"

/** Inter-character timeout in us */
#define MODBUS_INTERCHAR_TIMEOUT 750
/** Inter-frame delay in us */
#define MODBUS_INTERFRAME_DELAY 1750

namespace MODBUSSlave {
int resp_len;

char raw_buffer[64];

int chars_rxed = 0;
int tot_chars_rxed = 0;
int finished = 0;
int wait_start = 0;
absolute_time_t last_rx;
int timeout = 0;
crc_t crc = 0;

enum function_code {
  FUNC_READ_COILS = 1,
  FUNC_READHOLDINGS = 3,
  FUNC_READHOLDING = 6,
};

class Request {
public:
  uint16_t addr;
  function_code func;
  uint16_t start;
  uint16_t size;
};
queue_t request_queue;

void clear_state()
{
  chars_rxed = 0;
  finished   = 0;
  wait_start = 0;
  timeout    = 0;
}
void on_modbus_timeout(unsigned int alarm_num) {
  timeout = 1;
}
constexpr int alarm_modbusslave = 1;

template< int UART, int TX, int RX, int DE, int RE >
class Slave {
public:
  Slave() {}

  int init() {
    queue_init(&request_queue, sizeof(Request), 1);
    gpio_set_function(TX, GPIO_FUNC_UART);
    gpio_set_function(RX, GPIO_FUNC_UART);
    uart_init(uart_id<UART>(), baud);

    gpio_init(RE);
    gpio_set_dir(RE, GPIO_OUT);
    gpio_init(DE);
    gpio_set_dir(DE, GPIO_OUT);
    rx_enable();

    hardware_alarm_claim(alarm_modbusslave);
    hardware_alarm_set_callback(alarm_modbusslave, &on_modbus_timeout);

    // Set our data format
    //uart_set_format(uart_id<UART>(), 8, 1, UART_PARITY_NONE);

    // And set up and enable the interrupt hand
    irq_set_exclusive_handler(uart_irq<UART>(), on_slave_rx);
    irq_set_enabled(uart_irq<UART>(), true);

    // Now enable the UART to send interrupts -
    uart_set_irq_enables(uart_id<UART>(), true, false);
    rx_enable();
    return 0;
  }
  /**
  ** Enable RX on transceiver.
  */
  void rx_enable() {
    gpio_put(DE, 0);
    gpio_put(RE, 0);
  }

  /**
  ** Disable transceiver.
  */
  void all_disable() {
    gpio_put(DE, 0);
    gpio_put(RE, 1);
  }

  /**
  ** Enable TX on transceiver.
  */
  void tx_enable() {
    gpio_put(RE, 1);
    gpio_put(DE, 1);
  }


  /**
  ** Callback for one byte.
  */
  static void on_slave_rx() {
    led_toggle(LED3_PIN);
    while (uart_is_readable(uart_id<UART>())) {
      tot_chars_rxed++;
      uint8_t ch = uart_getc(uart_id<UART>());
      if (finished) {
        continue;
      }
      if ((wait_start == 0) && (ch == 0x42)) {
        wait_start = 1;
        chars_rxed = 0;
        crc = crc_init();
      }

      raw_buffer[chars_rxed++] = ch;
      crc = crc_update(crc, &ch, 1);
      if (chars_rxed >= sizeof(raw_buffer)) {
        chars_rxed = 0;
        crc = crc_init();
      }


      if (wait_start == 1 && chars_rxed >= 8) {
        crc = crc_finalize(crc);
        finished=1;
        uint8_t dev_addr = raw_buffer[0];
        function_code fc = (function_code)raw_buffer[1];
        uint16_t start = (raw_buffer[2] << 8 | raw_buffer[3]);
        uint16_t size =  (raw_buffer[4] << 8 | raw_buffer[5]);
        Request r = {dev_addr, fc, start, size};
        queue_try_add(&request_queue, &r);
      }
    }
    last_rx = get_absolute_time();
    absolute_time_t to = delayed_by_us(last_rx, MODBUS_INTERFRAME_DELAY);
    hardware_alarm_set_target(alarm_modbusslave, to);
  }


  int send_holding(uint8_t func, uint8_t size, uint8_t* data)
  {
    tx_enable();
    uint8_t d[32] = {0x42, func, size};
    for (int i = 0; i < size; i++) {
      d[i+3] = data[i];
    }

    tx_arr_crc(uart_id<UART>(), d, size+3);
    sleep_ms(20);
    rx_enable();
    return 1;
  }
  /**
  ** Check the CRC of the response.
  **
  ** \return 1 if CRC is ok, 0 otherwise.
  */
  int check_crc() {
      return crc == 0;
  }
  int check_resp() {
    if (finished && timeout) {
      return check_crc();
    } else {
      return 0;
    }
  }

  void clear() {
    clear_state();
  }
private:


  static constexpr int baud = 19200;
};
}
