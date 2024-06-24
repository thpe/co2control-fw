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

template< int UART, int TX, int RX, int TXEN, int RXEN >
class Slave {
public:
  Slave() {}

  int init() {
    queue_init(&request_queue, sizeof(Request), 1);
    gpio_set_function(TX, GPIO_FUNC_UART);
    gpio_set_function(RX, GPIO_FUNC_UART);
    uart_init(uart_id<UART>(), baud);
    uart_set_fifo_enabled(uart_id<UART>(), false);
    uart_set_hw_flow(uart_id<UART>(), false, false);

    hardware_alarm_claim(alarm_modbusslave);
    hardware_alarm_set_callback(alarm_modbusslave, &on_timeout);

    // Set our data format
    //uart_set_format(uart_id<UART>(), 8, 1, UART_PARITY_NONE);

    // And set up and enable the interrupt hand
    irq_set_exclusive_handler(uart_irq<UART>(), on_uart_rx);
    irq_set_enabled(uart_irq<UART>(), true);

    // Now enable the UART to send interrupts -
    uart_set_irq_enables(uart_id<UART>(), true, false);
    return 0;
  }


  /**
  ** Callback for one byte.
  */
  static void on_uart_rx() {
    while (uart_is_readable(uart_id<UART>())) {
      tot_chars_rxed++;
      uint8_t ch = uart_getc(uart_id<UART>());
      if (finished) {
        continue;
      }
      if ((wait_start == 0) && (ch == 0x61)) {
//        printf("X%xX\r\n", (int)ch);
        wait_start = 1;
        chars_rxed = 0;
        crc = crc_init();
      }
//      printf("RX %d-%x,", chars_rxed, (int)ch);

      raw_buffer[chars_rxed++] = ch;
      crc = crc_update(crc, &ch, 1);
      if (chars_rxed >= sizeof(raw_buffer)) {
        chars_rxed = 0;
        crc = crc_init();
      }


      if (wait_start == 1 && chars_rxed >= resp_len) {
        crc = crc_finalize(crc);
        finished=1;
      }
    }
    last_rx = get_absolute_time();
    absolute_time_t to = delayed_by_us(last_rx, MODBUS_INTERFRAME_DELAY);
    hardware_alarm_set_target(alarm_scd30, to);
  }


  /**
  ** Check the CRC of the response.
  **
  ** \return 1 if CRC is ok, 0 otherwise.
  */
  int check_crc() {
      return crc == 0;
  }

private:


  static constexpr int baud = 19200;
};
}
