#pragma once
#include "hardware/uart.h"



const uint8_t zero[] =       {0x00};
const uint8_t start_cont[] = {0x61, 0x06, 0x00, 0x36, 0x00, 0x00, 0x60, 0x64};
const uint8_t stop_cont[]  = {0x61, 0x06, 0x00, 0x37, 0x00, 0x01, 0xF0, 0x64};
const uint8_t soft_reset[] = {0x61, 0x06, 0x00, 0x34, 0x00, 0x01, 0x00, 0x64};
const uint8_t read_meas[]  = {0x61, 0x03, 0x00, 0x28, 0x00, 0x06, 0x4C, 0x60};
const uint8_t ready[]      = {0x61, 0x03, 0x00, 0x27, 0x00, 0x01, 0x3D, 0xA1};
int resp_len;

char raw_buffer[64];

int chars_rxed = 0;
int tot_chars_rxed = 0;
int finished = 0;
int wait_start = 0;

template< int ID > uart_inst_t* uart_id();
template<> uart_inst_t* uart_id<0> () {return uart0;}
template<> uart_inst_t* uart_id<1> () {return uart1;}

template< int ID > unsigned int uart_irq();
template<> unsigned int uart_irq<0> () {return UART0_IRQ;}
template<> unsigned int uart_irq<1> () {return UART1_IRQ;}

uint8_t tx_arr(const uint8_t* ptr, int len) {
#if 0
  uart_puts(UART0_ID, "tx: ");
  for (int i = 0; i < len; i++) {
    printf("%x,", (int)ptr[i]);
  }
  wait_start = 0;
  for (int i = 0; i < len; i++) {
    uart_putc(UART1_ID, ptr[i]);
    sleep_us(300);
  }
#endif
  //uart_write_blocking(UART1_ID, ptr, len);
#if 0
  uart_puts(UART0_ID, "tx END\r\n");
#endif
  return 0;
}
template< int UART, int TX, int RX >
class SCD30 {
public:
  SCD30() {}

  int init() {
    uart_init(uart_id<UART>(), baud);
    gpio_set_function(TX, GPIO_FUNC_UART);
    gpio_set_function(RX, GPIO_FUNC_UART);
    uart_set_fifo_enabled(uart_id<UART>(), true);
    uart_set_hw_flow(uart_id<UART>(), false, false);

    // Set our data format
    uart_set_format(uart_id<UART>(), 8, 1, UART_PARITY_NONE);

    // And set up and enable the interrupt hand
    irq_set_exclusive_handler(uart_irq<UART>(), on_uart_rx);
    irq_set_enabled(uart_irq<UART>(), true);

    // Now enable the UART to send interrupts -
    uart_set_irq_enables(uart_id<UART>(), true, false);
    return 0;
  }

  void txContStart() {
    resp_len = sizeof(start_cont);
    uart_write_blocking(uart_id<UART>(), start_cont, sizeof(start_cont));
  }
  void txMeas() {
    resp_len = 17;
    uart_write_blocking(uart_id<UART>(), read_meas, sizeof(read_meas));
  }

  static void on_uart_rx() {
    while (uart_is_readable(uart_id<UART>())) {
      tot_chars_rxed++;
      uint8_t ch = uart_getc(uart_id<UART>());
      if (finished) {
        continue;
      }
      if ((wait_start == 0) && (ch == 0x61)) {
        //    printf("X%xX\r\n", (int)ch);
        wait_start = 1;
        chars_rxed = 0;
      }

      raw_buffer[chars_rxed++] = ch;
      if (chars_rxed >= sizeof(raw_buffer)) {
        chars_rxed = 0;
      }


      if (wait_start == 1 && chars_rxed > resp_len) {
        printf("IRQ: rx finished\r\n");
        finished=1;
      }
    }
  }
  int check_resp(){
    if (finished) {
      for (int i = 0; i < chars_rxed; i++) {
        printf("%x,", (int)raw_buffer[i]);
      }
      uart_puts(uart_id<UART>(), "\r\n");
      finished= 0;
      chars_rxed = 0;
      return 1;
    } else {
      printf("not finished %d\r\n", tot_chars_rxed);
      for (int i = 0; i < chars_rxed; i++) {
        printf("%x,", (int)raw_buffer[i]);
      }
      uart_puts(uart_id<UART>(), "\r\n");
      return 0;
    }
  }
private:
  static constexpr int baud = 19200;
};
