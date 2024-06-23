#pragma once

#include "hardware/uart.h"
#include "crc.h"
#include "measurement.hpp"

/** Inter-character timeout in us */
#define MODBUS_INTERCHAR_TIMEOUT 750
/** Inter-frame delay in us */
#define MODBUS_INTERFRAME_DELAY 1750

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
absolute_time_t last_rx;
int timeout = 0;
void clear_state()
{
  chars_rxed = 0;
  finished   = 0;
  wait_start = 0;
  timeout    = 0;
}
void on_timeout(unsigned int alarm_num) {
  timeout = 1;
}
constexpr int alarm_scd30 = 0;

template< int ID > uart_inst_t* uart_id();
template<> uart_inst_t* uart_id<0> () {return uart0;}
template<> uart_inst_t* uart_id<1> () {return uart1;}

template< int ID > unsigned int uart_irq();
template<> unsigned int uart_irq<0> () {return UART0_IRQ;}
template<> unsigned int uart_irq<1> () {return UART1_IRQ;}

template< int UART, int TX, int RX >
class SCD30 {
public:
  SCD30() {}

  int init() {
    gpio_set_function(TX, GPIO_FUNC_UART);
    gpio_set_function(RX, GPIO_FUNC_UART);
    uart_init(uart_id<UART>(), baud);
    uart_set_fifo_enabled(uart_id<UART>(), false);
    uart_set_hw_flow(uart_id<UART>(), false, false);

    hardware_alarm_claim(alarm_scd30);
    hardware_alarm_set_callback(alarm_scd30, &on_timeout);

    // Set our data format
    //uart_set_format(uart_id<UART>(), 8, 1, UART_PARITY_NONE);

    // And set up and enable the interrupt hand
    irq_set_exclusive_handler(uart_irq<UART>(), on_uart_rx);
    irq_set_enabled(uart_irq<UART>(), true);

    // Now enable the UART to send interrupts -
    uart_set_irq_enables(uart_id<UART>(), true, false);
    return 0;
  }

  void txReset() {
    resp_len = sizeof(soft_reset);
    clear_state();
    tx_arr_crc(uart_id<UART>(), soft_reset, sizeof(soft_reset)-2);
  }

  void txContStart() {
    resp_len = sizeof(start_cont);
    clear_state();
    tx_arr_crc(uart_id<UART>(), start_cont, sizeof(start_cont)-2);
  }

  /**
  ** Send measurement request.
  */
  void txMeas() {
    clear_state();
    resp_len = readMultipleHoldingRegister(uart_id<UART>(), 0x61, 0x28, 0x06);
  }


  float getCO2 () {
    uint32_t co2 = ((uint32_t)raw_buffer[3]) << 24 |
                   ((uint32_t)raw_buffer[4]) << 16 |
                   ((uint32_t)raw_buffer[5]) << 8  |
                   ((uint32_t)raw_buffer[6]);
    float co2f = *(float*)&co2;
    return co2f;
  }

  float getTemp() {
    uint32_t temp = ((uint32_t)raw_buffer[7]) << 24 |
                    ((uint32_t)raw_buffer[8]) << 16 |
                    ((uint32_t)raw_buffer[9]) << 8  |
                    ((uint32_t)raw_buffer[10]);
    float tempf = *(float*)&temp;
    return tempf;
  }

  float getHum() {
    uint32_t hum = ((uint32_t)raw_buffer[11]) << 24 |
                   ((uint32_t)raw_buffer[12]) << 16 |
                   ((uint32_t)raw_buffer[13]) << 8  |
                   ((uint32_t)raw_buffer[14]);
    float humf = *(float*)&hum;
    return humf;
  }

  void getMeas(measurement_t& measurement) {
    measurement.co2  = getCO2();
    measurement.temp = getTemp();
    measurement.hum  = getHum();
  }
  void printMeas(bool csv = false) {
    float co2f  = getCO2();
    float tempf = getTemp();
    float humf  = getHum();
    if (csv) {
      printf("%f,%f,%f", co2f, tempf, humf);
    } else {
      printf("CO2: %f, temp %f C, hum %f %%\r\n", co2f, tempf, humf);
    }
  }
  void printStatus() {
    printf("SCD30 status: rx chars %d (%d), finished %d \r\n", chars_rxed, tot_chars_rxed, finished);
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
      }
//      printf("RX %d-%x,", chars_rxed, (int)ch);

      raw_buffer[chars_rxed++] = ch;
      if (chars_rxed >= sizeof(raw_buffer)) {
        chars_rxed = 0;
      }


      if (wait_start == 1 && chars_rxed >= resp_len) {
//        printf("IRQ: rx finished\r\n");
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
      crc_t crc = crc_init();
      crc = crc_update(crc, raw_buffer, resp_len);
      crc = crc_finalize(crc);
      return crc == 0;
  }

  int check_resp() {
    if (finished && timeout) {
      return check_crc();
    } else {
      return 0;
    }
  }
  int print_resp() {
    if (finished) {
      crc_t crc = crc_init ();
      crc = crc_update (crc, raw_buffer, resp_len);
      crc = crc_finalize (crc);

      for (int i = 0; i < chars_rxed; i++) {
        printf("%x,", (int)raw_buffer[i]);
      }
      printf("\r\n");
      finished= 0;
      chars_rxed = 0;
      printf("CRC %x\r\n", (int)crc);
      return 1;
    } else {
      printf("not finished %d\r\n", tot_chars_rxed);
      for (int i = 0; i < chars_rxed; i++) {
        printf("%x,", (int)raw_buffer[i]);
      }
      printf("\r\n");
      return 0;
    }
  }
private:


  static constexpr int baud = 19200;
};
