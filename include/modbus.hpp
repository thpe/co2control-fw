/**
** @file
** @author Thomas Petig
** @section DESCRIPTION
** MODBUS interface
*/
#pragma once
#include "crc.h"

/**
** Initialising modbus on UART0
*/
void modbus_init()
{
  stdio_uart_init_full(UART0_ID, UART0_BAUD, UART0_TX_PIN, UART0_RX_PIN);
  gpio_init(UART0_RE_PIN);
  gpio_set_dir(UART0_RE_PIN, GPIO_OUT);
  gpio_put(UART0_RE_PIN, 1);
  gpio_init(UART0_DE_PIN);
  gpio_set_dir(UART0_DE_PIN, GPIO_OUT);
  gpio_put(UART0_DE_PIN, 1);
}

/**
** Enable RX on transceiver.
*/
void modbus_rx_enable() {
  gpio_put(UART0_DE_PIN, 0);
  gpio_put(UART0_RE_PIN, 0);
}

/**
** Disable transceiver.
*/
void modbus_all_disable() {
  gpio_put(UART0_DE_PIN, 0);
  gpio_put(UART0_RE_PIN, 1);
}

/**
** Enable TX on transceiver.
*/
void modbus_tx_enable() {
  gpio_put(UART0_RE_PIN, 1);
  gpio_put(UART0_DE_PIN, 1);
}
uint8_t msb(uint16_t data)
{
  return (data >> 8) & 0xFF;
}
uint8_t lsb(uint16_t data)
{
  return data & 0xFF;
}


uint8_t tx_arr_crc(uart_inst_t* uartid, const uint8_t* ptr, int len) {
  crc_t crc = crc_init ();
  crc = crc_update (crc, ptr, len);
  crc = crc_finalize(crc);

#if 0
  for (int i = 0; i < len; i++) {
    printf("%x,", (int)ptr[i]);
  }
  printf("%x,", crc & 0xFF);
  printf("%x,", (crc >> 8) & 0xFF);
  printf("\r\n");
#endif

  uart_write_blocking(uartid, ptr, len);
  uart_putc(uartid, lsb(crc));
  uart_putc(uartid, msb(crc));
  return 0;
}

uint8_t readMultipleHoldingRegister (uart_inst_t* uartid, uint8_t slave, uint16_t address, uint16_t numreg)
{
  uint8_t func_code = 0x03;
  uint8_t req[] = {slave, func_code,
                   msb(address), lsb(address),
                   msb(numreg),  lsb(numreg)};
  tx_arr_crc (uartid, req, sizeof(req));
  return numreg * 2 + 5;
}
uint8_t writeSingleHoldingRegister (uart_inst_t* uartid, uint8_t slave, uint16_t address, uint16_t value)
{
  uint8_t func_code = 0x06;
  uint8_t req[] = {slave, func_code,
                   msb(address), lsb(address),
                   msb(value),  lsb(value)};
  tx_arr_crc (uartid, req, sizeof(req));
  return 2 * 2 + 5;
}
