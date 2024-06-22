/**
** @file
** @author Thomas Petig
** @section DESCRIPTION
** MODBUS interface
*/
#pragma once

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
