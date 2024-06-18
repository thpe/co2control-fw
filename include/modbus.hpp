#pragma once

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
