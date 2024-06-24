#pragma once

#define LED0_PIN 19
#define LED1_PIN 20
#define LED2_PIN 21
#define LED3_PIN 22


void led_set(uint led)
{
  gpio_put(led, 1);
}

void led_reset(uint led)
{
  gpio_put(led, 0);
}

void led_init(uint led)
{
  gpio_init(led);
  gpio_set_dir(led, GPIO_OUT);
}


uint led_toggle(uint led)
{
  uint val = gpio_get(led);
  val = (~val & 0x1);
  gpio_put(led, val);
  return val;
}
