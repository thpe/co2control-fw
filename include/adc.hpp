#pragma once

#define ADC_CELL 0
#define ADC_SYS 3

/**
** Return ADC value in Volt
**
** \return a float containing the ADC value.
*/
float adc_read_V(int ch)
{
   const float conversion_factor = 3.3f / (1 << 12) * 2.0;
   adc_select_input(ch);
   uint16_t result = adc_read();
   return result * conversion_factor;
}
