#pragma once
#include <hardware/uart.h>


template< int ID > uart_inst_t* uart_id();
template<> uart_inst_t* uart_id<0> () {return uart0;}
template<> uart_inst_t* uart_id<1> () {return uart1;}

template< int ID > unsigned int uart_irq();
template<> unsigned int uart_irq<0> () {return UART0_IRQ;}
template<> unsigned int uart_irq<1> () {return UART1_IRQ;}
