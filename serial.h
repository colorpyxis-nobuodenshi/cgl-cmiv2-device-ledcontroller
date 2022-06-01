#ifndef _SERIAL_H_
#define _SERIAL_H_
#include <avr/io.h>
void serial_init(int32_t baud);
void serial_putc(uint8_t value);
uint8_t serial_getc();
uint8_t serial_available();
#endif