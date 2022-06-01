#ifndef _SPI_H_
#define _SPI_H_
#include <avr/io.h>

void spi_init();
uint8_t spi_transfer(uint8_t value);

#endif