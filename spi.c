#include <avr/io.h>
#include <avr/interrupt.h>
#include "spi.h"

#define PORT_SPI PORTB
#define DDR_SPI DDRB
#define DD_MISO DDB3
#define DD_MOSI DDB2
//#define DD_SS       DDB2
#define DD_SCK DDB1

void spi_init()
{
    DDR_SPI |= (1 << DD_MOSI) | (1 << DD_SCK);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0) | (1 << CPOL) | (1 << CPHA);
}

uint8_t spi_transfer(uint8_t value)
{
    SPDR = value;
    asm volatile("nop");
    while (!(SPSR & (1 << SPIF)));

    return SPDR;
}