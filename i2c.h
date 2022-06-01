#ifndef _I2C_H_
#define _I2C_H_

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>

void i2c_init();
void i2c_start(uint8_t sla);
void i2c_stop();
void i2c_write(uint8_t value);
uint8_t i2c_read_nack();
uint8_t i2c_read_ack();

#endif