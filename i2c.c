#include "i2c.h"

void i2c_init()
{
    TWBR = 72;
    TWSR = 0;

    TWCR = (1 << TWEN);
}

void i2c_start(uint8_t sla)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    while ((TWSR & 0xF8) != TW_START);

    TWDR = sla;
    TWCR = (1 << TWINT) | (1 << TWEN);
    
    while ((!(TWCR & (1 << TWINT))));
    while ((TWSR & 0xF8) != TW_MT_SLA_ACK);
    
}
void i2c_stop()
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (!(TWCR & (1 << TWSTO)));
}
void i2c_write(uint8_t value)
{
    TWDR = value;
    TWCR = (1 << TWINT) | (1 << TWEN);
    
    while ((!(TWCR & (1 << TWINT))));
    while ((TWSR & 0xF8) != TW_MT_DATA_ACK);
}
uint8_t i2c_read_nack()
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    return TWDR;
}
uint8_t i2c_read_ack()
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));

    return TWDR;
}