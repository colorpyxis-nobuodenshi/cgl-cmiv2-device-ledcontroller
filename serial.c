#include <avr/io.h>
#include <avr/interrupt.h>

#define SERIAL_RX_BUFFER_SIZE 32
#define SERIAL_TX_BUFFER_SIZE 32

volatile uint8_t rx[SERIAL_RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;
volatile uint8_t tx[SERIAL_TX_BUFFER_SIZE];
volatile uint8_t tx_head = 0;
volatile uint8_t tx_tail = 0;

void serial_init(int32_t baud)
{
    uint16_t b = (F_CPU / baud / 16) - 1;
    UBRR1H = b >> 8;
    UBRR1L = b;

    UCSR1C = (3 << UCSZ10);
    UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);
}

void serial_putc(uint8_t value)
{
    while (!(UCSR1A & (1 << UDRE1)));
    UDR1 = value;
}

void serial_put_string(char *values)
{
    while (*values)
    {
        serial_putc(*values++);
    }
}

uint8_t serial_getc()
{
    //while((UCSR0A & (1<<RXC0)) == 0);
    //return UDR0;

    UCSR1B &= ~(1 << RXCIE1);

    if (rx_head == rx_tail)
    {
        UCSR1B |= (1 << RXCIE1);    
        return -1;
    }
    uint8_t c = rx[rx_tail];
    rx_tail = (rx_tail + 1) % SERIAL_RX_BUFFER_SIZE;

    UCSR1B |= (1 << RXCIE1);

    return c;
}

uint8_t serial_available()
{
    return (SERIAL_RX_BUFFER_SIZE + rx_head - rx_tail) % SERIAL_RX_BUFFER_SIZE;
}

ISR(USART1_RX_vect)
{
    if (bit_is_clear(UCSR1A, UPE1) && bit_is_clear(UCSR1A, FE1) && bit_is_clear(UCSR1A, DOR1))
    {
        uint8_t c = UDR1;
        uint8_t i = (rx_head + 1) % SERIAL_RX_BUFFER_SIZE;
        if (i != rx_tail)
        {
            rx[rx_tail] = c;
            rx_head = i;
        }
    }
    else
    {
        UDR1;
    }
}

// ISR (USART0_UDRE_vect)
// {

// 	UDR0 = tx[tx_head];
// 	tx_head = (tx_head + 1) % SERIAL_TX_BUFFER_SIZE;
// 	if ( tx_head == tx_tail ) {
// 		UCSR0B &= ~(1<<UDRIE0);
// 	}
// }
