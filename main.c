#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <util/delay.h>
#include <math.h>

#include "spi.h"
#include "i2c.h"
#include "serial.h"
#include "VirtualSerial.h"

#define SSNPORT PORTB
#define LDACPORT PORTD
#define SSN1 PB0
#define SSN2 PB4
#define SSN3 PB5
#define SSN4 PB6
#define SSN5 PB7
#define LDAC1 PD6
#define LDAC2 PD7
#define TEMPERATURE1 0//PF0
#define TEMPERATURE2 1//PF1
#define LDMON1 4//PF4
#define LDMON2 5//PF5

#define digital_out_high(port, pin){ port |= (1<<pin);}
#define digital_out_low(port, pin){ port &= ~(1<<pin);}

static uint16_t EEMEM EEPROM_V1 = 600;
static uint16_t EEMEM EEPROM_V2 = 1000;
static uint16_t EEMEM EEPROM_V10 = 600;
static uint16_t EEMEM EEPROM_V11 = 1000;

#define CDC_BUFFER_SIZE 32
char cdc_recive_index = 0;
bool cdc_recive_terminator = false;
char recvBuffer[CDC_BUFFER_SIZE];

float temperature1 = 0.0;
float temperature2 = 0.0;
long long opticalpower1 = 0;
long long opticalpower2 = 0;
long long opticalpower3 = 0;
static uint16_t ledvalue1 = 0;
static uint16_t ledvalue2 = 0;
int integ = 0;

void init_mcp4911()
{

}

void update_mcp4911_1(int value)
{
    cli();

    digital_out_low(SSNPORT, SSN1);
    _delay_us(100);

    uint8_t m = ((value >> 6) & 0xFF)|(0x7<<4);
    spi_transfer(m);
    m = (value << 2) & 0xFF;
    spi_transfer(m);
    digital_out_high(SSNPORT, SSN1);
    _delay_us(100);

    // digital_out_low(LDACPORT, LDAC1);
    // _delay_ms(1);

    // digital_out_high(LDACPORT, LDAC1);
    // _delay_us(100);

    sei();
}

void update_mcp4911_2(int value)
{
    cli();

    digital_out_low(SSNPORT, SSN2);
    _delay_us(100);

    uint8_t m = ((value >> 6) & 0xFF)|(0x7<<4);
    spi_transfer(m);
    m = (value << 2) & 0xFF;
    spi_transfer(m);
    digital_out_high(SSNPORT, SSN2);
    _delay_us(100);

    // digital_out_low(LDACPORT, LDAC2);
    // _delay_ms(1);

    // digital_out_high(LDACPORT, LDAC2);
    // _delay_us(100);

    sei();
}

uint16_t read_ltc2452_1()
{
    cli();

    uint16_t d = 0;

    digital_out_low(SSNPORT, SSN3);
    _delay_us(100);

    d = spi_transfer(0) << 8;
    d |= spi_transfer(0);
    digital_out_high(SSNPORT, SSN3);
    _delay_us(100);

    sei();

    return d - 32768;
}

uint16_t read_ltc2452_2()
{
    cli();

    uint16_t d = 0;

    digital_out_low(SSNPORT, SSN4);
    _delay_us(100);

    d = spi_transfer(0) << 8;
    d |= spi_transfer(0);
    digital_out_high(SSNPORT, SSN4);
    _delay_us(100);

    sei();

    return d - 32768;
}

uint16_t read_ltc2452_3()
{
    cli();

    uint16_t d = 0;

    digital_out_low(SSNPORT, SSN5);
    _delay_us(100);

    d = spi_transfer(0) << 8;
    d |= spi_transfer(0);
    digital_out_high(SSNPORT, SSN5);
    _delay_us(100);

    sei();

    return d - 32768;
}

uint16_t read_temperature(int channel)
{
    ADMUX = 0b01000000;
    ADMUX |= (channel & 0x1F);
    ADCSRB = 0b00000000;
    ADCSRB |= (((channel >> 5) & 1) << MUX5);
    ADCSRA = (1 << ADEN) | (1 << ADSC) | 7;

    while ((ADCSRA & (1<<ADSC)));

    return (ADCL | (ADCH << 8)) & 0x3FF; 
    //return ADCH;
}

float get_temperature(uint16_t value)
{
    float x = value / 1023.0 * 5.0;
    float temp = 1.8781 * x * x * x -12.192 * x * x + 48.312 * x -34.966;

    if(temp > 80)
        temp = 80;
    if(temp < -10)
        temp = -10;

    return temp; 
}
float read_led_current(int channel)
{
    ADMUX = 0b01100000;
    ADMUX |= (channel & 0x1F);
    ADCSRB = 0b00000000;
    ADCSRB |= (((channel >> 5) & 1) << MUX5);
    ADCSRA = (1 << ADEN) | (1 << ADSC) | 7;

    while ((ADCSRA & (1<<ADSC)));

    return ADCL | (ADCH << 8); 
    //return ADCH;
}

USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber   = INTERFACE_ID_CDC_CCI,
				.DataINEndpoint           =
					{
						.Address          = CDC_TX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.DataOUTEndpoint =
					{
						.Address          = CDC_RX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.NotificationEndpoint =
					{
						.Address          = CDC_NOTIFICATION_EPADDR,
						.Size             = CDC_NOTIFICATION_EPSIZE,
						.Banks            = 1,
					},
			},
        .State =
            {
                .LineEncoding = 
                    {
                        .BaudRateBPS = 9600,
                        .CharFormat = CDC_LINEENCODING_OneStopBit,
                        .DataBits = 8,
                        .ParityType = CDC_PARITY_None,
                    }
            }
	};

static FILE USBSerialStream;

void EVENT_USB_Device_Connect(void)
{
	
}

void EVENT_USB_Device_Disconnect(void)
{
	
}

void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	
}

void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo)
{
	//bool HostReady = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR) != 0;
}

void update()
{

    
}

void print()
{
    char msg[64] = {0};
    sprintf(msg, "T1:%.3f T2:%.3f P1:%u P2:%u P3:%u L1:%d L2:%d\n", 
        get_temperature(read_temperature(0)), get_temperature(read_temperature(1)), 
        read_ltc2452_1(), read_ltc2452_2(), read_ltc2452_3(), ledvalue1, ledvalue2);
    // sprintf(msg, "T1:%.3f T2:%.3f P1:%u P2:%u P3:%u L1:%d L2:%d\n", 
    //     temperature1, temperature2, opticalpower1, opticalpower2, opticalpower3,
    //      ledvalue1, ledvalue2);
    char *m = msg;
    while (*m != '\0')
    {
        CDC_Device_SendByte(&VirtualSerial_CDC_Interface, *m);
        m++;
    }
}

void led1_update(int v)
{
    if(v >= 0 && v <= 1023)
    {
        update_mcp4911_1(v);
        if(v == 0)
        {
            digital_out_low(LDACPORT, LDAC1);
        }
        else
        {
            digital_out_high(LDACPORT, LDAC1);
        }
    }
}

void led2_update(int v)
{
    if(v >= 0 && v <= 1023)
    {
        update_mcp4911_2(v);
        if(v == 0)
        {
            digital_out_low(LDACPORT, LDAC2);
        }
        else
        {
            digital_out_high(LDACPORT, LDAC2);
        }
    }
}

void led_default()
{
    eeprom_busy_wait();
    uint16_t value1 = eeprom_read_word(&EEPROM_V10);
    eeprom_busy_wait();
    uint16_t value2 = eeprom_read_word(&EEPROM_V11);

    eeprom_busy_wait();
    eeprom_write_word(&EEPROM_V1, value1);
    eeprom_busy_wait();
    eeprom_write_word(&EEPROM_V2, value2);

    led1_update(value1);
    led2_update(value2);

    ledvalue1 = value1;
    ledvalue2 = value2;
}

void update_status()
{
    temperature1 += get_temperature(read_temperature(0));
    temperature2 += get_temperature(read_temperature(1));
    opticalpower1 += read_ltc2452_1();
    opticalpower2 += read_ltc2452_1();
    opticalpower3 += read_ltc2452_1();
    ledvalue1 = eeprom_read_word(&EEPROM_V1);
    ledvalue2 = eeprom_read_word(&EEPROM_V2);

    integ++;
    if(integ == 8)
    {
        temperature1 /= 8;
        temperature2 /= 8;
        opticalpower1 /= 8;
        opticalpower2 /= 8;
        opticalpower3 /= 8;
    }
}
void CDC_Recive_Event()
{
    int8_t d = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
    if(d > 0)
    {
        recvBuffer[cdc_recive_index] = d;
        cdc_recive_index++;
        if(d == '\n')
        {
            cdc_recive_terminator = true;
        }
    }
}

void CDC_Recive_Event_Process()
{
    if(cdc_recive_terminator)
    {
        cdc_recive_terminator = false;
        char* message = recvBuffer;
        
        if(strncmp(message,"L1", 2) == 0)
        {
            
            int v = 0;
            sscanf(message,"L1/%d\n", &v);
            led1_update(v);
            ledvalue1 = v;
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'A');
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'C');
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'K');
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, '\n');
        }
        else if(strncmp(message,"L2", 2) == 0)
        {
            int v = 0;
            sscanf(message,"L2/%d\n", &v);
            led2_update(v);
            ledvalue2 = v;
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'A');
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'C');
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'K');
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, '\n');
        }
        else if(strncmp(message,"ST", 2) == 0)
        {
            print();
        }
        else if(strncmp(message,"LD", 2) == 0)
        {
            led_default();

            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'A');
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'C');
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'K');
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, '\n');
        }
        else if(strncmp(message,"LS", 1) == 0)
        {
            uint16_t v1 = 0;
            uint16_t v2 = 0;
            sscanf(message,"LS/%d,%d\n", &v1, &v2);
            if(v1 >= 0 && v1 <= 1023)
            {
                eeprom_busy_wait();
                eeprom_write_word(&EEPROM_V1, v1);

                ledvalue1 = eeprom_read_word(&EEPROM_V1) & 0x3FF;

            }
            if(v2 >= 0 && v2 <= 1023)
            {
                eeprom_busy_wait();
                eeprom_write_word(&EEPROM_V2,  v2);   

                ledvalue2 = eeprom_read_word(&EEPROM_V2) & 0x3FF;
            }
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'A');
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'C');
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'K');
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, '\n');
        }
        else
        {
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'N');
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'A');
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'K');
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, '\n');
        }
        
        memset(recvBuffer,0,CDC_BUFFER_SIZE);
        cdc_recive_index = 0;
    }
}

void SetupHardware(void)
{
    MCUSR &= ~(1 << WDRF);
	wdt_disable();
	clock_prescale_set(clock_div_1);


    DDRB |= 0b11110001;
    DDRD |= 0b11000000;
    PORTB |= 0b11110001;
    PORTD |= 0b11000000;

    spi_init();
    serial_init(9600);
    
    eeprom_busy_wait();
    uint16_t value1 = eeprom_read_word(&EEPROM_V1) & 0x3FF;
    eeprom_busy_wait();
    uint16_t value2 = eeprom_read_word(&EEPROM_V2) & 0x3FF;

    // value1 = 600;
    // value2 = 1000;
    ledvalue1 = value1;
    ledvalue2 = value2;
    led1_update(value1);
    led2_update(value2);

    USB_Init();

}

int main(void)
{
	SetupHardware();

	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

	GlobalInterruptEnable();

	for (;;)
	{ 
        //update_status();

        CDC_Recive_Event_Process();

        //print();
        //CDC_Device_SendByte(&VirtualSerial_CDC_Interface, 'A');

        CDC_Recive_Event();
        
        //Delay_MS(100);

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();

	}
}

