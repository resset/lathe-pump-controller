#define F_CPU 2000000L
// lf = 0xe2 (internal 2 MHz RC clock)
// hf = 0xd9

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile uint16_t counter = 0;

void uart_init()
{
    #define BAUD 9600                                   // define baud
    #define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)            // set baud rate value for UBRR
    UBRRH = (BAUDRATE>>8);                      // shift the register right by 8 bits
    UBRRL = BAUDRATE;                           // set baud rate
    UCSRB|= (1<<TXEN)|(1<<RXEN);                // enable receiver and transmitter
    UCSRC|= (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);   // 8bit data format
}

void uart_send_byte_nl(uint8_t byte)
{
        while (!(UCSRA & (1 << UDRE)));
        UDR = byte;
        while (!(UCSRA & (1 << UDRE)));
        UDR = '\n';
}

void adc_init()
{
    ADCSRA |= _BV(ADEN);
}

uint16_t adc_read_channel(uint8_t channel)
{
    // Setup channel
    ADMUX = (1 << REFS0) | channel;

    // Start conversion
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);

    uint16_t value = 0;
    value = ADCL;
    value |= ADCH << 8;

    return value;
}

int main(void)
{
    DDRD |= _BV(PD5) | _BV(PD6) | _BV(PD7);
    DDRB |= _BV(PB6) | _BV(PB7) | _BV(PB0);

    PORTD &= ~(_BV(PD5) | _BV(PD6) | _BV(PD7));
    PORTB &= ~(_BV(PB6) | _BV(PB7) | _BV(PB0));

    adc_init();

    TCCR1A = 0x03;
    TCCR1B = 0x1b;
    OCR1A = 31250;

    TIMSK = _BV(TOIE1);

    uart_init();

    sei();

    uint8_t first_run = 1;

    while (1) {
        // Main period, ADC0
        uint16_t period = adc_read_channel(0);
        // We convert 0 - 1023 range to 600 - 3669
        period = period * 3 + 600;
        //uart_send_byte_nl(period & 0xff);
        //period = 20;

        // Duration of ON stat, ADC1
        uint16_t duration = adc_read_channel(1);
        // We convert 0 - 1023  range to 2 - 10,
        // add +10 to eventualy reach 10 as a max
        duration = ((duration + 10) >> 7) + 2;

        //uart_send_byte_nl(duration & 0xff);

        if (first_run == 1 || counter > period) {
            first_run = 0;

            cli();
            counter = 0;
            sei();

            PORTD |= _BV(PD5);

            // uart_send_byte_nl(duration & 0xff);

            uint8_t i = 0;
            while (i < duration) {
                _delay_ms(950);
                i++;
            }
            PORTD &= ~_BV(PD5);
        }
    }

    return 0;
}

ISR(TIMER1_OVF_vect)
{
    PORTB |= _BV(PB0);
    _delay_ms(50);
    PORTB &= ~_BV(PB0);

    counter++;
}
