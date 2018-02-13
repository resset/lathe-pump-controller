#define F_CPU 2000000L
// lf = 0xe2 (internal 2 MHz RC clock)
// hf = 0xd9

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile uint16_t counter = 0;

int main(void)
{
    DDRD |= _BV(PD5) | _BV(PD6) | _BV(PD7);
    DDRB |= _BV(PB6) | _BV(PB7) | _BV(PB0);

    PORTD &= ~(_BV(PD5) | _BV(PD6) | _BV(PD7));
    PORTB &= ~(_BV(PB6) | _BV(PB7));
    PORTB |= _BV(PB0);

    ADCSRA |= _BV(ADEN);
    ADMUX |= _BV(REFS0) | _BV(MUX2);

    TCCR1A = 0x03;
    TCCR1B = 0x1b;
    OCR1A = 31250;

    TIMSK = _BV(TOIE1);

    sei();

    while (1) {
        ADCSRA |= _BV(ADSC);
        loop_until_bit_is_clear(ADCSRA, ADSC);
        uint16_t a = 0;
        a = ADCL;
        a |= ADCH << 8;
        a += 4;

        if (counter > a) {
            cli();
            counter = 0;
            sei();

            PORTD |= _BV(PD7);
            _delay_ms(500);
            PORTD &= ~_BV(PD7);
        }
    }

    /*TCCR1A = 0xa0;
    TCCR1B = 0x12;
    ICR1 = 10230;
    OCR1A = 1000;

    while (1) {
        ADCSRA |= _BV(ADSC);
        loop_until_bit_is_clear(ADCSRA, ADSC);
        uint16_t a = 0;
        a = ADCL;
        a |= ADCH << 8;

        OCR1A = a * 10;
    }*/

    return 0;
}

ISR(TIMER1_OVF_vect)
{
    PORTB = PORTB ^ _BV(PB0);

    counter++;
}
