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
    PORTB &= ~(_BV(PB6) | _BV(PB7) | _BV(PB0));

    ADCSRA |= _BV(ADEN);

    TCCR1A = 0x03;
    TCCR1B = 0x1b;
    OCR1A = 31250;

    TIMSK = _BV(TOIE1);

    sei();

    while (1) {
        // Main period, ADC0
        ADMUX = _BV(REFS0);
        ADCSRA |= _BV(ADSC);
        loop_until_bit_is_clear(ADCSRA, ADSC);
        uint16_t period = 0;
        period = ADCL;
        period |= ADCH << 8;
        // We convert 0 - 1023 range to 600 - 3669
        period = period * 3 + 600;

        // Duration of ON stat, ADC1
        ADMUX = _BV(REFS0) | _BV(ADLAR) | _BV(MUX1);
        ADCSRA |= _BV(ADSC);
        loop_until_bit_is_clear(ADCSRA, ADSC);
        uint8_t duration = 0;
        duration = ADCH;
        // We convert 0 - 255 range to 2 - 10
        duration = (duration >> 5) + 2;

        if (counter > period) {
            cli();
            counter = 0;
            sei();

            PORTD |= _BV(PD7);
            uint8_t i = 0;
            while (i < duration) {
                _delay_ms(1000);
                i++;
            }
            PORTD &= ~_BV(PD7);
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
