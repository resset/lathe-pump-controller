#define F_CPU 2000000L
// lf = 0xe2 (internal 2 MHz RC clock)
// hf = 0xd9

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define PIN_D_FORCE_ENABLE  PD2
#define PIN_D_PUMP_ENABLE   PD5
#define PIN_B_LED_STATUS    PB0
#define PIN_D_LED_2         PD7
#define PIN_D_LED_3         PD6
#define PIN_B_STEPPER_PULSE PB3

#define ADC_CHANNEL_PERIOD   0
#define ADC_CHANNEL_DURATION 1
#define ADC_CHANNEL_STEPPER  2

volatile uint16_t counter = 0;

void gpio_init()
{
    // Outputs
    DDRD |= _BV(PIN_D_PUMP_ENABLE) | _BV(PIN_D_LED_3) | _BV(PIN_D_LED_2);
    DDRB |= _BV(PIN_B_LED_STATUS);

    // Initial low level
    PORTD &= ~(_BV(PIN_D_PUMP_ENABLE) | _BV(PIN_D_LED_3) | _BV(PIN_D_LED_2));
    PORTB &= ~_BV(PIN_B_LED_STATUS);
}

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
    gpio_init();

    uart_init();

    adc_init();

    TCCR1A = 0x03;
    TCCR1B = 0x1b;
    OCR1A = 31250;

    TIMSK = _BV(TOIE1);

    sei();

    uint8_t first_run = 1;

    while (1) {
        // Main period, ADC0
        uint16_t period = adc_read_channel(ADC_CHANNEL_PERIOD);
        // We convert 0 - 1023 range to 600 - 3669
        period = period * 3 + 600;
        //period = 20;

        // Duration of ON stat, ADC1
        uint16_t duration = adc_read_channel(ADC_CHANNEL_DURATION);
        // We convert 0 - 1023  range to 2 - 10,
        // add +10 to eventualy reach 10 as a max
        duration = ((duration + 10) >> 7) + 2;

        if (first_run == 1 || counter > period) {
            first_run = 0;

            cli();
            counter = 0;
            sei();

            PORTD |= _BV(PIN_D_PUMP_ENABLE);

            uint8_t i = 0;
            while (i < duration) {
                _delay_ms(950);
                i++;
            }
            PORTD &= ~_BV(PIN_D_PUMP_ENABLE);
        }
    }

    return 0;
}

ISR(TIMER1_OVF_vect)
{
    PORTB |= _BV(PIN_B_LED_STATUS);
    _delay_ms(50);
    PORTB &= ~_BV(PIN_B_LED_STATUS);

    counter++;
}
