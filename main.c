#define F_CPU 2000000L
// lf = 0xe2 (internal 2 MHz RC clock)
// hf = 0xd9

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define PIN_D_FORCE_ENABLE  PD2
#define PIN_D_PUMP_ENABLE   PD5
#define PIN_B_LED_1         PB0
#define PIN_D_LED_2         PD7
#define PIN_D_LED_STATUS    PD6
#define PIN_B_STEPPER_PULSE PB3

#define ADC_CHANNEL_PERIOD   0
#define ADC_CHANNEL_DURATION 1
#define ADC_CHANNEL_STEPPER  2

volatile uint16_t counter = 0;

void gpio_init()
{
    // Outputs
    DDRD |= _BV(PIN_D_PUMP_ENABLE) | _BV(PIN_D_LED_STATUS) | _BV(PIN_D_LED_2);
    DDRB |= _BV(PIN_B_LED_1) | _BV(PIN_B_STEPPER_PULSE);

    // Initial low level
    PORTD &= ~(_BV(PIN_D_PUMP_ENABLE) | _BV(PIN_D_LED_STATUS) | _BV(PIN_D_LED_2));
    PORTB &= ~_BV(PIN_B_LED_1);

    // Inputs
    DDRD &= ~_BV(PIN_D_FORCE_ENABLE);

    // Pull-ups
    PORTD |= _BV(PIN_D_FORCE_ENABLE);
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

    adc_init();

    // Main on-second counter
    TCCR1A = 0x03;
    TCCR1B = 0x1b;
    OCR1A = 31250;
    // Enable interrupt
    TIMSK = _BV(TOIE1);

    // Stepper motor step counter
    // CTC mode
    TCCR2 |= (1 << WGM21);
    // Toggle OC2 on compare match
    TCCR2 |= (1 << COM20);
    // Initial frequency: 512 Hz
    OCR2 = 60;

    sei();

    uint8_t first_run = 1;

    while (1) {
        // No force enable
        if (PIND & (1 << PIN_D_FORCE_ENABLE)) {
            // Clear output in case we had just exited 'else' condition
            PORTD &= ~_BV(PIN_D_PUMP_ENABLE);

            // Main period, ADC0
            uint16_t period = adc_read_channel(ADC_CHANNEL_PERIOD);
            // We convert 0 - 1023 range to 600 - 3669
            period = period * 3 + 600;

            // Duration of ON stat, ADC1
            uint16_t duration = adc_read_channel(ADC_CHANNEL_DURATION);
            // We convert 0 - 1023 range to 2 - 10,
            // and add 10 to eventualy reach 10 as a max.
            duration = ((duration + 10) >> 7) + 2;

            // Stepper motor step frequency, ADC2
            uint8_t step_freq = (adc_read_channel(ADC_CHANNEL_STEPPER) >> 2) & 0xff;
            step_freq = 255 - step_freq;
            // This way we get 122 - 1008 Hz range
            if (step_freq < 226) {
                OCR2 = step_freq + 30;
            } else {
                OCR2 = 0xff;
            }
            // Enable counter only when OCR2 is known, we use clk/32
            TCCR2 |= (1 << CS21) | (1 << CS20);

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
        } else {
            // Jumper/switch was set to force pump enable, so we do it
            PORTD |= _BV(PIN_D_PUMP_ENABLE);
        }
    }

    return 0;
}

ISR(TIMER1_OVF_vect)
{
    // Status blink
    PORTD |= _BV(PIN_D_LED_STATUS);
    _delay_ms(50);
    PORTD &= ~_BV(PIN_D_LED_STATUS);

    counter++;
}
