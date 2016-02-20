#include "hardware.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "simple_uart.h"
#include "bit_operations.h"
#include "bool.h"

#define microseconds(x) (((x) * F_CPU) / (1000000 * 64))

static inline void external_interrupt_init() {
    MCUCR = (1<<ISC00); // any change on INT1 causes interrupt
    GIMSK = (1<<INT0);
}

static inline void timer0_init() {
    TCCR0B = (1<<CS00) | (1<<CS01);      // divide clock by 64
    TIMSK |= (1<<TOIE0) | (1<<OCIE0A);    // enable overflow interrupt
}

static inline void timer1_init() {
    TCCR1B = (1<<CS10) | (1 << CS11) | (1 << ICNC1) | (1 << ICES1);      // divide clock by 1
    TIMSK |= (1<<ICIE1) | (1<<OCIE1A);    // enable overflow interrupt
    ACSR = (1<<ACIC);
    DIDR = (1<<AIN1D) | (1<<AIN0D);
}

static inline void timer_reset() {
    TCNT0 = 0;    // magic number
}

static inline void init() {
    DDRB = DDRB_STATE;
    DDRD = DDRD_STATE;

    _delay_ms(500);

    external_interrupt_init();
    timer0_init();
    timer1_init();
    sei();
}

static inline void soft_reset()
{
    wdt_enable(WDTO_30MS);
    for(;;);
}

uint8_t state = 0;
uint16_t halfperiod_length;
uint16_t last_transition;
uint16_t transition_period;

ISR(TIMER1_CAPT_vect) {
    uint16_t transition_time = ICR1;
    clearbit(PORTB, 3);

    halfperiod_length = (transition_time - last_transition) >> 1;
    last_transition = transition_time;

    transition_period = halfperiod_length / 3;

    OCR1A = last_transition + transition_period;
    state = 1;
}

ISR(TIMER1_COMPA_vect) {
    if (state == 0) {
        return;
    }
    uint16_t pulse_end;
    if (state == 1 || state == 2) {   // first halfperiod
        uint16_t length = halfperiod_length * state;    // well, this is shit to explain.
        pulse_end = length - microseconds(1700);
        if (pulse_end > length) {
            // do a small pulse because we don't have a time for a big one
            // the triac won't like it, but it's the best we can do.
            pulse_end = length - microseconds(200);
            if (pulse_end > length) {
                state = 0;  // too late to do a pulse, giving up.
                return;
            }
        }
        OCR1A = last_transition + pulse_end;
        setbit(PORTB, 3);
        if (state == 1) {
            state = 3;
        } else {
            state = 4;
        }
        return;
    }
    if (state == 3) {   // end of trigger impulse
        OCR1A = last_transition + halfperiod_length + transition_period;
        state = 2;
        clearbit(PORTB, 3);
        return;
    }
    if (state == 4) {
        clearbit(PORTB, 3);
        state = 0;
    }
}


void process_data(uint8_t data) {
    setbitval(PORTD, 1, bitclear(PORTD, 1))
}

uint16_t pulse_rough_length = microseconds(5000);
uint8_t last_state = 0;
uint8_t received_bits = 9;
uint8_t data;
uint8_t have_state_change = false;

ISR(TIMER0_OVF_vect) {
    if (pulse_rough_length < microseconds(5000)) {
        pulse_rough_length += 256;
    } else if (received_bits < 8) {
        // send out incomplete bytes anyway: CRC hates wrong message lengths
        process_data(data);
        received_bits = 8;
    }
}

ISR(INT0_vect) {
    OCR0A = TCNT0 + microseconds(210);
    have_state_change = true;
}

ISR(TIMER0_COMPA_vect) {
    if (!have_state_change) {
        return;
    }
    have_state_change = false;

    uint16_t pulse_length = pulse_rough_length + TCNT0;
    uint8_t current_state = !!bitset(PIND, 2);

    if (last_state == current_state) {
        return;
    }

    // reset timer to start measuring the next pulse
    TCNT0 = 0;
    pulse_rough_length = 0;

    last_state = current_state;

    if (pulse_length > microseconds(2200)) {
        // skip long pulses
    } else if (pulse_length > microseconds(1600) && current_state == 0) {
        // start condition
        data = 0;
        received_bits = 0;
    } else {
        if (pulse_length > microseconds(600)) {
            // long pulse is a "1", short pulse is a "0"
            data |= (1 << received_bits);
        }

        if (received_bits <= 8) {
            received_bits++;
        }

        if (received_bits == 8) {
            process_data(data);
        }
    }
}

int main()
{
    init();
    for (;;) {
    }
    return 0;
}
