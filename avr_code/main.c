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
    // enable OC1A pin
    TCCR1A = (1<<COM1A1);

    // divide clock by 64, enable input noise canceller, capture on 0->1
    TCCR1B = (1<<CS10) | (1 << CS11) | (1 << ICNC1) | (1 << ICES1);

    // enable compare interrupt
    TIMSK |= (1<<ICIE1) | (1<<OCIE1A);

    // start the comparator and use it as capture source for the timer
    ACSR = (1<<ACIC);

    // disable digital buffers on the comparator pins
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

#define min_weakness 1500

uint16_t weakness = 65535;
uint16_t target_weakness = 65535;
uint16_t fade_speed = 65535;

uint8_t state = 0;
uint16_t last_transition;
uint16_t times[4];

ISR(TIMER1_CAPT_vect) {
    uint16_t transition_time = ICR1;

    uint16_t period_length = transition_time - last_transition;
    if (period_length < microseconds(15000)) {  // (1/50Hz) * (2/3)
        // this is a bogus transition - maybe noise or jitter from the triac
        // turning on
        return;
    }

    last_transition = transition_time;

    if (weakness < target_weakness) {
        if (target_weakness - weakness < fade_speed) {
            weakness = target_weakness;
        } else {
            weakness += fade_speed;
        }
    }
    if (weakness > target_weakness) {
        if (weakness - target_weakness < fade_speed) {
            weakness = target_weakness;
        } else {
            weakness -= fade_speed;
        }
    }

    uint16_t transition_period = ((uint32_t)weakness * period_length) >> 17;
    // 17 because we divide by 2^16 to normalize weakness (0-65536)
    // and then divide by 2 to get the halfperiod length
    
    if ((period_length >> 1) - transition_period < microseconds(500)) {
        state = 4;  // off
        return;
    }

    // start a pulse after transition_period
    times[0] = last_transition + transition_period;

    // end the pulse after 100us
    times[1] = times[0] + microseconds(100);

    // start another pulse a halfperiod after the first one
    times[2] = times[0] + (period_length >> 1);   // halfperiod length

    // end the second pulse after 100us
    times[3] = times[2] + microseconds(100);

    OCR1A = times[0];

    // output will be set on next state entry (for the first pulse)
    setbit(TCCR1A, COM1A0);

    state = 0;
}

ISR(TIMER1_COMPA_vect) {
    state++;
    if (state > 3) {
        return;
    }
    OCR1A = times[state];

    // at even states we start a pulse, at odd states we end the pulse
    // therefore at even states we make the next match end the pulse (go low)
    // and at odd states we make the next match start a pulse (go high)
    setbitval(TCCR1A, COM1A0, !(state & 1))
}

void update_weakness(uint16_t new_target) {
    if (new_target > min_weakness) {
        target_weakness = new_target;
    } else {
        target_weakness = min_weakness;
    }
}

void offset_weakness(int16_t offset) {
    if (offset > 0 && target_weakness + offset < target_weakness) {
        target_weakness = 65535;
        return;
    }
    if (offset < 0 && target_weakness + offset > target_weakness) {
        target_weakness = min_weakness;
        return;
    }
    update_weakness(target_weakness + offset);
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
    uint8_t encoder = 0;
    bool button_was_pressed = false; 
    uint16_t saved_weakness = 0;

    for (;;) {
        if (bitset(PIND, 6)) {
            button_was_pressed = true;
        } else if (button_was_pressed) {
            button_was_pressed = false;
            fade_speed = 3000;
            if (target_weakness == 65535) {
                update_weakness(saved_weakness);
            } else {
                saved_weakness = target_weakness;
                update_weakness(65535);
            }
        }


        encoder = ((encoder << 2) | ((PIND >> 4) & 0b11)) & 0b1111;

        // bits 0 and 1 are the current state of the encoder
        // bits 2 and 3 are the previous state of the encoder
        /*  correct encoder table
        switch(encoder) {
            case 0b0001:
            case 0b0111:
            case 0b1110:
            case 0b1000:
                fade_speed = 1000;
                offset_weakness(1000);
                break;
            case 0b1011:
            case 0b1101:
            case 0b0100:
            case 0b0010:
                fade_speed = 1000;
                offset_weakness(-1000);
                break;
        }
        */
        /* incorrect proteus encoder table
            */
        switch(encoder) {
            case 0b1000:
            case 0b0001:
            case 0b0111:
            case 0b1110:
                fade_speed = 1000;
                offset_weakness(1000);
                break;
            case 0b0100:
            case 0b0010:
            case 0b1011:
            case 0b1101:
                fade_speed = 1000;
                offset_weakness(-1000);
                break;
        }
        /**/
    }
    return 0;
}
