#include "hardware.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "simple_uart.h"
#include "bit_operations.h"
#include "bool.h"

#define microseconds(x) (((x) * F_CPU) / 1000000)

static inline void external_interrupt_init() {
    MCUCR = (1<<ISC00); // any change on INT1 causes interrupt
    GIMSK = (1<<INT0);
}

static inline void timer0_init() {
    TCCR0B = (1<<CS00);      // divide clock by 1
    TIMSK |= (1<<TOIE0);    // enable overflow interrupt
}

static inline void timer1_init() {
    TCCR1B = (1<<CS10);      // divide clock by 1
    TIMSK |= (1<<TOIE1);    // enable overflow interrupt
}

static inline void timer_reset() {
    TCNT0 = 0;    // magic number
}

static inline void init() {
    DDRB = DDRB_STATE;
    DDRD = DDRD_STATE;

    _delay_ms(500);

    setbit(PORTB, 0);

    uart_init();
    uart_enable_interrupt();

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

void process_data(uint8_t data) {
    uart_write_byte(data);
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
    TCNT1 = 65536 - microseconds(210);    // filter all short pulses
    have_state_change = true;
}

ISR(TIMER1_OVF_vect) {
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

ISR(USART_RX_vect) {
    char c = uart_read_byte();
    if (c == 'q') {
        uart_write_string("bye!\n");
        soft_reset();
    }
    uart_write_string("you said: ");
    uart_write_byte(c);
    uart_write_newline();
}

int main()
{
    init();
    for (;;) {
    }
    return 0;
}
