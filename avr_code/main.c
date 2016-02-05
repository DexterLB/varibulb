#include "hardware.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "bit_operations.h"


static inline void init(void)
// main init
{
    DDRB = DDRB_STATE;
    DDRD = DDRD_STATE;

	// sei();
}


int main(void)
{
    init();

    for (;;) {
    }
    return 0;
}
