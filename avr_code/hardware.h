#pragma once

#define F_CPU 20000000ull

#define BAUDRATE 115200

// port direction settings:
// 1 - out, 0 - in   v bits v
//                   76543210
#define DDRB_STATE 0b00001000
#define DDRC_STATE 0b00000000
#define DDRD_STATE 0b00000010

// pullup settings:
// 1 - on, 0 - off    v bits v
//                    76543210
#define PORTB_STATE 0b00000000
#define PORTD_STATE 0b01110000
