#pragma once

#define F_CPU 14745600ull

#define BAUDRATE 115200

// 1 - out, 0 - in   v bits v
//                   76543210
#define DDRB_STATE 0b00000001
#define DDRC_STATE 0b00000000
#define DDRD_STATE 0b00000010
