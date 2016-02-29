#pragma once

#include "bool.h"

#include <stdint.h>

uint16_t fletcher16(uint8_t data[], uint8_t count);
bool fletcher16_check(uint8_t data[], uint8_t size);
