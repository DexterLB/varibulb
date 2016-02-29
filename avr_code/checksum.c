#include "checksum.h"

uint16_t fletcher16(uint8_t data[], uint8_t count) {
    // https://en.wikipedia.org/wiki/Fletcher%27s_checksum#Implementation
 
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;

    for (uint8_t i = 0; i < count; i++) {
        sum1 = (sum1 + data[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }

    return (sum2 << 8) | sum1;
}

bool fletcher16_check(uint8_t data[], uint8_t size) {
    uint16_t data_checksum = data[size - 1] & ((uint16_t)(data[size - 2]) << 8);
    uint16_t actual_checksum = fletcher16(data, size - 2);
    return (data_checksum == actual_checksum);
}
