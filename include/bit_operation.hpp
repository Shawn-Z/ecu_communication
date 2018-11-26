#ifndef BIT_OPRATION_HPP
#define BIT_OPRATION_HPP

#include <stdint-gcc.h>

namespace bit_operation {

uint8_t get_bit(uint8_t byte, uint8_t position) {
    return (uint8_t)((byte >> position) & 0b00000001);
};

void set_bit_to_one(uint8_t & byte, uint8_t position) {
    byte |= (1 << position);
}

void set_bit_to_zero(uint8_t & byte, uint8_t position) {
    byte &= (~(1 << position));
}

}
#endif //BIT_OPRATION_HPP