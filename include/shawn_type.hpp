#ifndef ECU_COMMUNICATION_SHAWN_TYPE_HPP
#define ECU_COMMUNICATION_SHAWN_TYPE_HPP

#include <stdint-gcc.h>


namespace shawn {

//// data[0] is the low byte
//// data[1] is the high byte
//// for example if data[0] equals 0 and data[1] equals 1, the result will be 256
union bit_16_type {
    uint8_t data[2];
    uint16_t result;
};

union bit_32_type {
    uint8_t data[4];
    uint16_t result;
};

union {
    struct example1_type {
        uint8_t data1; //// low byte
        uint8_t data2; //// high byte
    } cell;
    uint16_t result;
} example1;

union {
    struct {
        uint8_t data1: 3; /// low bits (bit2 bit1 bit0)
        uint8_t data2: 5; /// high bits (bit7 bit6 bit5 bit4 bit3)
    }cell;
    uint8_t result;
} example2;

union {
    struct {
        uint16_t data; //// result[1] result[0]
    } cell;
    uint8_t result[2];
} example3;

union {
    struct {
        uint8_t data1[2];
        uint8_t data2[3];
    } cell;
    uint8_t result[5];  //// data1[0] data1[1] data2[0] data2[1] data2[2]
} example4;


}


#endif //ECU_COMMUNICATION_SHAWN_TYPE_HPP