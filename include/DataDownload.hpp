#ifndef ECU_COMMUNICATION_DATADOWNLOAD_HPP
#define ECU_COMMUNICATION_DATADOWNLOAD_HPP

#include <cmath>
#include <stdint-gcc.h>

namespace ecu_communication {

union data_download_pack_one_type {
    struct {
        uint32_t data_ID;
        uint8_t valid_data_mark;
        uint8_t valid_data_length;
        uint8_t curvature_low_byte;
        uint8_t curvature_high_byte;
        uint8_t expect_vehicle_speed;
        uint8_t expect_left_speed;
        uint8_t expect_right_speed;
        uint8_t work_mode;
        union {
            struct {
                uint8_t reserve_bits: 3;
                uint8_t 
            };
            uint8_t functions_one;
        };
        uint8_t functions_one;
        uint8_t functions_two;
    } original_data;
    uint8_t result_data[14];
};

union data_download_pack_two_type {
    struct {
        uint32_t data_ID;
        uint8_t valid_data_mark;
        uint8_t valid_data_length;
        uint8_t functions_three;
        uint8_t functions_four;
        uint8_t functions_five;
        uint8_t reserve_byte1;
        uint8_t reserve_byte2;
        uint8_t reserve_byte3;
        uint8_t reserve_byte4;
        uint8_t reserve_byte5;
    } original_data;
    uint8_t result_data[14];
};

class DataDownload {
public:
    data_download_pack_one_type data_download_pack_one;
    data_download_pack_two_type data_download_pack_two;
    DataDownload();
    void init();
};

}
#endif //ECU_COMMUNICATION_DATADOWNLOAD_HPP