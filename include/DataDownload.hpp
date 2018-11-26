#ifndef ECU_COMMUNICATION_DATADOWNLOAD_HPP
#define ECU_COMMUNICATION_DATADOWNLOAD_HPP

#include <cstring>
#include <cmath>
#include <stdint-gcc.h>

namespace ecu_communication {

union data_download_pack_one_type {
    struct {
        uint32_t data_ID;
        uint8_t valid_data_mark;
        uint8_t valid_data_length;
        uint16_t thousand_times_curvature;
        uint8_t expect_vehicle_speed;
        uint8_t expect_left_speed;
        uint8_t expect_right_speed;
        uint8_t work_mode;
        union {
            struct {
                uint8_t reserve_bits1: 3;
                uint8_t right_wheel_rotate: 1;
                uint8_t left_wheel_rotate: 1;
                uint8_t vehicle_turn_to: 1;
                uint8_t vehicle_gear: 2;
            };
            uint8_t functions_one;
        };
        union {
            struct {
                uint8_t turn_light: 3;
                uint8_t wide_taillight: 1;
                uint8_t forward_big_light: 2;
                uint8_t ring_control: 1;
                uint8_t parking_control: 1;
            };
            uint8_t functions_two;
        };
    };
    uint8_t result_data[14];
};

union data_download_pack_two_type {
    struct {
        uint32_t data_ID;
        uint8_t valid_data_mark;
        uint8_t valid_data_length;
        union {
            struct {
                uint8_t suspension_select: 4;
                uint8_t cylinder_select: 4;
            };
            uint8_t functions_three;
        };
        union {
            struct {
                uint8_t vertical_wall_mode: 2;
                uint8_t suspension_cylinder_motor_control: 1;
                uint8_t suspension_cylinder_select_mode: 1;
                uint8_t suspension_work_mode_detail: 2;
                uint8_t suspension_work_mode: 2;
            };
            uint8_t functions_four;
        };
        union {
            struct {
                uint8_t reserve_bits2: 6;
                uint8_t tailgate_control: 2;
            };
            uint8_t functions_five;
        };
        uint8_t reserve_bytes1[5];
    };
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