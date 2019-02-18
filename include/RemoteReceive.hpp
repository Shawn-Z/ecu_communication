// FUCK 207

#ifndef ECU_COMMUNICATION_REMOTERECEIVE_HPP
#define ECU_COMMUNICATION_REMOTERECEIVE_HPP

#include <cstdint>
#include <cstring>

#include "DataDownload.hpp"

namespace ecu_communication {

#pragma pack(1)
    union remote_receive_pack_one_type {
        struct {
            uint8_t ID_one;
            uint8_t ID_two;
            int32_t steer_level;
            int32_t throttle;
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
        uint8_t result_data[2];
    };

    union remote_receive__pack_two_type {
        struct {
            uint8_t ID_one;
            uint8_t ID_two;
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
                    uint8_t vehicle_start: 1;
                    uint8_t power_on: 1;
                    uint8_t reserve_bits2: 3;
                    uint8_t fix_two_chamber_valve: 1;
                    uint8_t tailgate_control: 2;
                };
                uint8_t functions_five;
            };
            uint8_t brake;
            uint8_t reserve_bytes1[3];
            uint8_t work_mode;
        };
        uint8_t result_data[2];
    };



#pragma pack()

    class RemoteReceive {
    public:
        remote_receive_pack_one_type pack_one;
        remote_receive__pack_two_type pack_two;

        uint8_t work_mode;

        shawn::handle pack_handle;

        bool receiveIDCheck(char *p_recv_raw_data, uint16_t p_recv_len);

    };

}

#endif //ECU_COMMUNICATION_REMOTERECEIVE_HPP