// FUCK 207

#ifndef ECU_COMMUNICATION_REMOTESEND_HPP
#define ECU_COMMUNICATION_REMOTESEND_HPP

#include <cstdint>

namespace ecu_communication {

#pragma pack(1)
    union pack_one_type {
        struct {
            uint16_t data_ID;
            uint8_t gear;
            uint8_t park_status;
            uint16_t left_wheel_expect_speed;
            uint16_t mechanical_brake;
            uint16_t right_wheel_expect_speed;
            uint8_t vehicle_speed;
        };
        uint8_t pack[14];
    };

    union data_upload_pack_two_type {
        struct {
            uint32_t data_ID;
            uint8_t valid_data_mark;
            uint8_t valid_data_length;
            uint16_t left_motor_actual_speed;
            uint8_t left_motor_gear;
            uint8_t right_motor_gear;
            uint16_t right_motor_actual_speed;
            uint8_t SOC;
            uint8_t tailgate_state;
        };
        uint8_t pack[14];
    };

    union data_upload_pack_three_type {
        struct {
            uint32_t data_ID;
            uint8_t valid_data_mark;
            uint8_t valid_data_length;
            uint8_t left_one_cylinder_position;
            uint8_t left_two_cylinder_position;
            uint8_t left_three_cylinder_position;
            uint8_t left_four_cylinder_position;
            uint8_t right_one_cylinder_position;
            uint8_t right_two_cylinder_position;
            uint8_t right_three_cylinder_position;
            uint8_t right_four_cylinder_position;
        };
        uint8_t pack[14];
    };

    union data_upload_pack_four_type {
        struct {
            uint32_t data_ID;
            uint8_t valid_data_mark;
            uint8_t valid_data_length;
            uint8_t left_one_cylinder_pressure;
            uint8_t left_two_cylinder_pressure;
            uint8_t left_three_cylinder_pressure;
            uint8_t left_four_cylinder_pressure;
            uint8_t right_one_cylinder_pressure;
            uint8_t right_two_cylinder_pressure;
            uint8_t right_three_cylinder_pressure;
            uint8_t right_four_cylinder_pressure;
        };
        uint8_t pack[14];
    };

    union data_upload_pack_five_type {
        struct {
            uint32_t data_ID;
            uint8_t valid_data_mark;
            uint8_t valid_data_length;
            uint32_t mileage;
            uint8_t vehicle_roll;
            uint8_t vehicle_pitch;
            int16_t left_torque;
        };
        uint8_t pack[14];
    };

    union data_upload_pack_six_type {
        struct {
            uint32_t data_ID;
            uint8_t valid_data_mark;
            uint8_t valid_data_length;
            int16_t right_torque;
            uint8_t vertical_wall_status;
            uint8_t error_code;
            uint32_t left_pulse;
        };
        uint8_t pack[14];
    };

    union data_upload_pack_seven_type {
        struct {
            uint32_t data_ID;
            uint8_t valid_data_mark;
            uint8_t valid_data_length;
            uint32_t right_pulse;
            uint8_t vehical_height;
            uint8_t reserve_bytes1[2];
        };
        uint8_t pack[14];
    };

    class RemoteSend {

    };

}

#endif //ECU_COMMUNICATION_REMOTESEND_HPP