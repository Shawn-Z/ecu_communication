#ifndef ECU_COMMUNICATION_DATAUPLOAD_HPP
#define ECU_COMMUNICATION_DATAUPLOAD_HPP

#include <cmath>
#include <stdint-gcc.h>
#include <cstring>

namespace ecu_communication {

union data_upload_pack_one_type {
    struct {
        uint32_t data_ID;
        uint8_t valid_data_mark;
        uint8_t valid_data_length;
        uint16_t left_wheel_expect_speed;
        uint16_t mechanical_brake;
        uint16_t right_wheel_expect_speed;
        uint8_t vehicle_speed;
        uint8_t gear;
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

union ID_calculate_type {
    uint8_t data[4];
    uint32_t result;
};

class DataUpload {
public:
    data_upload_pack_one_type data_upload_pack_one;
    data_upload_pack_two_type data_upload_pack_two;
    data_upload_pack_three_type data_upload_pack_three;
    data_upload_pack_four_type data_upload_pack_four;

    uint8_t recv_raw_data[14];
    ID_calculate_type ID_calculate;
    uint64_t recv_counter;

    DataUpload();
    void dataDistribution();
    int data_pack_num;
    int dataPackCheck(char * p_recv_raw_data);
};

}
#endif //ECU_COMMUNICATION_DATAUPLOAD_HPP