#ifndef ECU_COMMUNICATION_DATAUPLOAD_HPP
#define ECU_COMMUNICATION_DATAUPLOAD_HPP

#include <cmath>
#include <stdint-gcc.h>
#include <cstring>
#include "three_one_msgs/Report.h"
#include "three_one_msgs/RawdataRecv.h"
#include "SHandle.hpp"
#include "ThreeOne.hpp"
#include "DEFINEs.hpp"

namespace ecu_communication {

#pragma pack(1)
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
        uint8_t vehicle_height;
        uint8_t park_status;
        uint8_t reserve_bytes1[2];
    };
    uint8_t pack[14];
};

union all_packs_type {
    struct {
        data_upload_pack_one_type pack_one;
        data_upload_pack_two_type pack_two;
        data_upload_pack_three_type pack_three;
        data_upload_pack_four_type pack_four;
        data_upload_pack_five_type pack_five;
        data_upload_pack_six_type pack_six;
        data_upload_pack_seven_type pack_seven;
    };
    uint8_t all[98];
};

#ifndef ID_CALCULATE
#define ID_CALCULATE
union ID_calculate_type {
    uint8_t data[4];
    uint32_t result;
};
#endif

#pragma pack()

class DataUpload {
public:
    data_upload_pack_one_type pack_one;
    data_upload_pack_two_type pack_two;
    data_upload_pack_three_type pack_three;
    data_upload_pack_four_type pack_four;
    data_upload_pack_five_type pack_five;
    data_upload_pack_six_type pack_six;
    data_upload_pack_seven_type pack_seven;

    uint8_t recv_raw_data[14];
    ID_calculate_type ID_calculate;
    uint64_t recv_counter;
    shawn::handle pack_handle;
    three_one_msgs::Report report;
    three_one_msgs::RawdataRecv recv_rawdata;


    DataUpload();
    void dataDistribution();
    bool dataIDCheck(char *p_recv_raw_data);
    bool dataCheck();
    void dataToMsg();


    uint8_t data_to_send[14];
    void prepareSend(shawn::handle p_handle);

    all_packs_type all_packs;
};

}
#endif //ECU_COMMUNICATION_DATAUPLOAD_HPP