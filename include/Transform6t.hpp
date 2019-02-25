#ifndef ECU_COMMUNICATION_TRANSFORM6T_HPP
#define ECU_COMMUNICATION_TRANSFORM6T_HPP

#include <tiff.h>
#include <cstdint>
#include <cstring>
#include "DataDownload.hpp"

namespace ecu_communication {

#pragma pack(1)
union send_6t_type {
    struct {
        uint8_t ID[5];
        uint8_t reserve_byte1;
        uint16_t vehicle_speed;
        uint8_t steer_level;
        uint8_t steer_direction;
        uint8_t reserve_bytes2[3];
    };
    uint8_t pack[13];
};

union receive_6t_type {
    struct {
        uint8_t ID[5];
        uint16_t left_rpm;
        uint16_t right_rpm;
        uint16_t left_torque;
        uint16_t right_torque;
    };
    uint8_t pack[13];
};
#pragma pack()

class Transform6t {
public:
    receive_6t_type receive_6t;
    send_6t_type send_6t;

    bool receiveCheck(char *p_recv_raw_data);
    bool prepareSend(DataDownload *p_data_download);
};

}

#endif //ECU_COMMUNICATION_TRANSFORM6T_HPP