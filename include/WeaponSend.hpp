#ifndef ECU_COMMUNICATION_WEAPONSEND_HPP
#define ECU_COMMUNICATION_WEAPONSEND_HPP

#include "RemoteSend.hpp"

#pragma pack(1)
union weapon_send_pack_control_type {
    struct {
        uint8_t ID_one;
        uint8_t ID_two;
        uint8_t cmd;
        uint8_t ID_three;
        uint8_t ID_four;
    };
    uint8_t pack[5];
};
#pragma pack()

namespace ecu_communication {
    class WeaponSend {
    public:
        WeaponSend();

        remote_send_pack_one_type pack_one;
        weapon_send_pack_control_type pack_control;
        shawn::handle pack_handle;
        uint8_t data_to_send[2048];

        size_t prepareSend(DataUpload *p_data_upload, sensor_driver_msgs::VehicleState gps);
    };
}

#endif //ECU_COMMUNICATION_WEAPONSEND_HPP