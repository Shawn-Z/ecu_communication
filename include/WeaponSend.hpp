#ifndef ECU_COMMUNICATION_WEAPONSEND_HPP
#define ECU_COMMUNICATION_WEAPONSEND_HPP

#include "RemoteSend.hpp"

namespace ecu_communication {
    class WeaponSend {
    public:
        remote_send_pack_one_type pack_one;
        shawn::handle pack_handle;
        uint8_t data_to_send[2048];

        size_t prepareSend(DataUpload *p_data_upload, sensor_driver_msgs::VehicleState gps);
    };
}

#endif //ECU_COMMUNICATION_WEAPONSEND_HPP