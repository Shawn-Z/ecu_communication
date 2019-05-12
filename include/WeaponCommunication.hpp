#ifndef ECU_COMMUNICATION_WEAPONCOMMUNICATION_HPP
#define ECU_COMMUNICATION_WEAPONCOMMUNICATION_HPP

#include <mutex>
#include <ros/ros.h>
#include "UDPCommunication.hpp"
#include "WeaponSend.hpp"
#include "shawn/SLog.hpp"

namespace ecu_communication {

class WeaponCommunication {
public:
    bool send_switch_;
    shawn::SLog *p_log_;

    //// UDP communication data and variables
    UDPCommunication udp_;
    WeaponSend weapon_send_;

    DataUpload *p_data_upload_;
    std::mutex *p_data_upload_mutex_;
    sensor_driver_msgs::VehicleState *p_gps_;

    void init(DataUpload *p_data_upload,
              std::mutex *p_data_upload_mutex,
              shawn::SLog *p_log,
              sensor_driver_msgs::VehicleState *p_gps);
    void dataSend();
    void cmdSend();
};

}

#endif //ECU_COMMUNICATION_WEAPONCOMMUNICATION_HPP