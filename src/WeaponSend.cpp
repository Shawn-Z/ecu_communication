#include "WeaponSend.hpp"
namespace ecu_communication {
size_t WeaponSend::prepareSend(DataUpload *p_data_upload, sensor_driver_msgs::VehicleState gps) {
    size_t send_len = 0;
    switch (this->pack_handle.getID()) {
        case 1: {
            this->pack_one.ID_one = 0xF1;
            this->pack_one.ID_two = 0x00;
            this->pack_one.gear = p_data_upload->pack_one.gear;
            this->pack_one.park_status = p_data_upload->pack_seven.park_status;
            this->pack_one.mechanical_brake = p_data_upload->pack_one.mechanical_brake;
            this->pack_one.vehicle_speed = p_data_upload->pack_one.vehicle_speed;
            this->pack_one.mileage = p_data_upload->pack_five.mileage;
            this->pack_one.error_code = p_data_upload->pack_six.error_code;
            this->pack_one.left_wheel_expect_speed = p_data_upload->pack_one.left_wheel_expect_speed;
            this->pack_one.right_wheel_expect_speed = p_data_upload->pack_one.right_wheel_expect_speed;
            this->pack_one.vehicle_height = p_data_upload->pack_seven.vehicle_height;
            this->pack_one.latitude = gps.gps.latitude;
            this->pack_one.longitude = gps.gps.longitude;
            this->pack_one.altitude = gps.gps.altitude;
            this->pack_one.yaw = gps.yaw;
            this->pack_one.roll = gps.roll;
            this->pack_one.pitch = gps.pitch;
            this->pack_one.north_speed = gps.linear_velocity.y;
            this->pack_one.east_speed = gps.linear_velocity.x;
            this->pack_one.up_speed = gps.linear_velocity.z;
            this->pack_one.SOC = p_data_upload->pack_two.SOC;
            this->pack_one.reserve_byte1 = 0;
            send_len = sizeof(this->pack_one.pack);
            memcpy(this->data_to_send, this->pack_one.pack, send_len);
            break;
        }
        default: {
            break;
        }
    }
    return send_len;
}

WeaponSend::WeaponSend() {
    this->pack_control.ID_one = 0xE1;
    this->pack_control.ID_two = 0xF1;
    this->pack_control.ID_three = 0xE7;
    this->pack_control.ID_four = 0xF7;
}

}