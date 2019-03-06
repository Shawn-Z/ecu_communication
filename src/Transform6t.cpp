#include "Transform6t.hpp"

namespace ecu_communication {

    bool Transform6t::receiveCheck(char *p_recv_raw_data) {
        memcpy(this->receive_6t.pack, p_recv_raw_data, sizeof(this->receive_6t.pack));
        bool ID_check = true;
        ID_check &= (this->receive_6t.pack[0] == 0x08);
        ID_check &= (this->receive_6t.pack[1] == 0x00);
        ID_check &= (this->receive_6t.pack[2] == 0x00);
        ID_check &= (this->receive_6t.pack[3] == 0x04);
        ID_check &= (this->receive_6t.pack[4] == 0x20);
        return ID_check;
    }

    bool Transform6t::prepareSend(ecu_communication::DataDownload *p_data_download) {
        this->send_6t.pack[0] = 0x08;
        this->send_6t.pack[1] = 0x00;
        this->send_6t.pack[2] = 0x00;
        this->send_6t.pack[3] = 0x06;
        this->send_6t.pack[4] = 0x60;
        this->send_6t.pack[5] = 0x00;
        double_t tmp_speed = p_data_download->pack_one.expect_vehicle_speed / 10.0 * 3.6 * 1000;
        if (tmp_speed > 40000) {
            tmp_speed = 40000;
        }
        this->send_6t.vehicle_speed = (uint16_t)round(tmp_speed);
        if ((p_data_download->pack_one.vehicle_gear == 1) && (p_data_download->pack_one.vehicle_turn_to == 0)) {
            this->send_6t.steer_direction = 3;
        }
        if ((p_data_download->pack_one.vehicle_gear == 1) && (p_data_download->pack_one.vehicle_turn_to == 1)) {
            this->send_6t.steer_direction = 1;
        }
        if ((p_data_download->pack_one.vehicle_gear == 2) && (p_data_download->pack_one.vehicle_turn_to == 0)) {
            this->send_6t.steer_direction = 15;
        }
        if ((p_data_download->pack_one.vehicle_gear == 2) && (p_data_download->pack_one.vehicle_turn_to == 1)) {
            this->send_6t.steer_direction = 7;
        }
        double_t tmp_b = 2.7;
        this->send_6t.steer_level = (uint8_t)round(200.0 * tmp_b * p_data_download->pack_one.thousand_times_curvature / 1000.0 / (2.0 + tmp_b * p_data_download->pack_one.thousand_times_curvature / 1000.0));
        if ((p_data_download->pack_one.vehicle_gear == 0) || (p_data_download->pack_one.work_mode != 1)) {
            this->send_6t.vehicle_speed = 0;
            this->send_6t.steer_level = 0;
        }
    }
}