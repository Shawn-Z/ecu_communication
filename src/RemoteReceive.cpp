#include "RemoteReceive.hpp"


namespace ecu_communication {
    bool RemoteReceive::receiveIDCheck(char *p_recv_raw_data, uint16_t p_recv_len) {
        if ((((uint8_t)p_recv_raw_data[0]) == 0xF3) && (((uint8_t)p_recv_raw_data[1]) == 0x00)) {
            if (p_recv_len != sizeof(this->pack_one.result_data)) {
                return false;
            }
            pack_handle.setID(0);
//            memcpy(this->pack_one.result_data, p_recv_raw_data, sizeof(this->pack_one.result_data));

//            p_data_download_->pack_one.thousand_times_curvature = this->pack_one.thousand_times_curvature;
//            p_data_download_->pack_one.expect_vehicle_speed = this->pack_one.expect_vehicle_speed;
//            p_data_download_->pack_one.work_mode = this->pack_one.work_mode;
//            p_data_download_->pack_one.vehicle_gear = this->pack_one.vehicle_gear;
//            p_data_download_->pack_one.vehicle_turn_to = this->pack_one.vehicle_turn_to;
//            p_data_download_->pack_one.parking_control = this->pack_one.parking_control;
//            p_data_download_->pack_one.ring_control = this->pack_one.ring_control;
//            p_data_download_->pack_one.forward_big_light = this->pack_one.forward_big_light;
//            p_data_download_->pack_one.wide_taillight = this->pack_one.wide_taillight;
//            p_data_download_->pack_one.turn_light = this->pack_one.turn_light;

            return true;
        }

        if ((((uint8_t)p_recv_raw_data[0]) == 0xF3) && (((uint8_t)p_recv_raw_data[1]) == 0x01)) {
            if (p_recv_len != sizeof(this->pack_two.result_data)) {
                return false;
            }
            pack_handle.setID(1);
//            memcpy(this->pack_two.result_data, p_recv_raw_data, sizeof(this->pack_two.result_data));

//            p_data_download_->pack_two.cylinder_select = this->pack_two.cylinder_select;
//            p_data_download_->pack_two.suspension_select = this->pack_two.suspension_select;
//            p_data_download_->pack_two.suspension_work_mode = this->pack_two.suspension_work_mode;
//            p_data_download_->pack_two.suspension_work_mode_detail = this->pack_two.suspension_work_mode_detail;
//            p_data_download_->pack_two.suspension_cylinder_select_mode = this->pack_two.suspension_cylinder_select_mode;
//            p_data_download_->pack_two.suspension_cylinder_motor_control = this->pack_two.suspension_cylinder_motor_control;
//            p_data_download_->pack_two.vertical_wall_mode = this->pack_two.vertical_wall_mode;
//            p_data_download_->pack_two.tailgate_control = this->pack_two.tailgate_control;
//            p_data_download_->pack_two.fix_two_chamber_valve = this->pack_two.fix_two_chamber_valve;
//            p_data_download_->pack_two.brake = this->pack_two.brake;

//            this->work_mode = this->pack_two.work_mode;

            return true;
        }

        return false;
    }

}