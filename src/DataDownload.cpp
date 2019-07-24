#include "DataDownload.hpp"

namespace ecu_communication {

DataDownload::DataDownload() {
    memset(this->pack_one.result_data, 0, sizeof(this->pack_one.result_data));
    memset(this->pack_two.result_data, 0, sizeof(this->pack_two.result_data));
    memset(this->data_to_send, 0, sizeof(this->data_to_send));
    this->send_rawdata.data.clear();
    this->pack_one.data_ID = 0xE0000000;
    this->pack_two.data_ID = 0xE1000000;
    this->pack_one.valid_data_mark = 1;
    this->pack_one.valid_data_length = 8;
    this->pack_two.valid_data_mark = 1;
    this->pack_two.valid_data_length = 8;


    this->pack_one.thousand_times_curvature = 0;
    this->pack_one.expect_vehicle_speed = 0;
    this->pack_one.expect_left_speed = 0;
    this->pack_one.expect_right_speed = 0;
    this->pack_one.work_mode = (int)three_one_control::work_mode::halt;
    this->pack_one.right_wheel_rotate = (int)three_one_control::right_wheel_rotate::forward;
    this->pack_one.left_wheel_rotate = (int)three_one_control::left_wheel_rotate::forward;
    this->pack_one.vehicle_turn_to = (int)three_one_control::vehicle_turn_to::left;
    this->pack_one.vehicle_gear = (int)three_one_control::vehicle_gear::N;
    this->pack_one.turn_light = (int)three_one_control::turn_light::all_off;
    this->pack_one.wide_taillight = (int)three_one_control::wide_taillight::off;
    this->pack_one.forward_big_light = (int)three_one_control::forward_big_light::off;
    this->pack_one.ring_control = (int)three_one_control::ring_control::off;
    this->pack_one.parking_control = (int)three_one_control::parking_control::off;

    this->pack_two.suspension_select = (int)three_one_control::suspension_select::none;
    this->pack_two.cylinder_select = (int)three_one_control::cylinder_select::none;
    this->pack_two.vertical_wall_mode = (int)three_one_control::vertical_wall_mode::normal_driving;
    this->pack_two.suspension_cylinder_motor_control = (int)three_one_control::suspension_cylinder_motor_control::off;
    this->pack_two.suspension_cylinder_select_mode = (int)three_one_control::suspension_cylinder_select_mode::all;
    this->pack_two.suspension_work_mode_detail = (int)three_one_control::suspension_up_down::keep;
    this->pack_two.suspension_work_mode = (int)three_one_control::suspension_work_mode::up_down;
    this->pack_two.tailgate_control = (int)three_one_control::tailgate_control::keep;
    this->pack_two.fix_two_chamber_valve = (int)three_one_control::fix_two_chamber_valve::fixed;
    this->pack_two.brake = 0;
    this->pack_two.weapon_330 = (int)three_one_control::weapon_330::off;
    this->pack_two.entrenchment = (int)three_one_control::entrenchment::disable;
    this->pack_two.weapon_28 = (int)three_one_control::weapon_28::off;
}

void DataDownload::init(bool chamber_fix) {
    this->pack_one.thousand_times_curvature = 0;
    this->pack_one.expect_vehicle_speed = 0;
    this->pack_one.expect_left_speed = 0;
    this->pack_one.expect_right_speed = 0;
    this->pack_one.work_mode = (int)three_one_control::work_mode::halt;
    this->pack_one.right_wheel_rotate = (int)three_one_control::right_wheel_rotate::forward;
    this->pack_one.left_wheel_rotate = (int)three_one_control::left_wheel_rotate::forward;
    this->pack_one.vehicle_turn_to = (int)three_one_control::vehicle_turn_to::left;
    this->pack_one.vehicle_gear = (int)three_one_control::vehicle_gear::N;
    this->pack_one.turn_light = (int)three_one_control::turn_light::all_off;
    this->pack_one.wide_taillight = (int)three_one_control::wide_taillight::on;
    this->pack_one.forward_big_light = (int)three_one_control::forward_big_light::off;
    this->pack_one.ring_control = (int)three_one_control::ring_control::off;
    this->pack_one.parking_control = (int)three_one_control::parking_control::off;

    this->pack_two.suspension_select = (int)three_one_control::suspension_select::none;
    this->pack_two.cylinder_select = (int)three_one_control::cylinder_select::none;
    this->pack_two.vertical_wall_mode = (int)three_one_control::vertical_wall_mode::normal_driving;
    this->pack_two.suspension_cylinder_motor_control = (int)three_one_control::suspension_cylinder_motor_control::off;
    this->pack_two.suspension_cylinder_select_mode = (int)three_one_control::suspension_cylinder_select_mode::all;
    this->pack_two.suspension_work_mode_detail = (int)three_one_control::suspension_up_down::keep;
    this->pack_two.suspension_work_mode = (int)three_one_control::suspension_work_mode::up_down;
    this->pack_two.tailgate_control = (int)three_one_control::tailgate_control::keep;
    this->pack_two.fix_two_chamber_valve = chamber_fix? ((int)three_one_control::fix_two_chamber_valve::fixed) : ((int)three_one_control::fix_two_chamber_valve::not_fixed);
    this->pack_two.brake = 0;
    this->pack_two.weapon_330 = (int)three_one_control::weapon_330::off;
    this->pack_two.entrenchment = (int)three_one_control::entrenchment::disable;
    this->pack_two.weapon_28 = (int)three_one_control::weapon_28::off;
}

void DataDownload::prepareSend(shawn::handle p_handle) {
    switch (p_handle.getID()) {
        case 0: {
            memcpy(this->data_to_send, this->pack_one.result_data, sizeof(this->pack_one.result_data));
            break;
        }
        case 1: {
            memcpy(this->data_to_send, this->pack_two.result_data, sizeof(this->pack_two.result_data));
            break;
        }
        default: {
            break;
        }
    }
}

bool DataDownload::dataIDCheck(char *p_recv_raw_data) {
    memcpy(this->recv_raw_data, p_recv_raw_data, sizeof(this->recv_raw_data));
    this->ID_calculate.data[0] = this->recv_raw_data[0];
    this->ID_calculate.data[1] = this->recv_raw_data[1];
    this->ID_calculate.data[2] = this->recv_raw_data[2];
    this->ID_calculate.data[3] = this->recv_raw_data[3];
    size_t tmp_ID = 0;
    switch (this->ID_calculate.result) {
        case 0xE0000000: {
            tmp_ID = 0;
            break;
        }
        case 0xE1000000: {
            tmp_ID = 1;
            break;
        }
        default: {
            return false;
        }
    }
    this->pack_handle.setID(tmp_ID);
    return true;
}

void DataDownload::dataDistribution() {
    switch (this->pack_handle.getID()) {
        case 0: {
            memcpy(this->pack_one.result_data, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 1: {
            memcpy(this->pack_two.result_data, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        default: {
            break;
        }
    }
}

bool DataDownload::durex(bool move, bool parked, bool need_force_stop, double limit_speed, int limit_thousand_curv) {
    static std::vector<bool> history_move(100, true);
    history_move.erase(history_move.begin());
    history_move.emplace_back(move);
    bool not_fully_stop = move;
    for (auto cell: history_move) {
        not_fully_stop |= cell;
    }
    if (parked) {
        not_fully_stop = false;
    }
    //todo
    if (!need_force_stop) {
        this->pack_one.parking_control = (uint8_t)three_one_control::parking_control::off;
    }
    if (need_force_stop && not_fully_stop) {
        this->pack_one.expect_vehicle_speed = 0;
        this->pack_one.thousand_times_curvature = std::min<uint16_t>(this->pack_one.thousand_times_curvature, 1250);
        this->pack_one.work_mode = (uint8_t)three_one_control::work_mode::halt;
        this->pack_one.parking_control = (uint8_t)three_one_control::parking_control::off;
    }
    if (need_force_stop && (!not_fully_stop)) {
        this->pack_one.expect_vehicle_speed = 0;
        this->pack_one.thousand_times_curvature = std::min<uint16_t>(this->pack_one.thousand_times_curvature, 1250);
        this->pack_one.work_mode = (uint8_t)three_one_control::work_mode::curvature_and_vehicle_speed;
        this->pack_one.parking_control = (uint8_t)three_one_control::parking_control::on;
    }
    if (parked) {
        this->pack_one.expect_vehicle_speed = std::min<uint8_t>(this->pack_one.expect_vehicle_speed, 1);
//        this->pack_one.thousand_times_curvature = std::min<uint8_t>(this->pack_one.thousand_times_curvature, 100);
    }
    if (this->pack_one.vehicle_gear == (uint8_t)three_one_control::vehicle_gear::N) {
        this->pack_one.expect_vehicle_speed = 0;
    }
    this->pack_one.expect_vehicle_speed = std::min<uint8_t>(this->pack_one.expect_vehicle_speed, (uint8_t)round(limit_speed * 10.0));
    this->pack_one.thousand_times_curvature = std::min<uint16_t>(this->pack_one.thousand_times_curvature, (uint16_t)limit_thousand_curv);
}

void DataDownload::devicesControl(three_one_feedback::control_mode control_mode) {
    if (control_mode == three_one_feedback::control_mode::autonomous) {
        this->pack_one.wide_taillight = (uint8_t)three_one_control::wide_taillight::on;
    }
    if (control_mode == three_one_feedback::control_mode::remote) {
        this->pack_one.wide_taillight = (uint8_t)three_one_control::wide_taillight::off;
    }
    if (control_mode == three_one_feedback::control_mode::ERROR) {
        this->pack_one.turn_light = (uint8_t)three_one_control::turn_light::all_on;
    }
}

}