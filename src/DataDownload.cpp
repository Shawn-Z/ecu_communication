#include "DataDownload.hpp"

namespace ecu_communication {

DataDownload::DataDownload() {
    memset(this->pack_one.result_data, 0, sizeof(this->pack_one.result_data));
    memset(this->pack_two.result_data, 0, sizeof(this->pack_two.result_data));
    memset(this->data_to_send, 0, sizeof(this->data_to_send));
    this->pack_one.data_ID = 0xE0000000;
    this->pack_two.data_ID = 0xE1000000;
}

void DataDownload::init() {
    //// todo chat with 31, how to init valid data mark?
    this->pack_one.valid_data_mark = 1;
    this->pack_one.valid_data_length = 8;
    this->pack_one.thousand_times_curvature = 0;
    this->pack_one.expect_vehicle_speed = 0;
    this->pack_one.expect_left_speed = 0;
    this->pack_one.expect_right_speed = 0;
    this->pack_one.work_mode = (int)three_one_control::work_mode::curvature_and_vehicle_speed;
    this->pack_one.right_wheel_rotate = (int)three_one_control::right_wheel_rotate::forward;
    this->pack_one.left_wheel_rotate = (int)three_one_control::left_wheel_rotate::forward;
    this->pack_one.vehicle_turn_to = (int)three_one_control::vehicle_turn_to::left;
    this->pack_one.vehicle_gear = (int)three_one_control::vehicle_gear::N;
    this->pack_one.turn_light = (int)three_one_control::turn_light::all_off;
    this->pack_one.wide_taillight = (int)three_one_control::wide_taillight::off;
    this->pack_one.forward_big_light = (int)three_one_control::forward_big_light::off;
    this->pack_one.ring_control = (int)three_one_control::ring_control::off;
    this->pack_one.parking_control = (int)three_one_control::parking_control::on;

    this->pack_two.valid_data_mark = 1;
    this->pack_two.valid_data_length = 8;
    this->pack_two.suspension_select = (int)three_one_control::suspension_select::none;
    this->pack_two.cylinder_select = (int)three_one_control::cylinder_select::left_one;
    this->pack_two.vertical_wall_mode = (int)three_one_control::vertical_wall_mode::normal_driving;
    this->pack_two.suspension_cylinder_motor_control = (int)three_one_control::suspension_cylinder_motor_control::off;
    this->pack_two.suspension_cylinder_select_mode = (int)three_one_control::suspension_cylinder_select_mode::all;
    this->pack_two.suspension_work_mode_detail = (int)three_one_control::suspension_up_down::keep;
    this->pack_two.suspension_work_mode = (int)three_one_control::suspension_work_mode::up_down;
    this->pack_two.tailgate_control = (int)three_one_control::tailgate_control::keep;
    this->pack_two.fix_two_chamber_valve = (int)three_one_control::fix_two_chamber_valve::fixed;
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

void DataDownload::modify_data_for_test() {
    this->pack_one.valid_data_mark = 1;
    this->pack_one.valid_data_length = 8;
    this->pack_one.thousand_times_curvature = 0;
    this->pack_one.expect_vehicle_speed = 0;
    this->pack_one.expect_left_speed = 0;
    this->pack_one.expect_right_speed = 0;
    this->pack_one.work_mode = (int)three_one_control::work_mode::curvature_and_vehicle_speed;
    this->pack_one.right_wheel_rotate = (int)three_one_control::right_wheel_rotate::forward;
    this->pack_one.left_wheel_rotate = (int)three_one_control::left_wheel_rotate::forward;
    this->pack_one.vehicle_turn_to = (int)three_one_control::vehicle_turn_to::left;
    this->pack_one.vehicle_gear = (int)three_one_control::vehicle_gear::N;
    this->pack_one.turn_light = (int)three_one_control::turn_light::all_off;
    this->pack_one.wide_taillight = (int)three_one_control::wide_taillight::off;
    this->pack_one.forward_big_light = (int)three_one_control::forward_big_light::off;
    this->pack_one.ring_control = (int)three_one_control::ring_control::off;
    this->pack_one.parking_control = (int)three_one_control::parking_control::on;

    this->pack_two.valid_data_mark = 1;
    this->pack_two.valid_data_length = 8;
    this->pack_two.suspension_select = (int)three_one_control::suspension_select::none;
    this->pack_two.cylinder_select = (int)three_one_control::cylinder_select::left_one;
    this->pack_two.vertical_wall_mode = (int)three_one_control::vertical_wall_mode::normal_driving;
    this->pack_two.suspension_cylinder_motor_control = (int)three_one_control::suspension_cylinder_motor_control::off;
    this->pack_two.suspension_cylinder_select_mode = (int)three_one_control::suspension_cylinder_select_mode::all;
    this->pack_two.suspension_work_mode_detail = (int)three_one_control::suspension_up_down::keep;
    this->pack_two.suspension_work_mode = (int)three_one_control::suspension_work_mode::up_down;
    this->pack_two.tailgate_control = (int)three_one_control::tailgate_control::keep;
    this->pack_two.fix_two_chamber_valve = (int)three_one_control::fix_two_chamber_valve::fixed;
}

}