#include "DataDownload.hpp"

namespace ecu_communication {

DataDownload::DataDownload() {
    memset(this->data_download_pack_one.result_data, 0, sizeof(this->data_download_pack_one.result_data));
    memset(this->data_download_pack_two.result_data, 0, sizeof(this->data_download_pack_two.result_data));
    this->data_download_pack_one.data_ID = 0xE0000000;
    this->data_download_pack_two.data_ID = 0xE1000000;
}

void DataDownload::init() {
    //// todo chat with 31, how to init valid data mark?
    this->data_download_pack_one.valid_data_mark = 1;
    this->data_download_pack_one.valid_data_length = 8;
    this->data_download_pack_one.thousand_times_curvature = 0;
    this->data_download_pack_one.expect_vehicle_speed = 0;
    this->data_download_pack_one.expect_left_speed = 0;
    this->data_download_pack_one.expect_right_speed = 0;
    this->data_download_pack_one.work_mode = (int)three_one_control::work_mode::curvature_and_vehicle_speed;
    this->data_download_pack_one.right_wheel_rotate = (int)three_one_control::right_wheel_rotate::forward;
    this->data_download_pack_one.left_wheel_rotate = (int)three_one_control::left_wheel_rotate::forward;
    this->data_download_pack_one.vehicle_turn_to = (int)three_one_control::vehicle_turn_to::left;
    this->data_download_pack_one.vehicle_gear = (int)three_one_control::vehicle_gear::N;
    this->data_download_pack_one.turn_light = (int)three_one_control::turn_light::all_off;
    this->data_download_pack_one.wide_taillight = (int)three_one_control::wide_taillight::off;
    this->data_download_pack_one.forward_big_light = (int)three_one_control::forward_big_light::off;
    this->data_download_pack_one.ring_control = (int)three_one_control::ring_control::off;
    this->data_download_pack_one.parking_control = (int)three_one_control::parking_control::on;

    this->data_download_pack_two.valid_data_mark = 1;
    this->data_download_pack_two.valid_data_length = 8;
    this->data_download_pack_two.suspension_select = (int)three_one_control::suspension_select::none;
    this->data_download_pack_two.cylinder_select = (int)three_one_control::cylinder_select::left_one;
    this->data_download_pack_two.vertical_wall_mode = (int)three_one_control::vertical_wall_mode::normal_driving;
    this->data_download_pack_two.suspension_cylinder_motor_control = (int)three_one_control::suspension_cylinder_motor_control::off;
    this->data_download_pack_two.suspension_cylinder_select_mode = (int)three_one_control::suspension_cylinder_select_mode::all;
    this->data_download_pack_two.suspension_work_mode_detail = (int)three_one_control::suspension_up_down::keep;
    this->data_download_pack_two.suspension_work_mode = (int)three_one_control::suspension_work_mode::up_down;
    this->data_download_pack_two.tailgate_control = (int)three_one_control::tailgate_control::keep;
    this->data_download_pack_two.fix_two_chamber_valve = (int)three_one_control::fix_two_chamber_valve::fixed;
}

void DataDownload::modify_data_for_test() {
    this->data_download_pack_one.valid_data_mark = 1;
    this->data_download_pack_one.valid_data_length = 8;
    this->data_download_pack_one.thousand_times_curvature = 0;
    this->data_download_pack_one.expect_vehicle_speed = 0;
    this->data_download_pack_one.expect_left_speed = 0;
    this->data_download_pack_one.expect_right_speed = 0;
    this->data_download_pack_one.work_mode = (int)three_one_control::work_mode::curvature_and_vehicle_speed;
    this->data_download_pack_one.right_wheel_rotate = (int)three_one_control::right_wheel_rotate::forward;
    this->data_download_pack_one.left_wheel_rotate = (int)three_one_control::left_wheel_rotate::forward;
    this->data_download_pack_one.vehicle_turn_to = (int)three_one_control::vehicle_turn_to::left;
    this->data_download_pack_one.vehicle_gear = (int)three_one_control::vehicle_gear::N;
    this->data_download_pack_one.turn_light = (int)three_one_control::turn_light::all_off;
    this->data_download_pack_one.wide_taillight = (int)three_one_control::wide_taillight::off;
    this->data_download_pack_one.forward_big_light = (int)three_one_control::forward_big_light::off;
    this->data_download_pack_one.ring_control = (int)three_one_control::ring_control::off;
    this->data_download_pack_one.parking_control = (int)three_one_control::parking_control::on;

    this->data_download_pack_two.valid_data_mark = 1;
    this->data_download_pack_two.valid_data_length = 8;
    this->data_download_pack_two.suspension_select = (int)three_one_control::suspension_select::none;
    this->data_download_pack_two.cylinder_select = (int)three_one_control::cylinder_select::left_one;
    this->data_download_pack_two.vertical_wall_mode = (int)three_one_control::vertical_wall_mode::normal_driving;
    this->data_download_pack_two.suspension_cylinder_motor_control = (int)three_one_control::suspension_cylinder_motor_control::off;
    this->data_download_pack_two.suspension_cylinder_select_mode = (int)three_one_control::suspension_cylinder_select_mode::all;
    this->data_download_pack_two.suspension_work_mode_detail = (int)three_one_control::suspension_up_down::keep;
    this->data_download_pack_two.suspension_work_mode = (int)three_one_control::suspension_work_mode::up_down;
    this->data_download_pack_two.tailgate_control = (int)three_one_control::tailgate_control::keep;
    this->data_download_pack_two.fix_two_chamber_valve = (int)three_one_control::fix_two_chamber_valve::fixed;
}

}