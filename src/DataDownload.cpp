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
    this->data_download_pack_one.valid_data_mark = 0;
    this->data_download_pack_one.valid_data_length = 0;
    this->data_download_pack_one.thousand_times_curvature = 0;
    this->data_download_pack_one.expect_vehicle_speed = 0;
    this->data_download_pack_one.expect_left_speed = 0;
    this->data_download_pack_one.expect_right_speed = 0;
    this->data_download_pack_one.work_mode = 0;
    this->data_download_pack_one.right_wheel_rotate = 0;
    this->data_download_pack_one.left_wheel_rotate = 0;
    this->data_download_pack_one.vehicle_turn_to = 0;
    this->data_download_pack_one.vehicle_gear = 0;
    this->data_download_pack_one.turn_light = 0;
    this->data_download_pack_one.wide_taillight = 0;
    this->data_download_pack_one.forward_big_light = 0;
    this->data_download_pack_one.ring_control = 1;
    this->data_download_pack_one.parking_control = 1;

    this->data_download_pack_two.valid_data_mark = 0;
    this->data_download_pack_two.valid_data_length = 0;
    this->data_download_pack_two.suspension_select = 0;
    this->data_download_pack_two.cylinder_select = 0;
    this->data_download_pack_two.vertical_wall_mode = 0;
    this->data_download_pack_two.suspension_cylinder_motor_control = 0;
    this->data_download_pack_two.suspension_cylinder_select_mode = 0;
    this->data_download_pack_two.suspension_work_mode_detail = 3;
    this->data_download_pack_two.suspension_work_mode = 0;
    this->data_download_pack_two.tailgate_control = 3;
}

}