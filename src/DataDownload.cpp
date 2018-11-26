#include <DataDownload.hpp>
#include "DataDownload.hpp"

namespace ecu_communication {

DataDownload::DataDownload() {
    this->data_download_pack_one.original_data.data_ID_one = 0;
    this->data_download_pack_one.original_data.data_ID_two = 0;
    this->data_download_pack_one.original_data.data_ID_three = 0;
    this->data_download_pack_one.original_data.data_ID_four = 0xE0;
    this->data_download_pack_two.original_data.data_ID_one = 0;
    this->data_download_pack_two.original_data.data_ID_two = 0;
    this->data_download_pack_two.original_data.data_ID_three = 0;
    this->data_download_pack_two.original_data.data_ID_four = 0xE1;
}

void DataDownload::init() {
    //// todo chat with 31, how to init valid data mark?
    this->data_download_pack_one.original_data.valid_data_mark = 0;
    this->data_download_pack_one.original_data.valid_data_length = 0;
    this->data_download_pack_one.original_data.curvature_low_byte = 0;
    this->data_download_pack_one.original_data.curvature_high_byte = 0;
    this->data_download_pack_one.original_data.expect_vehicle_speed = 0;
    this->data_download_pack_one.original_data.expect_left_speed = 0;
    this->data_download_pack_one.original_data.expect_right_speed = 0;
    this->data_download_pack_one.original_data.work_mode = 0;
    this->data_download_pack_one.original_data.functions_one = 0b00000000;
    this->data_download_pack_one.original_data.functions_two = 0b11000000;

    this->data_download_pack_two.original_data.valid_data_mark = 0;
    this->data_download_pack_two.original_data.valid_data_length = 0;
    this->data_download_pack_two.original_data.functions_three = 0b00000000;
    this->data_download_pack_two.original_data.functions_four = 0b00110000;
    this->data_download_pack_two.original_data.functions_five = 0b11000000;
    this->data_download_pack_two.original_data.reserve_byte1 = 0;
    this->data_download_pack_two.original_data.reserve_byte2 = 0;
    this->data_download_pack_two.original_data.reserve_byte3 = 0;
    this->data_download_pack_two.original_data.reserve_byte4 = 0;
    this->data_download_pack_two.original_data.reserve_byte5 = 0;
}

}