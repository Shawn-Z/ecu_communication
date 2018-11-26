#include "DataUpload.hpp"

namespace ecu_communication {

bool DataUpload::dataDistribution() {
    this->ID_calculate.data[0] = this->recv_raw_data[0];
    this->ID_calculate.data[1] = this->recv_raw_data[1];
    this->ID_calculate.data[2] = this->recv_raw_data[2];
    this->ID_calculate.data[3] = this->recv_raw_data[3];
    switch (this->ID_calculate.result) {
        case 0xF0000000: {
            memcpy(this->data_upload_pack_one.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 0xF1000000: {
            memcpy(this->data_upload_pack_two.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 0xF2000000: {
            memcpy(this->data_upload_pack_three.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 0xF3000000: {
            memcpy(this->data_upload_pack_four.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        default: {
            return false;
        }
    }
    return true;
}

}