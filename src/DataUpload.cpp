#include "DataUpload.hpp"

namespace ecu_communication {

void DataUpload::dataDistribution() {
    switch (this->data_pack_num) {
        case 1: {
            memcpy(this->data_upload_pack_one.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 2: {
            memcpy(this->data_upload_pack_two.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 3: {
            memcpy(this->data_upload_pack_three.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 4: {
            memcpy(this->data_upload_pack_four.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        default: {
            break;
        }
    }
}

DataUpload::DataUpload() {
    memset(this->data_upload_pack_one.pack, 0, sizeof(this->data_upload_pack_one.pack));
    memset(this->data_upload_pack_two.pack, 0, sizeof(this->data_upload_pack_two.pack));
    memset(this->data_upload_pack_three.pack, 0, sizeof(this->data_upload_pack_three.pack));
    memset(this->data_upload_pack_four.pack, 0, sizeof(this->data_upload_pack_four.pack));

    memset(this->recv_raw_data, 0, sizeof(this->recv_raw_data));

    this->ID_calculate.result = 0;
}

int DataUpload::dataPackCheck(char *p_recv_raw_data) {
    memcpy(this->recv_raw_data, p_recv_raw_data, sizeof(this->recv_raw_data));
    this->ID_calculate.data[0] = this->recv_raw_data[0];
    this->ID_calculate.data[1] = this->recv_raw_data[1];
    this->ID_calculate.data[2] = this->recv_raw_data[2];
    this->ID_calculate.data[3] = this->recv_raw_data[3];
    switch (this->ID_calculate.result) {
        case 0xF0000000: {
            this->data_pack_num = 1;
            break;
        }
        case 0xF1000000: {
            this->data_pack_num = 2;
            break;
        }
        case 0xF2000000: {
            this->data_pack_num = 3;
            break;
        }
        case 0xF3000000: {
            this->data_pack_num = 4;
            break;
        }
        default: {
            this->data_pack_num = -1;
            break;
        }
    }
    return this->data_pack_num;
}

}