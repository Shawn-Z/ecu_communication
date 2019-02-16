#include "Transform6t.hpp"

namespace ecu_communication {

    bool Transform6t::receiveCheck(char *p_recv_raw_data) {
        memcpy(this->receive_6t.pack, p_recv_raw_data, sizeof(this->receive_6t.pack));
        bool ID_check = true;
        ID_check &= (this->receive_6t.pack[0] == 0x88);
        ID_check &= (this->receive_6t.pack[1] == 0x01);
        ID_check &= (this->receive_6t.pack[2] == 0x00);
        ID_check &= (this->receive_6t.pack[3] == 0x04);
        ID_check &= (this->receive_6t.pack[4] == 0x20);
        return ID_check;
    }

}