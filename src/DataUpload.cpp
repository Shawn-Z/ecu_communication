#include "DataUpload.hpp"

namespace ecu_communication {

DataUpload::DataUpload() {
    memset(this->pack_one.pack, 0, sizeof(this->pack_one.pack));
    memset(this->pack_two.pack, 0, sizeof(this->pack_two.pack));
    memset(this->pack_three.pack, 0, sizeof(this->pack_three.pack));
    memset(this->pack_four.pack, 0, sizeof(this->pack_four.pack));
    memset(this->recv_raw_data, 0, sizeof(this->recv_raw_data));
    this->ID_calculate.result = 0;
    this->recv_counter = 0;
}

bool DataUpload::dataIDCheck(char *p_recv_raw_data) {
    memcpy(this->recv_raw_data, p_recv_raw_data, sizeof(this->recv_raw_data));
    this->ID_calculate.data[0] = this->recv_raw_data[0];
    this->ID_calculate.data[1] = this->recv_raw_data[1];
    this->ID_calculate.data[2] = this->recv_raw_data[2];
    this->ID_calculate.data[3] = this->recv_raw_data[3];
    size_t tmp_ID = 0;
    switch (this->ID_calculate.result) {
        case 0xF0000000: {
            tmp_ID = 0;
            break;
        }
        case 0xF1000000: {
            tmp_ID = 1;
            break;
        }
        case 0xF2000000: {
            tmp_ID = 2;
            break;
        }
        case 0xF3000000: {
            tmp_ID = 3;
            break;
        }
        default: {
            return false;
        }
    }
    this->pack_handle.setID(tmp_ID);
    return true;
}

void DataUpload::dataDistribution() {
    ++this->recv_counter;
    switch (this->pack_handle.getID()) {
        case 1: {
            memcpy(this->pack_one.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 2: {
            memcpy(this->pack_two.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 3: {
            memcpy(this->pack_three.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 4: {
            memcpy(this->pack_four.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        default: {
            --this->recv_counter;
            break;
        }
    }
}

bool DataUpload::dataCheck() {
    if (this->pack_one.left_wheel_expect_speed > 25000) {
        return false;
    }
    if (this->pack_one.right_wheel_expect_speed > 25000) {
        return false;
    }
    if (this->pack_one.mechanical_brake > 1000) {
        return false;
    }
    if (this->pack_one.vehicle_speed > 250) {
        return false;
    }
    if ((this->pack_one.gear < 1) || (this->pack_one.gear > 3)) {
        return false;
    }

    if (this->pack_two.left_motor_actual_speed > 10000) {
        return false;
    }
    if (this->pack_two.right_motor_actual_speed > 10000) {
        return false;
    }
    if (this->pack_two.left_motor_gear > 1) {
        return false;
    }
    if (this->pack_two.right_motor_gear > 1) {
        return false;
    }
    if (this->pack_two.SOC > 100) {
        return false;
    }
    if (this->pack_two.tailgate_state > 2) {
        return false;
    }

    if (this->pack_three.left_one_cylinder_position > 250) {
        return false;
    }
    if (this->pack_three.left_two_cylinder_position > 250) {
        return false;
    }
    if (this->pack_three.left_three_cylinder_position > 250) {
        return false;
    }
    if (this->pack_three.left_four_cylinder_position > 250) {
        return false;
    }
    if (this->pack_three.right_one_cylinder_position > 250) {
        return false;
    }
    if (this->pack_three.right_two_cylinder_position > 250) {
        return false;
    }
    if (this->pack_three.right_three_cylinder_position > 250) {
        return false;
    }
    if (this->pack_three.right_four_cylinder_position > 250) {
        return false;
    }

    if (this->pack_four.left_one_cylinder_pressure > 250) {
        return false;
    }
    if (this->pack_four.left_two_cylinder_pressure > 250) {
        return false;
    }
    if (this->pack_four.left_three_cylinder_pressure > 250) {
        return false;
    }
    if (this->pack_four.left_four_cylinder_pressure > 250) {
        return false;
    }
    if (this->pack_four.right_one_cylinder_pressure > 250) {
        return false;
    }
    if (this->pack_four.right_two_cylinder_pressure > 250) {
        return false;
    }
    if (this->pack_four.right_three_cylinder_pressure > 250) {
        return false;
    }
    if (this->pack_four.right_four_cylinder_pressure > 250) {
        return false;
    }
    return true;
}

void DataUpload::dataToMsg() {
    this->report.counter = this->recv_counter;

    this->report.give_back.left_wheel_expect_speed = this->pack_one.left_wheel_expect_speed * 0.001;
    this->report.give_back.right_wheel_expect_speed = this->pack_one.right_wheel_expect_speed * 0.001;
    this->report.vehicle_state.mechanical_brake = this->pack_one.mechanical_brake * 0.1;
    this->report.vehicle_state.vehicle_speed = this->pack_one.vehicle_speed * 0.1;
    this->report.vehicle_state.current_gear = this->pack_one.gear;

    this->report.vehicle_state.left_motor_rpm = this->pack_two.left_motor_actual_speed;
    this->report.vehicle_state.right_motor_rpm = this->pack_two.right_motor_actual_speed;
    this->report.vehicle_state.left_wheel_rotate = this->pack_two.left_motor_gear;
    this->report.vehicle_state.right_wheel_rotate = this->pack_two.right_motor_gear;
    this->report.vehicle_state.SOC = this->pack_two.SOC;
    this->report.vehicle_state.tailgate_state = this->pack_two.tailgate_state;

    this->report.cylinder_position.left_one_cylinder_position = this->pack_three.left_one_cylinder_position * 2;
    this->report.cylinder_position.left_two_cylinder_position = this->pack_three.left_two_cylinder_position * 2;
    this->report.cylinder_position.left_three_cylinder_position = this->pack_three.left_three_cylinder_position * 2;
    this->report.cylinder_position.left_four_cylinder_position = this->pack_three.left_four_cylinder_position * 2;
    this->report.cylinder_position.right_one_cylinder_position = this->pack_three.right_one_cylinder_position * 2;
    this->report.cylinder_position.right_two_cylinder_position = this->pack_three.right_two_cylinder_position * 2;
    this->report.cylinder_position.right_three_cylinder_position = this->pack_three.right_three_cylinder_position * 2;
    this->report.cylinder_position.right_four_cylinder_position = this->pack_three.right_four_cylinder_position * 2;

    this->report.cylinder_pressure.left_one_cylinder_pressure = this->pack_four.left_one_cylinder_pressure;
    this->report.cylinder_pressure.left_two_cylinder_pressure = this->pack_four.left_two_cylinder_pressure;
    this->report.cylinder_pressure.left_three_cylinder_pressure = this->pack_four.left_three_cylinder_pressure;
    this->report.cylinder_pressure.left_four_cylinder_pressure = this->pack_four.left_four_cylinder_pressure;
    this->report.cylinder_pressure.right_one_cylinder_pressure = this->pack_four.right_one_cylinder_pressure;
    this->report.cylinder_pressure.right_two_cylinder_pressure = this->pack_four.right_two_cylinder_pressure;
    this->report.cylinder_pressure.right_three_cylinder_pressure = this->pack_four.right_three_cylinder_pressure;
    this->report.cylinder_pressure.right_four_cylinder_pressure = this->pack_four.right_four_cylinder_pressure;
}

}