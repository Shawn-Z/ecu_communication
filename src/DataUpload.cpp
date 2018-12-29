#include "DataUpload.hpp"

namespace ecu_communication {

DataUpload::DataUpload() {
    memset(this->pack_one.pack, 0, sizeof(this->pack_one.pack));
    memset(this->pack_two.pack, 0, sizeof(this->pack_two.pack));
    memset(this->pack_three.pack, 0, sizeof(this->pack_three.pack));
    memset(this->pack_four.pack, 0, sizeof(this->pack_four.pack));
    memset(this->pack_five.pack, 0, sizeof(this->pack_five.pack));
    memset(this->pack_six.pack, 0, sizeof(this->pack_six.pack));
    memset(this->pack_seven.pack, 0, sizeof(this->pack_seven.pack));
    memset(this->recv_raw_data, 0, sizeof(this->recv_raw_data));
    this->ID_calculate.result = 0;
    this->recv_counter = 0;
    this->recv_rawdata.recv_rawdata.clear();
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
        case 0xF4000000: {
            tmp_ID = 4;
            break;
        }
        case 0xF5000000: {
            tmp_ID = 5;
            break;
        }
        case 0xF6000000: {
            tmp_ID = 6;
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
        case 0: {
            memcpy(this->pack_one.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 1: {
            memcpy(this->pack_two.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 2: {
            memcpy(this->pack_three.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 3: {
            memcpy(this->pack_four.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 4: {
            memcpy(this->pack_five.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 5: {
            memcpy(this->pack_six.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        case 6: {
            memcpy(this->pack_seven.pack, this->recv_raw_data, sizeof(this->recv_raw_data));
            break;
        }
        default: {
            --this->recv_counter;
            break;
        }
    }
}

bool DataUpload::dataCheck() {
    //// todo modify
    return true;
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
    this->report.speed.vehicle_speed = this->pack_one.vehicle_speed * 0.1;
    this->report.gear.current_gear = this->pack_one.gear;

    this->report.speed.left_motor_rpm = this->pack_two.left_motor_actual_speed;
    this->report.speed.right_motor_rpm = this->pack_two.right_motor_actual_speed;
    this->report.gear.left_wheel_rotate = this->pack_two.left_motor_gear;
    this->report.gear.right_wheel_rotate = this->pack_two.right_motor_gear;
    this->report.vehicle_state.SOC = this->pack_two.SOC;
    this->report.vehicle_state.tailgate_state = this->pack_two.tailgate_state;

    this->report.cylinder_position.left_one = this->pack_three.left_one_cylinder_position * 2;
    this->report.cylinder_position.left_two = this->pack_three.left_two_cylinder_position * 2;
    this->report.cylinder_position.left_three = this->pack_three.left_three_cylinder_position * 2;
    this->report.cylinder_position.left_four = this->pack_three.left_four_cylinder_position * 2;
    this->report.cylinder_position.right_one = this->pack_three.right_one_cylinder_position * 2;
    this->report.cylinder_position.right_two = this->pack_three.right_two_cylinder_position * 2;
    this->report.cylinder_position.right_three = this->pack_three.right_three_cylinder_position * 2;
    this->report.cylinder_position.right_four = this->pack_three.right_four_cylinder_position * 2;

    this->report.cylinder_pressure.left_one = this->pack_four.left_one_cylinder_pressure;
    this->report.cylinder_pressure.left_two = this->pack_four.left_two_cylinder_pressure;
    this->report.cylinder_pressure.left_three = this->pack_four.left_three_cylinder_pressure;
    this->report.cylinder_pressure.left_four = this->pack_four.left_four_cylinder_pressure;
    this->report.cylinder_pressure.right_one = this->pack_four.right_one_cylinder_pressure;
    this->report.cylinder_pressure.right_two = this->pack_four.right_two_cylinder_pressure;
    this->report.cylinder_pressure.right_three = this->pack_four.right_three_cylinder_pressure;
    this->report.cylinder_pressure.right_four = this->pack_four.right_four_cylinder_pressure;

    this->report.distance.mileage = this->pack_five.mileage;
    this->report.vehicle_state.vehicle_roll = this->pack_five.vehicle_roll;
    this->report.vehicle_state.vehicle_pitch = this->pack_five.vehicle_pitch;
    this->report.torque.left = this->pack_five.left_torque;

    this->report.torque.right = this->pack_six.right_torque;
    this->report.vehicle_state.vertical_wall_status = this->pack_six.vertical_wall_status;
    this->report.vehicle_state.error_code = this->pack_six.error_code;
    this->report.distance.left_pulse = this->pack_six.left_pulse;

    this->report.distance.right_pulse = this->pack_seven.right_pulse;
    this->report.vehicle_state.vehicle_height = this->pack_seven.vehical_height;
    this->report.vehicle_state.park_status = this->pack_seven.park_status;
}

}