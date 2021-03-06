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
    this->recv_rawdata.data.clear();

}

bool DataUpload::dataIDCheck(char *p_recv_raw_data) {
//    bool tmp_result = true;
//    memcpy(this->all_packs.all, p_recv_raw_data, sizeof(this->all_packs.all));
//    tmp_result &= (this->all_packs.pack_one.data_ID == 0x000000F0);
//    tmp_result &= (this->all_packs.pack_two.data_ID == 0x000000F1);
//    tmp_result &= (this->all_packs.pack_three.data_ID == 0x000000F2);
//    tmp_result &= (this->all_packs.pack_four.data_ID == 0x000000F3);
//    tmp_result &= (this->all_packs.pack_five.data_ID == 0x000000F4);
//    tmp_result &= (this->all_packs.pack_six.data_ID == 0x000000F5);
//    tmp_result &= (this->all_packs.pack_seven.data_ID == 0x000000F6);
//    return tmp_result;

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
//    memcpy(this->pack_one.pack, this->all_packs.pack_one.pack, sizeof(this->pack_one.pack));
//    memcpy(this->pack_two.pack, this->all_packs.pack_two.pack, sizeof(this->pack_two.pack));
//    memcpy(this->pack_three.pack, this->all_packs.pack_three.pack, sizeof(this->pack_three.pack));
//    memcpy(this->pack_four.pack, this->all_packs.pack_four.pack, sizeof(this->pack_four.pack));
//    memcpy(this->pack_five.pack, this->all_packs.pack_five.pack, sizeof(this->pack_five.pack));
//    memcpy(this->pack_six.pack, this->all_packs.pack_six.pack, sizeof(this->pack_six.pack));
//    memcpy(this->pack_seven.pack, this->all_packs.pack_one.pack, sizeof(this->pack_seven.pack));
//    ++this->recv_counter;
//    return;

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
    this->report.operation_mode = this->pack_seven.operation_mode;

    this->report.motion.park = this->pack_seven.park_status;
    this->report.motion.current_gear = this->pack_one.gear;
    this->report.motion.left_wheel_rotate = this->pack_two.left_motor_gear;
    this->report.motion.right_wheel_rotate = this->pack_two.right_motor_gear;
    this->report.motion.left_motor_rpm = this->pack_two.left_motor_actual_speed;
    this->report.motion.right_motor_rpm = this->pack_two.right_motor_actual_speed;
    this->report.motion.mechanical_brake = this->pack_one.mechanical_brake * 0.1;
//    this->report.motion.vehicle_speed = this->pack_one.vehicle_speed * 0.1;
    //// todo comment for 6t
    this->report.motion.left_wheel_speed =
            ((this->pack_two.left_motor_gear == (uint8_t)three_one_feedback::left_wheel_rotate::forward)? 1: -1) *
            this->pack_two.left_motor_actual_speed * RPM_TO_SPEED;
    this->report.motion.right_wheel_speed =
            ((this->pack_two.right_motor_gear == (uint8_t)three_one_feedback::right_wheel_rotate ::forward)? 1: -1) *
            this->pack_two.right_motor_actual_speed * RPM_TO_SPEED;

    //// todo this block for 6t test
//    this->report.motion.left_wheel_speed =
//            ((this->pack_two.left_motor_gear == (uint8_t)three_one_feedback::left_wheel_rotate::forward)? 1: -1) *
//            this->pack_two.left_motor_actual_speed / 148.5 / 3.6;
//    this->report.motion.right_wheel_speed =
//            ((this->pack_two.right_motor_gear == (uint8_t)three_one_feedback::right_wheel_rotate ::forward)? 1: -1) *
//            this->pack_two.right_motor_actual_speed / 148.5 / 3.6;
//    this->report.motion.current_gear = (uint8_t)three_one_feedback::current_gear::N;
//    if ((this->report.motion.left_wheel_speed <= 0) && (this->report.motion.right_wheel_speed <= 0)) {
//        this->report.motion.current_gear = (uint8_t)three_one_feedback::current_gear::R;
//    }
//    if ((this->report.motion.left_wheel_speed >= 0) && (this->report.motion.right_wheel_speed >= 0)) {
//        this->report.motion.current_gear = (uint8_t)three_one_feedback::current_gear::D;
//    }





    this->report.motion.vehicle_speed = 0.5 * (this->report.motion.left_wheel_speed + this->report.motion.right_wheel_speed);
    if (this->report.motion.left_wheel_speed * this->report.motion.right_wheel_speed < -0.0001) {
        this->report.motion.current_gear = (uint8_t)three_one_feedback::current_gear::spin;
        if (this->report.motion.left_wheel_speed < this->report.motion.right_wheel_speed) {
            this->report.motion.spin = (uint8_t)three_one_feedback::spin_status::counterclockwise;
        } else {
            this->report.motion.spin = (uint8_t)three_one_feedback::spin_status::clockwise;
        }
    } else {
        this->report.motion.spin = (uint8_t)three_one_feedback::spin_status::not_spin;
    }

    this->report.torque.left = this->pack_five.left_torque;
    this->report.torque.right = this->pack_six.right_torque;

    this->report.distance.mileage = this->pack_five.mileage;
    this->report.distance.left_pulse = this->pack_six.left_pulse;
    this->report.distance.right_pulse = this->pack_seven.right_pulse;
    this->report.distance.left_distance = this->pack_six.left_pulse * ONE_PULSE_DISTANCE;
    this->report.distance.right_distance = this->pack_seven.right_pulse * ONE_PULSE_DISTANCE;

    this->report.vehicle_state.error_code = this->pack_six.error_code;
    this->report.vehicle_state.vertical_wall_status = this->pack_six.vertical_wall_status;
    this->report.vehicle_state.vehicle_height = this->pack_seven.vehicle_height;
    this->report.vehicle_state.tailgate_state = this->pack_two.tailgate_state;
    this->report.vehicle_state.SOC = this->pack_two.SOC;
    this->report.vehicle_state.vehicle_roll = this->pack_five.vehicle_roll;
    this->report.vehicle_state.vehicle_pitch = this->pack_five.vehicle_pitch;
    this->report.vehicle_state.two_chamber_valve = this->pack_seven.two_chamber_valve;
    this->report.vehicle_state.entrenchment = this->pack_seven.entrenchment;
    this->report.vehicle_state.weapon_330 = this->pack_seven.weapon_330;
    this->report.vehicle_state.weapon_28 = this->pack_seven.weapon_28;

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

    this->report.give_back.left_wheel_expect_speed = this->pack_one.left_wheel_expect_speed * 0.001;
    this->report.give_back.right_wheel_expect_speed = this->pack_one.right_wheel_expect_speed * 0.001;

    //// todo report three_one speed
    this->report.motion.vehicle_speed = this->pack_one.vehicle_speed / 10.0;
    double l_plus_r = this->report.motion.left_wheel_speed + this->report.motion.right_wheel_speed;
    double l_minus_r = this->report.motion.left_wheel_speed - this->report.motion.right_wheel_speed;
    if (fabs(l_plus_r) < DBL_EPSILON) {
        l_plus_r = ((l_plus_r > 0)? 1: -1) * DBL_EPSILON;
    }
    if (this->report.motion.spin == (uint8_t)(three_one_feedback::spin_status::not_spin)) {
        if (this->pack_one.gear == (uint8_t)three_one_feedback::current_gear::R) {
            this->report.motion.curvature = l_minus_r / l_plus_r / 0.85;
        } else {
            this->report.motion.curvature = -l_minus_r / l_plus_r / 0.85;
        }
    } else {
        if (this->report.motion.spin == (uint8_t)(three_one_feedback::spin_status::counterclockwise)) {
            this->report.motion.curvature = FLT_MAX;
        } else {
            this->report.motion.curvature = -FLT_MAX;
        }
    }
}

void DataUpload::prepareSend(shawn::handle p_handle) {
    switch (p_handle.getID()) {
        case 0: {
            memcpy(this->data_to_send, this->pack_one.pack, sizeof(this->pack_one.pack));
            break;
        }
        case 1: {
            memcpy(this->data_to_send, this->pack_two.pack, sizeof(this->pack_two.pack));
            break;
        }
        case 2: {
            memcpy(this->data_to_send, this->pack_three.pack, sizeof(this->pack_three.pack));
            break;
        }
        case 3: {
            memcpy(this->data_to_send, this->pack_four.pack, sizeof(this->pack_four.pack));
            break;
        }
        case 4: {
            memcpy(this->data_to_send, this->pack_five.pack, sizeof(this->pack_five.pack));
            break;
        }
        case 5: {
            memcpy(this->data_to_send, this->pack_six.pack, sizeof(this->pack_six.pack));
            break;
        }
        case 6: {
            memcpy(this->data_to_send, this->pack_seven.pack, sizeof(this->pack_seven.pack));
            break;
        }
        default: {
            break;
        }
    }
}

}