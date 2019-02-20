#include "RemoteSend.hpp"


namespace ecu_communication {
    size_t RemoteSend::prepareSend(DataUpload *p_data_upload, uint8_t *p_work_mode) {
        size_t send_len = 0;
        switch (this->pack_handle.getID()) {
            case 0: {
                this->pack_fuck.ID_one = 0xF1;
                this->pack_fuck.ID_two = 0xBC;
                this->pack_fuck.gps_week;
                this->pack_fuck.gps_ms;
                this->pack_fuck.SOC = p_data_upload->pack_two.SOC;
                if ((*p_work_mode) == 2) {
                    this->pack_fuck.controlled_state = 1;
                } else {
                    this->pack_fuck.controlled_state = 0;
                }
                this->pack_fuck.terminal_ID;
                this->pack_fuck.check_sum;
                send_len = sizeof(this->pack_fuck.pack);
                memcpy(this->data_to_send, this->pack_fuck.pack, send_len);
                break;
            }
            case 1: {
                this->pack_one.ID_one = 0xF1;
                this->pack_one.ID_two = 0x00;
                this->pack_one.gear = p_data_upload->pack_one.gear;
                this->pack_one.park_status = p_data_upload->pack_seven.park_status;
                this->pack_one.mechanical_brake = p_data_upload->pack_one.mechanical_brake;
                this->pack_one.vehicle_speed = p_data_upload->pack_one.vehicle_speed;
                this->pack_one.mileage = p_data_upload->pack_five.mileage;
                this->pack_one.error_code = p_data_upload->pack_six.error_code;
                this->pack_one.left_wheel_expect_speed = p_data_upload->pack_one.left_wheel_expect_speed;
                this->pack_one.right_wheel_expect_speed = p_data_upload->pack_one.right_wheel_expect_speed;
                this->pack_one.vehicle_height = p_data_upload->pack_seven.vehicle_height;
                this->pack_one.latitude;
                this->pack_one.longitude;
                this->pack_one.altitude;
                this->pack_one.yaw;
                this->pack_one.roll;
                this->pack_one.pitch;
                this->pack_one.north_speed;
                this->pack_one.east_speed;
                this->pack_one.up_speed;
                this->pack_one.SOC = p_data_upload->pack_two.SOC;
                this->pack_one.reserve_byte1 = 0;
                send_len = sizeof(this->pack_one.pack);
                memcpy(this->data_to_send, this->pack_one.pack, send_len);
                break;
            }
            case 2: {
                this->pack_two.ID_one = 0xF1;
                this->pack_two.ID_two = 0x01;
                this->pack_two.left_wheel_expect_speed = p_data_upload->pack_one.left_wheel_expect_speed;
                this->pack_two.mechanical_brake = p_data_upload->pack_one.mechanical_brake;
                this->pack_two.right_wheel_expect_speed = p_data_upload->pack_one.right_wheel_expect_speed;
                this->pack_two.vehicle_speed = p_data_upload->pack_one.vehicle_speed;
                this->pack_two.gear = p_data_upload->pack_one.gear;
                send_len = sizeof(this->pack_two.pack);
                memcpy(this->data_to_send, this->pack_two.pack, send_len);
                break;
            }
            case 3: {
                this->pack_three.ID_one = 0xF1;
                this->pack_three.ID_two = 0x02;
                this->pack_three.left_motor_actual_speed = p_data_upload->pack_two.left_motor_actual_speed;
                this->pack_three.left_motor_gear = p_data_upload->pack_two.left_motor_gear;
                this->pack_three.right_motor_gear = p_data_upload->pack_two.right_motor_gear;
                this->pack_three.right_motor_actual_speed = p_data_upload->pack_two.right_motor_actual_speed;
                this->pack_three.SOC = p_data_upload->pack_two.SOC;
                this->pack_three.tailgate_state = p_data_upload->pack_two.tailgate_state;
                send_len = sizeof(this->pack_three.pack);
                memcpy(this->data_to_send, this->pack_three.pack, send_len);
                break;
            }
            case 4: {
                this->pack_four.ID_one = 0xF1;
                this->pack_four.ID_two = 0x03;
                this->pack_four.left_one_cylinder_position = p_data_upload->pack_three.left_one_cylinder_position;
                this->pack_four.left_two_cylinder_position = p_data_upload->pack_three.left_two_cylinder_position;
                this->pack_four.left_three_cylinder_position = p_data_upload->pack_three.left_three_cylinder_position;
                this->pack_four.left_four_cylinder_position = p_data_upload->pack_three.left_four_cylinder_position;
                this->pack_four.right_one_cylinder_position = p_data_upload->pack_three.right_one_cylinder_position;
                this->pack_four.right_two_cylinder_position = p_data_upload->pack_three.right_two_cylinder_position;
                this->pack_four.right_three_cylinder_position = p_data_upload->pack_three.right_three_cylinder_position;
                this->pack_four.right_four_cylinder_position = p_data_upload->pack_three.right_four_cylinder_position;
                send_len = sizeof(this->pack_four.pack);
                memcpy(this->data_to_send, this->pack_four.pack, send_len);
                break;
            }
            case 5: {
                this->pack_five.ID_one = 0xF1;
                this->pack_five.ID_two = 0x04;
                this->pack_five.left_one_cylinder_pressure = p_data_upload->pack_four.left_one_cylinder_pressure;
                this->pack_five.left_two_cylinder_pressure = p_data_upload->pack_four.left_two_cylinder_pressure;
                this->pack_five.left_three_cylinder_pressure = p_data_upload->pack_four.left_three_cylinder_pressure;
                this->pack_five.left_four_cylinder_pressure = p_data_upload->pack_four.left_four_cylinder_pressure;
                this->pack_five.right_one_cylinder_pressure = p_data_upload->pack_four.right_one_cylinder_pressure;
                this->pack_five.right_two_cylinder_pressure = p_data_upload->pack_four.right_two_cylinder_pressure;
                this->pack_five.right_three_cylinder_pressure = p_data_upload->pack_four.right_three_cylinder_pressure;
                this->pack_five.right_four_cylinder_pressure = p_data_upload->pack_four.right_four_cylinder_pressure;
                send_len = sizeof(this->pack_five.pack);
                memcpy(this->data_to_send, this->pack_five.pack, send_len);
                break;
            }
            case 6: {
                this->pack_six.ID_one = 0xF1;
                this->pack_six.ID_two = 0x05;
                this->pack_six.mileage = p_data_upload->pack_five.mileage;
                this->pack_six.vehicle_roll = p_data_upload->pack_five.vehicle_roll;
                this->pack_six.vehicle_pitch = p_data_upload->pack_five.vehicle_pitch;
                this->pack_six.left_torque = p_data_upload->pack_five.left_torque;
                send_len = sizeof(this->pack_six.pack);
                memcpy(this->data_to_send, this->pack_six.pack, send_len);
                break;
            }
            case 7: {
                this->pack_seven.ID_one = 0xF1;
                this->pack_seven.ID_two = 0x06;
                this->pack_seven.right_torque = p_data_upload->pack_six.right_torque;
                this->pack_seven.vertical_wall_status = p_data_upload->pack_six.vertical_wall_status;
                this->pack_seven.error_code = p_data_upload->pack_six.error_code;
                this->pack_seven.left_pulse = p_data_upload->pack_six.left_pulse;
                send_len = sizeof(this->pack_seven.pack);
                memcpy(this->data_to_send, this->pack_seven.pack, send_len);
                break;
            }
            case 8: {
                this->pack_eight.ID_one = 0xF1;
                this->pack_eight.ID_two = 0x07;
                this->pack_eight.right_pulse = p_data_upload->pack_seven.right_pulse;
                this->pack_eight.vehicle_height = p_data_upload->pack_seven.vehicle_height;
                this->pack_eight.park_status = p_data_upload->pack_seven.park_status;
                this->pack_eight.reserve_bytes1[0] = 0;
                this->pack_eight.reserve_bytes1[1] = 0;
                send_len = sizeof(this->pack_eight.pack);
                memcpy(this->data_to_send, this->pack_eight.pack, send_len);
                break;
            }
            default: {
                break;
            }
        }
        return send_len;
    }

}