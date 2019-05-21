#include <AutonomousControl.hpp>
#include "AutonomousControl.hpp"

namespace ecu_communication {

void AutonomousControl::init(ros::NodeHandle node_handle,
                             DataDownload *p_data_download, DataUpload *p_data_upload,
                             std::mutex *p_data_upload_mutex, std::mutex *p_data_download_mutex,
                             three_one_feedback::control_mode *p_control_mode, std::mutex *p_control_mode_mutex,
                             sensor_driver_msgs::VehicleState *p_gps) {
    this->nh_ = node_handle;
    this->p_data_download_ = p_data_download;
    this->p_data_upload_ = p_data_upload;
    this->p_data_upload_mutex_ = p_data_upload_mutex;
    this->p_data_download_mutex_ = p_data_download_mutex;
    this->p_control_mode_ = p_control_mode;
    this->p_control_mode_mutex_ = p_control_mode_mutex;
    this->p_gps_ = p_gps;
    this->setHandles();
    this->msg_priority.total.current = -1;
    this->msg_priority.total.spare = -1;
    this->msg_priority.speed.current = 0;
    this->msg_priority.speed.spare = 0;
    this->msg_priority.steer.current = 0;
    this->msg_priority.steer.spare = 0;
    this->priority_check_timer_ = this->nh_.createTimer(ros::Duration(MSG_PRIORITY_CHECK_PERIOD), boost::bind(&AutonomousControl::priorityCheck, this));
    this->gpsInit();
}

void AutonomousControl::setHandles() {
    this->speed_sub_handle_ = this->msg_update_times.time_handle.newHandle("check the period of speed");
    this->steer_sub_handle_ = this->msg_update_times.time_handle.newHandle("check the period of steer");
    this->gps_sub_handle_ = this->msg_update_times.time_handle.newHandle("check the period of gps");
}

void AutonomousControl::receive_init() {
    this->speed_sub_ = this->nh_.subscribe<three_one_msgs::ControlSpeed>("/speed_plan", 1, &AutonomousControl::speedCb, this);
    this->steer_sub_ = this->nh_.subscribe<three_one_msgs::ControlSteer>("/steer_cmd", 1, &AutonomousControl::steerCb, this);
    this->gps_sub_ = this->nh_.subscribe<sensor_driver_msgs::VehicleState>("/vehiclestate", 1, &AutonomousControl::gpsCb, this);
    this->suspension_sub_ = this->nh_.subscribe<three_one_msgs::ControlSuspension>("/suspension", 1, &AutonomousControl::suspensionCb, this);
}

void AutonomousControl::dataProcess() {
    static ros::Publisher publisher = this->nh_.advertise<three_one_msgs::Report>("/ecudatareport", 1);
    if (!this->send_switch_) {
        return;
    }
    this->p_data_upload_mutex_->lock();
    if (!this->p_data_upload_->dataCheck()) {
        this->p_data_upload_mutex_->unlock();
        LOG_ERROR << "data to publish check failure";
        return;
    }
    this->p_data_upload_->dataToMsg();
    this->p_data_upload_mutex_->unlock();
    this->p_data_download_mutex_->lock();
    this->reportControlData();
    this->p_data_download_mutex_->unlock();
    this->p_control_mode_mutex_->lock();
    this->p_data_upload_->report.control_mode = (uint8_t)(*this->p_control_mode_);
    this->p_control_mode_mutex_->unlock();
    publisher.publish(this->p_data_upload_->report);
}

void AutonomousControl::reportControlData() {
    this->p_data_upload_->report.control.curvature = this->p_data_download_->pack_one.thousand_times_curvature / 1000.0;
    this->p_data_upload_->report.control.speed = this->p_data_download_->pack_one.expect_vehicle_speed / 10.0;
    if (this->p_data_download_->pack_one.vehicle_gear == (int)three_one_control::vehicle_gear::R) {
        this->p_data_upload_->report.control.speed = -this->p_data_upload_->report.control.speed;
    }
    if (this->p_data_download_->pack_one.vehicle_gear == (int)three_one_control::vehicle_gear::N) {
        this->p_data_upload_->report.control.speed = 0;
    }
    this->p_data_upload_->report.control.rpm = (uint16_t)(fabs(this->p_data_upload_->report.control.speed / RPM_TO_SPEED));
    this->p_data_upload_->report.control.work_mode = this->p_data_download_->pack_one.work_mode;
    this->p_data_upload_->report.control.gear = this->p_data_download_->pack_one.vehicle_gear;
    this->p_data_upload_->report.control.turn_to = this->p_data_download_->pack_one.vehicle_turn_to;
    this->p_data_upload_->report.control.brake = this->p_data_download_->pack_two.brake;
    this->p_data_upload_->report.control.park = this->p_data_download_->pack_one.parking_control;

    this->p_data_upload_->report.control.cylinder_select = this->p_data_download_->pack_two.cylinder_select;
    this->p_data_upload_->report.control.suspension_select = this->p_data_download_->pack_two.suspension_select;
    this->p_data_upload_->report.control.vertical_wall_mode = this->p_data_download_->pack_two.vertical_wall_mode;
    this->p_data_upload_->report.control.suspension_work_mode = this->p_data_download_->pack_two.suspension_work_mode;
    this->p_data_upload_->report.control.suspension_work_mode_detail = this->p_data_download_->pack_two.suspension_work_mode_detail;
    this->p_data_upload_->report.control.suspension_cylinder_select_mode = this->p_data_download_->pack_two.suspension_cylinder_select_mode;
    this->p_data_upload_->report.control.suspension_cylinder_motor_control = this->p_data_download_->pack_two.suspension_cylinder_motor_control;
    this->p_data_upload_->report.control.fix_two_chamber_valve = this->p_data_download_->pack_two.fix_two_chamber_valve;
}

bool AutonomousControl::rosmsgUpdateCheck() {
    //// todo modify check
    bool speed_check = true;
    bool steer_check = true;
    speed_check = this->msg_update_times.checkSingleTimestampTillNow(this->speed_sub_handle_, -1, 500);
    steer_check = this->msg_update_times.checkSingleTimestampTillNow(this->steer_sub_handle_, -1, 500);
    return speed_check && steer_check;
}

void AutonomousControl::speedCb(three_one_msgs::ControlSpeed msg) {
    if (!this->receive_switch_) {
        this->msg_update_times.pushTimestamp(this->speed_sub_handle_);
        return;
    }
    if (msg.priority >= this->msg_priority.speed.current) {
        if (msg.priority > this->msg_priority.speed.current) {
            this->msg_priority.speed.spare = this->msg_priority.speed.current;
            this->msg_priority.speed.current = msg.priority;
        }
    } else {
        this->msg_priority.speed.spare = std::max(msg.priority, this->msg_priority.speed.spare);
        return;
    }
    this->p_data_download_mutex_->lock();
    this->p_data_download_->pack_one.expect_vehicle_speed = (uint8_t)round(msg.speed * 10.0);
    this->p_data_download_->pack_one.vehicle_gear = msg.gear;
    if (msg.halt != (uint8_t)three_one_control::halt::off) {
        this->p_data_download_->pack_one.work_mode = (uint8_t)three_one_control::work_mode::halt;
    } else {
        this->p_data_download_->pack_one.work_mode = (uint8_t)three_one_control::work_mode::curvature_and_vehicle_speed;
    }
    this->p_data_download_mutex_->unlock();
    this->msg_update_times.pushTimestamp(this->speed_sub_handle_);
}

void AutonomousControl::steerCb(three_one_msgs::ControlSteer msg) {
    if (!this->receive_switch_) {
        this->msg_update_times.pushTimestamp(this->steer_sub_handle_);
        return;
    }
    if (msg.priority >= this->msg_priority.steer.current) {
        if (msg.priority > this->msg_priority.steer.current) {
            this->msg_priority.steer.spare = this->msg_priority.steer.current;
            this->msg_priority.steer.current = msg.priority;
        }
    } else {
        this->msg_priority.steer.spare = std::max(msg.priority, this->msg_priority.steer.spare);
        return;
    }
    this->p_data_download_mutex_->lock();
    if (this->p_data_download_->pack_one.vehicle_gear == (uint8_t)(three_one_control::vehicle_gear::R)) {
        this->p_data_download_->pack_one.vehicle_turn_to = (msg.curvature < 0)?
                                                           ((uint8_t)three_one_control::vehicle_turn_to::left): ((uint8_t)three_one_control::vehicle_turn_to::right);
    } else {
        this->p_data_download_->pack_one.vehicle_turn_to = (msg.curvature > 0)?
                                                           ((uint8_t)three_one_control::vehicle_turn_to::left): ((uint8_t)three_one_control::vehicle_turn_to::right);
    }
    this->p_data_download_->pack_one.thousand_times_curvature = (uint16_t)round(fabs(msg.curvature) * 1000.0);
    this->p_data_download_mutex_->unlock();
    this->msg_update_times.pushTimestamp(this->steer_sub_handle_);
}

void AutonomousControl::priorityCheck() {
    bool speed_check = true;
    bool steer_check = true;
    bool master_check = true;

    //// todo not write master control now
    master_check = false;
    if (!master_check) {
        this->msg_priority.total.current = this->msg_priority.total.spare;
        this->msg_priority.total.spare = -1;
    }
    if (this->msg_priority.total.current != -1) {
        this->msg_priority.speed.spare = this->msg_priority.speed.current;
        this->msg_priority.speed.current = 255;
        this->msg_priority.steer.spare = this->msg_priority.steer.current;
        this->msg_priority.steer.current = 255;
        return;
    }
    speed_check = this->msg_update_times.checkSingleTimestampTillNow(this->speed_sub_handle_, -1, 500);
    steer_check = this->msg_update_times.checkSingleTimestampTillNow(this->steer_sub_handle_, -1, 500);
    if (!speed_check) {
        this->msg_priority.speed.current = this->msg_priority.speed.spare;
        this->msg_priority.speed.spare = 0;
    }
    if (!steer_check) {
        this->msg_priority.steer.current = this->msg_priority.steer.spare;
        this->msg_priority.steer.spare = 0;
    }
}

void AutonomousControl::gpsCb(sensor_driver_msgs::VehicleState msg) {
    *this->p_gps_ = msg;
    this->msg_update_times.pushTimestamp(this->gps_sub_handle_);
}

bool AutonomousControl::gpsCheck() {
    return this->msg_update_times.checkSingleTimestampTillNow(this->gps_sub_handle_, -1, 500);
}

void AutonomousControl::gpsInit() {
    this->p_gps_->linear_velocity.x = 0.0;
    this->p_gps_->linear_velocity.y = 0.0;
    this->p_gps_->linear_velocity.z = 0.0;
    this->p_gps_->gps.latitude = 0.0;
    this->p_gps_->gps.longitude = 0.0;
    this->p_gps_->gps.altitude = 0.0;
    this->p_gps_->yaw = 0.0;
    this->p_gps_->roll = 0.0;
    this->p_gps_->pitch = 0.0;
}

void AutonomousControl::suspensionCb(three_one_msgs::ControlSuspension msg) {
    this->p_data_download_mutex_->lock();
    this->p_data_download_->pack_two.cylinder_select = msg.cylinder_select;
    this->p_data_download_->pack_two.suspension_select = msg.suspension_select;
    this->p_data_download_->pack_two.suspension_work_mode = msg.suspension_work_mode;
    this->p_data_download_->pack_two.suspension_work_mode_detail = msg.suspension_work_mode_detail;
    this->p_data_download_->pack_two.suspension_cylinder_select_mode = msg.suspension_cylinder_select_mode;
    this->p_data_download_->pack_two.suspension_cylinder_motor_control = msg.suspension_cylinder_motor_control;
    this->p_data_download_->pack_two.vertical_wall_mode = msg.vertical_wall_mode;
    this->p_data_download_->pack_two.fix_two_chamber_valve = msg.fix_two_chamber_valve;
    this->p_data_download_->pack_two.entrenchment = msg.entrenchment;
    this->p_data_download_mutex_->unlock();
}

}