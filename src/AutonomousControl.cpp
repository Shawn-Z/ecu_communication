#include "AutonomousControl.hpp"

namespace ecu_communication {

void AutonomousControl::init(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle,
                             DataDownload *p_data_download, DataUpload *p_data_upload, std::mutex *p_data_upload_mutex) {
    this->nh_ = node_handle;
    this->private_nh_ = private_node_handle;
    this->p_data_download_ = p_data_download;
    this->p_data_upload_ = p_data_upload;
    this->p_data_upload_mutex_ = p_data_upload_mutex;
    this->setHandles();
}

void AutonomousControl::receive_init() {
    this->speed_sub_ = this->nh_.subscribe<speed_ctrl_msgs::speed_ctrl>("/speed_plan", 1, &AutonomousControl::speedCb, this);
}

void AutonomousControl::send_init() {
    this->data_process_timer_ = this->nh_.createTimer(ros::Duration(PUBLISH_PERIOD),
                                                      boost::bind(&AutonomousControl::dataProcess, this));
}

void AutonomousControl::dataProcess() {
    static ros::Publisher publisher = this->nh_.advertise<three_one_msgs::report>("/ecudatareport", 1);
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
    this->reportControlData();
    publisher.publish(this->p_data_upload_->report);
}

bool AutonomousControl::rosmsgUpdateCheck() {
    //// todo modify check
    bool speed_check = true;
    bool steer_check = true;
    speed_check = this->msg_update_times.checkSingleTimestampTillNow(this->speed_sub_handle_, -1, -1);
    steer_check = this->msg_update_times.checkSingleTimestampTillNow(this->steer_sub_handle_, -1, -1);
    return speed_check && steer_check;
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
    this->p_data_upload_->report.control.rpm = (uint16_t)(fabs(this->p_data_upload_->report.control.speed / this->p_data_upload_->rpm_to_speed));
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

void AutonomousControl::speedCb(speed_ctrl_msgs::speed_ctrl msg) {
    if (!this->receive_switch_) {
        return;
    }
    this->p_data_download_->pack_one.expect_vehicle_speed = (uint8_t)(msg.issue_v * 10 + 0.001);
    this->p_data_download_->pack_one.vehicle_gear = msg.direction;
    this->msg_update_times.pushTimestamp(this->speed_sub_handle_);
}

void AutonomousControl::setHandles() {
    this->speed_sub_handle_ = this->msg_update_times.time_handle.newHandle("check the period of speed");
    this->steer_sub_handle_ = this->msg_update_times.time_handle.newHandle("check the period of steer");
}

}