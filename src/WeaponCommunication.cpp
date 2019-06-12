#include "WeaponCommunication.hpp"

namespace ecu_communication {
void WeaponCommunication::init(ecu_communication::DataUpload *p_data_upload, std::mutex *p_data_upload_mutex,
                               shawn::SLog *p_log, sensor_driver_msgs::VehicleState *p_gps, weapon::cmd *p_weapon_cmd) {
    this->p_data_upload_ = p_data_upload;
    this->p_data_upload_mutex_ = p_data_upload_mutex;
    this->p_log_ = p_log;
    this->p_gps_ = p_gps;
    this->p_weapon_cmd_ = p_weapon_cmd;
    while (!this->udp_.init()) {
        ROS_INFO_STREAM_THROTTLE(1.2, "udp with remote init error, keep trying");
    }
    this->send_switch_ = true;
}

void WeaponCommunication::dataSend() {
    if (!this->send_switch_) {
        return;
    }
    this->weapon_send_.pack_handle.setID(1);
    this->p_data_upload_mutex_->lock();
    size_t send_len = this->weapon_send_.prepareSend(this->p_data_upload_, *this->p_gps_);
    this->p_data_upload_mutex_->unlock();
    if (!this->udp_.sendToRemote(this->weapon_send_.data_to_send, send_len)) {
        LOG_ERROR << "weapon send length error: " << this->udp_.get_send_len() << ". raw data as following:";
        this->p_log_->logUint8Array((char *)this->weapon_send_.data_to_send, send_len, google::ERROR);
    }
}

void WeaponCommunication::cmdSend() {
    if (!this->send_switch_) {
        return;
    }
    this->weapon_send_.pack_control.cmd = (uint8_t)(*this->p_weapon_cmd_);
    if (!this->udp_.sendToRemote(this->weapon_send_.pack_control.pack, 5)) {
        LOG_ERROR << "weapon send length error: " << this->udp_.get_send_len() << ". raw data as following:";
        this->p_log_->logUint8Array((char *)this->weapon_send_.data_to_send, 5, google::ERROR);
    }
}

}