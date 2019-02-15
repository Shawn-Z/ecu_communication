#include "RemoteControl.hpp"

namespace ecu_communication {

void RemoteControl::receiveInit(uint16_t p_server_port) {
    this->server_port_ = p_server_port;
}

void RemoteControl::sendInit(std::string p_remote_ip, uint16_t p_remote_port) {
    while (!(this->udp_client_.init(p_remote_ip.data(), p_remote_port))) {
        ROS_ERROR_STREAM("UDP SEND TO REMOTE INIT FAILURE, KEEP TRYING!");
    }
    this->udp_send_proportion_.inputProportion(1, 1, 1, 1, 1, 1, 1);
    this->data_send_timer_ = this->nh_.createTimer(ros::Duration(REMOTE_SEND_PERIOD),
                                                  boost::bind(&RemoteControl::dataSend, this));
}

void RemoteControl::init(ros::NodeHandle node_handle, DataDownload *p_data_download, DataUpload *p_data_upload,
                         std::mutex *p_data_download_mutex, std::mutex *p_data_upload_mutex, shawn::SLog *p_log) {
    this->nh_ = node_handle;
    this->p_data_download_ = p_data_download;
    this->p_data_upload_ = p_data_upload;
    this->p_data_download_mutex_ = p_data_download_mutex;
    this->p_data_upload_mutex_ = p_data_upload_mutex;
    this->p_log_ = p_log;
}

void RemoteControl::dataReceive() {
    while (!(this->udp_server_.init(this->server_port_))) {
        ROS_ERROR_STREAM("UDP RECEIVE FROM REMOTE INIT FAILURE, KEEP TRYING!");
    }
    while (ros::ok()) {
        if (this->receive_switch_) {
            continue;
        }
        this->udp_server_.process();
        this->udp_recv_times_.pushTimestamp(this->udp_recv_handle_);
        if (this->udp_server_.get_recv_len() != 14) {
            if (this->udp_server_.get_recv_len() > 0) {
                LOG_ERROR << "remote receive length error: " << this->udp_server_.get_recv_len() << ". raw data as following:";
                this->p_log_->logUint8Array((char *)this->udp_server_.buffer, this->udp_server_.get_recv_len(), google::ERROR);
            } else {
                LOG_ERROR << "remote receive length error: " << this->udp_server_.get_recv_len();
            }
            continue;
        }
        this->udp_recv_times_.pushTimestamp(this->udp_recv_correct_handle_);
        if (!this->p_data_download_->dataIDCheck((char *)this->udp_server_.buffer)) {
            LOG_ERROR << "dataID for remote illegal, receive raw data as following:";
            this->p_log_->logUint8Array((char *)this->udp_server_.buffer, this->udp_server_.get_recv_len(), google::ERROR);
            continue;
        }
        this->setWorkMode();
        this->pack_recv_times_.pushTimestamp(this->p_data_download_->pack_handle);
        this->p_data_download_mutex_->lock();
        this->p_data_download_->dataDistribution();
        this->p_data_download_mutex_->unlock();
    }
}

void RemoteControl::dataSend() {
    if (this->send_switch_) {
        return;
    }
    this->udp_pack_handle_.setID(this->udp_send_proportion_.stepping());
    this->p_data_upload_mutex_->lock();
    this->p_data_upload_->prepareSend(this->udp_pack_handle_);
    this->p_data_upload_mutex_->unlock();
    if (!this->udp_client_.process(this->p_data_upload_->data_to_send, sizeof(this->p_data_upload_->data_to_send))) {
        LOG_ERROR << "remote send length error: " << this->udp_client_.get_send_len() << ". raw data as following:";
        this->p_log_->logUint8Array((char *)this->p_data_upload_->data_to_send, sizeof(this->p_data_upload_->data_to_send), google::ERROR);
    }
}

bool RemoteControl::time_check() {
    bool udp_recv_duration_check = true;
    bool udp_recv_till_now_check = true;
    bool pack_recv_duration_check = true;
    bool pack_recv_till_now_check = true;
    //// todo modify parameter of check
    udp_recv_duration_check = this->udp_recv_times_.checkTimestampsDuration(-1, -1);
    udp_recv_till_now_check = this->udp_recv_times_.checkTimestampsTillNow(-1, -1);
    pack_recv_duration_check = this->pack_recv_times_.checkTimestampsDuration(-1, -1);
    pack_recv_till_now_check = this->pack_recv_times_.checkTimestampsTillNow(-1, -1);

    return (udp_recv_duration_check && udp_recv_till_now_check && pack_recv_duration_check && pack_recv_till_now_check);
}

void RemoteControl::setWorkMode() {
    if (1 == this->p_data_download_->pack_handle.getID()) {
        this->work_mode_ = this->p_data_download_->recv_raw_data[13];
    }
}

uint8_t RemoteControl::getWorkMode() {
    return this->work_mode_;
}

}