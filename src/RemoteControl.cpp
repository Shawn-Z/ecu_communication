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
    this->setHandles();
}

void RemoteControl::dataReceive() {
    while (!(this->udp_server_.init(this->server_port_))) {
        ROS_ERROR_STREAM("UDP RECEIVE FROM REMOTE INIT FAILURE, KEEP TRYING!");
    }
    while (ros::ok()) {
//        if (!this->receive_switch_) {
//            continue;
//        }
        this->udp_server_.process();
        this->udp_recv_times_.pushTimestamp(this->udp_recv_handle_);
        if ((this->udp_server_.get_recv_len() > 512) || (this->udp_server_.get_recv_len() < 1)) {
            LOG_ERROR << "remote receive length error: " << this->udp_server_.get_recv_len();
            continue;
        }
        this->udp_recv_times_.pushTimestamp(this->udp_recv_correct_handle_);
        if (!this->remoteReceive_.receiveIDCheck((char *)this->udp_server_.buffer, this->udp_server_.get_recv_len())) {
            LOG_ERROR << "dataID for remote illegal, receive raw data as following:";
            this->p_log_->logUint8Array((char *)this->udp_server_.buffer, this->udp_server_.get_recv_len(), google::ERROR);
            continue;
        }
        this->pack_recv_times_.pushTimestamp(this->remoteReceive_.pack_handle);
        this->p_data_download_mutex_->lock();
        this->setWorkMode();
        this->p_data_download_mutex_->unlock();
        if (!this->receive_switch_) {
            continue;
        }
        this->p_data_download_mutex_->lock();
        this->remoteReceive_.dataDistribution((char *)this->udp_server_.buffer, this->p_data_download_);
        this->p_data_download_mutex_->unlock();
    }
}

void RemoteControl::dataSend() {
    if (this->send_switch_) {
        return;
    }
    static size_t counter = 0;
    ++counter;
    if ((counter % 100) == 0) {
        this->remoteSend_.pack_handle.setID(0);
    }
    if ((counter % 7) == 0) {
        this->remoteSend_.pack_handle.setID(1);
    }
    if ((counter % 20) == 2) {
        this->remoteSend_.pack_handle.setID(2);
    }
    if ((counter % 20) == 4) {
        this->remoteSend_.pack_handle.setID(3);
    }
    if ((counter % 20) == 6) {
        this->remoteSend_.pack_handle.setID(4);
    }
    if ((counter % 20) == 8) {
        this->remoteSend_.pack_handle.setID(5);
    }
    if ((counter % 20) == 11) {
        this->remoteSend_.pack_handle.setID(6);
    }
    if ((counter % 20) == 14) {
        this->remoteSend_.pack_handle.setID(7);
    }
    if ((counter % 20) == 17) {
        this->remoteSend_.pack_handle.setID(8);
    }
    this->p_data_upload_mutex_->lock();
    size_t send_len = this->remoteSend_.prepareSend(this->p_data_upload_, this->p_work_mode_);
    this->p_data_upload_mutex_->unlock();
    if (!this->udp_client_.process(this->remoteSend_.data_to_send, send_len)) {
        LOG_ERROR << "remote send length error: " << this->udp_client_.get_send_len() << ". raw data as following:";
        this->p_log_->logUint8Array((char *)this->remoteSend_.data_to_send, send_len, google::ERROR);
    }
}

bool RemoteControl::time_check() {
    bool udp_recv_duration_check = true;
    bool udp_recv_till_now_check = true;
    bool pack_recv_duration_check = true;
    bool pack_recv_till_now_check = true;
    //// todo modify parameter of check
//    udp_recv_duration_check = this->udp_recv_times_.checkTimestampsDuration(-1, -1);
//    udp_recv_till_now_check = this->udp_recv_times_.checkTimestampsTillNow(-1, -1);
//    pack_recv_duration_check = this->pack_recv_times_.checkTimestampsDuration(-1, -1);
//    pack_recv_till_now_check = this->pack_recv_times_.checkTimestampsTillNow(-1, -1);

    return (udp_recv_duration_check && udp_recv_till_now_check && pack_recv_duration_check && pack_recv_till_now_check);
}



uint8_t RemoteControl::getWorkMode() {
    return this->work_mode_;
}

RemoteControl::RemoteControl() {
    this->work_mode_ = DEFAULT_WORK_MODE;
}

void RemoteControl::setHandles() {
    this->udp_recv_handle_ = this->udp_recv_times_.time_handle.newHandle("udp receive from remote");
    this->udp_recv_correct_handle_ = this->udp_recv_times_.time_handle.newHandle("udp receive correct from remote");
    this->pack1_recv_handle_ = this->pack_recv_times_.time_handle.newHandle("pack 1 receive from remote");
    this->pack2_recv_handle_ = this->pack_recv_times_.time_handle.newHandle("pack 2 receive from remote");
}

void RemoteControl::setWorkMode() {
    if (this->remoteReceive_.pack_handle.getID() == 1) {
        this->work_mode_ = this->udp_server_.buffer[9];
    }
}

}