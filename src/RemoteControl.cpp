#include "RemoteControl.hpp"

namespace ecu_communication {

RemoteControl::RemoteControl() {
}

void RemoteControl::init(DataDownload *p_data_download, DataUpload *p_data_upload,
                         std::mutex *p_data_download_mutex, std::mutex *p_data_upload_mutex,
                         shawn::SLog *p_log, three_one_feedback::control_mode *p_control_mode, std::mutex *p_control_mode_mutex) {
    this->p_data_download_ = p_data_download;
    this->p_data_upload_ = p_data_upload;
    this->p_data_download_mutex_ = p_data_download_mutex;
    this->p_data_upload_mutex_ = p_data_upload_mutex;
    this->p_log_ = p_log;
    this->p_control_mode_ = p_control_mode;
    this->p_control_mode_mutex_ = p_control_mode_mutex;
    this->setHandles();
    while (!this->udp_.init()) {
        ROS_ERROR_STREAM("udp with remote init error, keep trying");
    }
}

void RemoteControl::setHandles() {
    this->udp_recv_handle_ = this->udp_recv_times_.time_handle.newHandle("udp receive from remote");
    this->udp_recv_correct_handle_ = this->udp_recv_times_.time_handle.newHandle("udp receive correct from remote");
    this->pack1_recv_handle_ = this->pack_recv_times_.time_handle.newHandle("pack 1 receive from remote");
    this->pack2_recv_handle_ = this->pack_recv_times_.time_handle.newHandle("pack 2 receive from remote");
}

void RemoteControl::dataReceive() {
    while (ros::ok()) {
//        if (!this->receive_switch_) {
//            continue;
//        }
        this->udp_.recv();
        this->udp_recv_times_.pushTimestamp(this->udp_recv_handle_);
        if ((this->udp_.get_recv_len() > 512) || (this->udp_.get_recv_len() < 1)) {
            continue;
            LOG_ERROR << "remote receive length error: " << this->udp_.get_recv_len();
            continue;
        }
        this->udp_recv_times_.pushTimestamp(this->udp_recv_correct_handle_);
        if (!this->remoteReceive_.receiveIDCheck((char *)this->udp_.buffer, this->udp_.get_recv_len())) {
            continue;
            LOG_ERROR << "dataID for remote illegal, receive raw data as following:";
            this->p_log_->logUint8Array((char *)this->udp_.buffer, this->udp_.get_recv_len(), google::ERROR);
            continue;
        }
        this->pack_recv_times_.pushTimestamp(this->remoteReceive_.pack_handle);
        this->p_control_mode_mutex_->lock();
        this->setControlMode();
        this->p_control_mode_mutex_->unlock();
        if (!this->receive_switch_) {
            continue;
        }
        this->p_data_download_mutex_->lock();
        this->remoteReceive_.dataDistribution((char *)this->udp_.buffer, this->p_data_download_);
        this->p_data_download_mutex_->unlock();
    }
}

void RemoteControl::dataSend() {
    this->p_control_mode_mutex_->lock();
    if ((*this->p_control_mode_) == three_one_feedback::control_mode::ERROR) {
        this->p_control_mode_mutex_->unlock();
        uint8_t connect_data[] = {0xF1, 0x00};
        this->udp_.sendToRemote(connect_data, 2);
        return;
    }
    this->p_control_mode_mutex_->unlock();
    static std::vector<uint8_t> ID_set{0,2,3,1,4,5,6,1,7,8,1,2,3,4,1,5,6,1,7,8,2,1,3,4,1,5,6,7,1,8,2,1,3,4,5,1,6,7,1,8,2,3,1,4,5,1,6,7,8,1};
    static int counter = -1;
    if (!this->send_switch_) {
        return;
    }
    ++counter;
    if (counter == 50) {
        counter = 0;
    }
    this->remoteSend_.pack_handle.setID(ID_set[counter]);
    this->p_data_upload_mutex_->lock();
    this->p_control_mode_mutex_->lock();
    size_t send_len = this->remoteSend_.prepareSend(this->p_data_upload_, this->p_control_mode_);
    this->p_data_upload_mutex_->unlock();
    this->p_control_mode_mutex_->unlock();
    if (!this->udp_.sendToRemote(this->remoteSend_.data_to_send, send_len)) {
        LOG_ERROR << "remote send length error: " << this->udp_.get_send_len() << ". raw data as following:";
        this->p_log_->logUint8Array((char *)this->remoteSend_.data_to_send, send_len, google::ERROR);
    }
}

void RemoteControl::setControlMode() {
    if (this->remoteReceive_.pack_handle.getID() == 1) {
        switch (this->udp_.buffer[9]) {
            case 1: {
                (*this->p_control_mode_) = three_one_feedback::control_mode::remote;
                break;
            }
            case 2: {
                (*this->p_control_mode_) = three_one_feedback::control_mode::autonomous;
                break;
            }
            default: {
                (*this->p_control_mode_) = three_one_feedback::control_mode::ERROR;
                break;
            }
        }
    }
}

bool RemoteControl::time_check() {
    bool udp_recv_duration_check = true;
    bool udp_recv_till_now_check = true;
    bool pack_recv_duration_check = true;
    bool pack_recv_till_now_check = true;
    //// todo modify parameter of check
//    udp_recv_duration_check = this->udp_recv_times_.checkTimestampsDuration(-1, -1);
    udp_recv_till_now_check = this->udp_recv_times_.checkTimestampsTillNow(-1, 800);
//    pack_recv_duration_check = this->pack_recv_times_.checkTimestampsDuration(-1, -1);
//    pack_recv_till_now_check = this->pack_recv_times_.checkTimestampsTillNow(-1, -1);

    return (udp_recv_duration_check && udp_recv_till_now_check && pack_recv_duration_check && pack_recv_till_now_check);
}

}


//    ROS_WARN_STREAM(this->remoteSend_.pack_handle.getID());
//    static size_t counter = 0;
//    bool send = false;
//    if ((counter % 100) == 0) {
//        this->remoteSend_.pack_handle.setID(0);
//        send = true;
//    }
//    if ((counter % 20) == 4) {
//        this->remoteSend_.pack_handle.setID(2);
//        send = true;
//    }
//    if ((counter % 20) == 6) {
//        this->remoteSend_.pack_handle.setID(3);
//        send = true;
//    }
//    if ((counter % 20) == 8) {
//        this->remoteSend_.pack_handle.setID(4);
//        send = true;
//    }
//    if ((counter % 20) == 10) {
//        this->remoteSend_.pack_handle.setID(5);
//        send = true;
//    }
//    if ((counter % 20) == 12) {
//        this->remoteSend_.pack_handle.setID(6);
//        send = true;
//    }
//    if ((counter % 20) == 14) {
//        this->remoteSend_.pack_handle.setID(7);
//        send = true;
//    }
//    if ((counter % 20) == 18) {
//        this->remoteSend_.pack_handle.setID(8);
//        send = true;
//    }
//    if ((counter % 7) == 2) {
//        this->remoteSend_.pack_handle.setID(1);
//        send = true;
//    }
//    ++counter;
//    if (counter >= 100) {
//        counter = 0;
//    }
//    if (!send) {
//        return;
//    }