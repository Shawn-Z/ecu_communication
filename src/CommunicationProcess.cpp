#include "CommunicationProcess.hpp"

//// todo manual gui
namespace ecu_communication {
CommunicationProcess::CommunicationProcess(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : reconfigSrv_{private_node_handle}, params_{private_node_handle} {

    this->nh_ = node_handle;
    this->private_nh_ = private_node_handle;

    this->glogInit();
    this->paramsInit();
    this->setTimeCheckHandle();

    this->params_.fake_issue = false;
    this->params_.params_lock = true;
    this->params_.toParamServer();
    if (this->yaml_params_.reconfig) {
        this->reconfigSrv_.setCallback(boost::bind(&CommunicationProcess::reconfigureRequest, this, _1, _2));
    }
    if (this->yaml_params_.publish_rawdata) {
        this->udp_recv_rawdata_publisher_ = this->nh_.advertise<three_one_msgs::RawdataRecv>("/udp_recv_rawdata", 1);
        this->udp_send_rawdata_publisher_ = this->nh_.advertise<three_one_msgs::RawdataSend>("/udp_send_rawdata", 1);
    }

    if (this->yaml_params_.lower_layer_receive || this->yaml_params_.lower_layer_send) {
        while (!this->udp_.init()) {
            ROS_INFO_STREAM_THROTTLE(1.2, "udp with ecu init error, keep trying");
        }
        if (this->yaml_params_.lower_layer_receive) {
            this->udp_receive_thread = std::thread(&CommunicationProcess::udpReceive, this);
            this->udp_receive_thread.detach();
        }
        if (this->yaml_params_.lower_layer_send) {
            this->udp_send_proportion_.inputProportion(1, 1);
            this->udp_send_timer_ = this->nh_.createTimer(ros::Duration(UDP_SEND_PERIOD),
                                                          boost::bind(&CommunicationProcess::udpSend, this));
        }
    }

    if (this->yaml_params_.upper_layer_send || this->yaml_params_.upper_layer_receive) {
        this->autonomousControl_.init(node_handle, &this->data_download_, &this->data_upload_, &this->data_upload_mutex_, &this->data_download_mutex_, &this->control_mode_, &this->control_mode_mutex_);
        if (this->yaml_params_.upper_layer_send) {
            this->ros_publish_timer_ = this->nh_.createTimer(ros::Duration(PUBLISH_PERIOD), boost::bind(&AutonomousControl::dataProcess, &this->autonomousControl_));
        }
        if (this->yaml_params_.upper_layer_receive) {
            this->autonomousControl_.receive_init();
        }
    }

    if (this->yaml_params_.remote_send || this->yaml_params_.remote_receive) {
        this->remoteControl_.init(&this->data_download_, &this->data_upload_, &this->data_download_mutex_, &this->data_upload_mutex_, &this->sLog_, &this->control_mode_, &this->control_mode_mutex_);
        if (this->yaml_params_.remote_send) {
            this->remote_send_timer_ = this->nh_.createTimer(ros::Duration(REMOTE_SEND_PERIOD), boost::bind(&RemoteControl::dataSend, &this->remoteControl_));
        }
        if (this->yaml_params_.remote_receive) {
            this->remote_receive_thread_ = std::thread(&RemoteControl::dataReceive, &this->remoteControl_);
            this->remote_receive_thread_.detach();
        }
    }

    this->time_check_timer_ = this->nh_.createTimer(ros::Duration(CHECK_PERIOD),
                                                    boost::bind(&CommunicationProcess::timeCheck, this));

//    this->control_mode_mutex_.lock();
//    this->control_mode_ = three_one_feedback::control_mode::autonomous;
//    this->control_mode_mutex_.unlock();

//    this->serialPortCommunication.init();
//    this->fuck_timer = this->nh_.createTimer(ros::Duration(0.001), boost::bind(&CommunicationProcess::fuck_send, this));
}

CommunicationProcess::~CommunicationProcess() {
    //// todo
    this->time_check_timer_.stop();

    this->udp_send_switch_ = false;
    this->yaml_params_.reconfig = false;
    this->params_.fake_issue = false;
    this->yaml_params_.send_default_when_no_msg = false;

    //// todo shutdown publisher and subscriber
    this->udp_recv_rawdata_publisher_.shutdown();
    this->udp_send_rawdata_publisher_.shutdown();

    LOG_INFO << "program end";
    LOG_WARN << "program end";
    LOG_ERROR << "program end";
    google::ShutdownGoogleLogging();
    this->serialPortCommunication.colse();
}

void CommunicationProcess::glogInit() {
    this->sLog_.init("ThreeOne", "log_three_one", google::ERROR);
    LOG_INFO << "glog start";
    LOG_WARN << "glog start";
    LOG_ERROR << "glog start";
}

void CommunicationProcess::paramsInit() {
    bool tmp_result = true;
    tmp_result &= this->private_nh_.getParam("upper_layer_send", this->yaml_params_.upper_layer_send);
    tmp_result &= this->private_nh_.getParam("upper_layer_receive", this->yaml_params_.upper_layer_receive);
    tmp_result &= this->private_nh_.getParam("lower_layer_send", this->yaml_params_.lower_layer_send);
    tmp_result &= this->private_nh_.getParam("lower_layer_receive", this->yaml_params_.lower_layer_receive);
    tmp_result &= this->private_nh_.getParam("remote_receive", this->yaml_params_.remote_receive);
    tmp_result &= this->private_nh_.getParam("remote_send", this->yaml_params_.remote_send);

    tmp_result &= this->private_nh_.getParam("ecu_local_ip", this->yaml_params_.ecu_local_ip);
    tmp_result &= this->private_nh_.getParam("ecu_local_port", this->yaml_params_.ecu_local_port);
    tmp_result &= this->private_nh_.getParam("ecu_remote_ip", this->yaml_params_.ecu_remote_ip);
    tmp_result &= this->private_nh_.getParam("ecu_remote_port", this->yaml_params_.ecu_remote_port);

    tmp_result &= this->private_nh_.getParam("remote_local_ip", this->yaml_params_.remote_local_ip);
    tmp_result &= this->private_nh_.getParam("remote_local_port", this->yaml_params_.remote_local_port);
    tmp_result &= this->private_nh_.getParam("remote_remote_ip", this->yaml_params_.remote_remote_ip);
    tmp_result &= this->private_nh_.getParam("remote_remote_port", this->yaml_params_.remote_remote_port);

    tmp_result &= this->private_nh_.getParam("reconfig", this->yaml_params_.reconfig);
    tmp_result &= this->private_nh_.getParam("send_default_when_no_msg", this->yaml_params_.send_default_when_no_msg);
    tmp_result &= this->private_nh_.getParam("log_rawdata", this->yaml_params_.log_rawdata);
    tmp_result &= this->private_nh_.getParam("publish_rawdata", this->yaml_params_.publish_rawdata);

    if (!tmp_result) {
        ROS_ERROR_STREAM("param not retrieved");
        ros::shutdown();
    }

    this->udp_.params.local_ip = this->yaml_params_.ecu_local_ip;
    this->udp_.params.local_port = this->yaml_params_.ecu_local_port;
    this->udp_.params.remote_ip = this->yaml_params_.ecu_remote_ip;
    this->udp_.params.remote_port = this->yaml_params_.ecu_remote_port;

    this->remoteControl_.udp_.params.local_ip = this->yaml_params_.remote_local_ip;
    this->remoteControl_.udp_.params.local_port = this->yaml_params_.remote_local_port;
    this->remoteControl_.udp_.params.remote_ip = this->yaml_params_.remote_remote_ip;
    this->remoteControl_.udp_.params.remote_port = this->yaml_params_.remote_remote_port;

    this->udp_send_switch_ = false;
    //// todo log params
}

void CommunicationProcess::setTimeCheckHandle() {
    //// todo modify ros msg update check handle
    this->udp_recv_handle_ = this->udp_recv_times_.time_handle.newHandle("simple udp receive");
    this->udp_recv_correct_handle_ = this->udp_recv_times_.time_handle.newHandle("udp receive correct");
    this->pack1_recv_handle_ = this->pack_recv_times_.time_handle.newHandle("udp receive pack 1");
    this->pack2_recv_handle_ = this->pack_recv_times_.time_handle.newHandle("udp receive pack 2");
    this->pack3_recv_handle_ = this->pack_recv_times_.time_handle.newHandle("udp receive pack 3");
    this->pack4_recv_handle_ = this->pack_recv_times_.time_handle.newHandle("udp receive pack 4");
    this->pack5_recv_handle_ = this->pack_recv_times_.time_handle.newHandle("udp receive pack 5");
    this->pack6_recv_handle_ = this->pack_recv_times_.time_handle.newHandle("udp receive pack 6");
    this->pack7_recv_handle_ = this->pack_recv_times_.time_handle.newHandle("udp receive pack 7");
}

void CommunicationProcess::reconfigureRequest(ecu_communication::ecu_communicationConfig &config, uint32_t level) {
    if (config.params_lock) {
        ROS_WARN_STREAM("params locked, not set!");
        return;
    }
    this->params_.fromConfig(config);
    ROS_WARN_STREAM("params set!");
}

void CommunicationProcess::udpReceive() {
    while (ros::ok()) {
        this->udp_.recv();
        this->udp_recv_times_.pushTimestamp(this->udp_recv_handle_);

        //// todo this block is test code on 6t
//        {
//            if (this->udp_.get_recv_len() > 91) {
//                continue;
//                if (this->udp_.get_recv_len() > 0) {
//                    LOG_ERROR << "ecu receive length error: " << this->udp_.get_recv_len()
//                              << ". raw data as following:";
//                    this->sLog_.logUint8Array((char *) this->udp_.buffer, this->udp_.get_recv_len(),
//                                              google::ERROR);
//                } else {
//                    LOG_ERROR << "ecu receive length error: " << this->udp_.get_recv_len();
//                }
//                continue;
//            }
//            this->udp_recv_times_.pushTimestamp(this->udp_recv_correct_handle_);
//
//            static Transform6t transform6t;
//            if (!transform6t.receiveCheck((char *) this->udp_.buffer)) {
//                continue;
//                LOG_ERROR << "ecu receive ID error, receive raw data as following:";
//                this->sLog_.logUint8Array((char *) this->udp_.buffer, this->udp_.get_recv_len(),
//                                          google::ERROR);
//                continue;
//            }
//
//            union {
//                struct {
//                    uint8_t low_byte;
//                    uint8_t high_byte;
//                };
//                uint16_t result;
//            } left_rpm_6t;
//
//            union {
//                struct {
//                    uint8_t low_byte;
//                    uint8_t high_byte;
//                };
//                uint16_t result;
//            } right_rpm_6t;
//
//            left_rpm_6t.low_byte = transform6t.receive_6t.pack[5];
//            left_rpm_6t.high_byte = transform6t.receive_6t.pack[6];
//            right_rpm_6t.low_byte = transform6t.receive_6t.pack[7];
//            right_rpm_6t.high_byte = transform6t.receive_6t.pack[8];
//            this->udp_.buffer[8] = (left_rpm_6t.result <= 12000)? 0: 1;
//            this->udp_.buffer[9] = (right_rpm_6t.result >= 12000)? 0: 1;
//
//            left_rpm_6t.result = (uint16_t)round(fabs(left_rpm_6t.result - 12000.0));
//            right_rpm_6t.result = (uint16_t)round(fabs(right_rpm_6t.result - 12000.0));
//
//            this->udp_.buffer[0] = 0x00;
//            this->udp_.buffer[1] = 0x00;
//            this->udp_.buffer[2] = 0x00;
//            this->udp_.buffer[3] = 0xF1;
//            this->udp_.buffer[4] = 0x01;
//            this->udp_.buffer[5] = 0x08;
//            this->udp_.buffer[6] = left_rpm_6t.low_byte;
//            this->udp_.buffer[7] = left_rpm_6t.high_byte;
//            this->udp_.buffer[10] = right_rpm_6t.low_byte;
//            this->udp_.buffer[11] = right_rpm_6t.high_byte;
//            this->udp_.buffer[12] = 0x00;
//            this->udp_.buffer[13] = 0x00;
//        }

        //// todo comment for test on 6t
        if (this->udp_.get_recv_len() != 14) {
            continue;
            if (this->udp_.get_recv_len() > 0) {
                LOG_ERROR << "ecu receive length error: " << this->udp_.get_recv_len() << ". raw data as following:";
                this->sLog_.logUint8Array((char *)this->udp_.buffer, this->udp_.get_recv_len(), google::ERROR);
            } else {
                LOG_ERROR << "ecu receive length error: " << this->udp_.get_recv_len();
            }
            continue;
        }
        this->udp_recv_times_.pushTimestamp(this->udp_recv_correct_handle_);
        if (!this->data_upload_.dataIDCheck((char *)this->udp_.buffer)) {
            continue;
            LOG_ERROR << "ecu receive ID error, receive raw data as following:";
            this->sLog_.logUint8Array((char *)this->udp_.buffer, this->udp_.get_recv_len(), google::ERROR);
            continue;
        }
        this->pack_recv_times_.pushTimestamp(this->data_upload_.pack_handle);
        this->data_upload_mutex_.lock();
        this->data_upload_.dataDistribution();
        this->data_upload_mutex_.unlock();
        if (this->yaml_params_.log_rawdata) {
            LOG_INFO << "UDP receive raw data log";
            this->sLog_.logUint8Array((char *)this->udp_.buffer, this->udp_.get_recv_len(), google::INFO);
        }
        if (this->yaml_params_.publish_rawdata) {
            this->data_upload_.recv_rawdata.data.clear();
            this->data_upload_.recv_rawdata.data.assign(this->udp_.buffer, this->udp_.buffer + this->udp_.get_recv_len());
            this->udp_recv_rawdata_publisher_.publish(this->data_upload_.recv_rawdata);
        }
    }
}


void CommunicationProcess::udpSend() {
    if (!(this->udp_send_switch_ || this->yaml_params_.send_default_when_no_msg || this->params_.fake_issue)) {
        return;
    }

    this->udp_pack_handle_.setID(this->udp_send_proportion_.stepping());
    if (!this->udp_send_switch_) {
        this->data_download_mutex_.lock();
        this->data_download_.init();
        this->data_download_mutex_.unlock();
    }
    if (this->yaml_params_.reconfig && this->params_.fake_issue) {
        this->data_download_mutex_.lock();
        fake_issue();
        this->data_download_mutex_.unlock();
    }
    this->data_download_mutex_.lock();
    this->data_upload_mutex_.lock();
    this->data_download_.durex(this->data_upload_.pack_seven.park_status);
    this->data_upload_mutex_.unlock();
    this->data_download_.prepareSend(this->udp_pack_handle_);
    this->data_download_mutex_.unlock();


    //// todo this block is test code on 6t
//    {
//        this->data_download_mutex_.lock();
//        this->autonomousControl_.transform6t.prepareSend(&this->data_download_);
//        this->data_download_mutex_.unlock();
//        this->udp_.sendToRemote(this->autonomousControl_.transform6t.send_6t.pack, 13);
//    }

    //// todo comment for test on 6t
    if (!this->udp_.sendToRemote(this->data_download_.data_to_send, sizeof(this->data_download_.data_to_send))) {
        LOG_ERROR << "UDP send error, send length: " << this->udp_.get_send_len() << ". raw data as following:";
        this->sLog_.logUint8Array((char *)this->data_download_.data_to_send, sizeof(this->data_download_.data_to_send), google::ERROR);
        return;
    }
    if (this->yaml_params_.log_rawdata) {
        LOG_INFO << "UDP send raw data log";
        this->sLog_.logUint8Array((char *)this->data_download_.data_to_send, sizeof(this->data_download_.data_to_send), google::INFO);
    }
    if (this->yaml_params_.publish_rawdata) {
        this->data_download_.send_rawdata.data.clear();
        this->data_download_.send_rawdata.data.assign(this->data_download_.data_to_send, this->data_download_.data_to_send + sizeof(this->data_download_.data_to_send));
        this->udp_send_rawdata_publisher_.publish(this->data_download_.send_rawdata);
    }
}

void CommunicationProcess::timeCheck() {
    static bool auto_disconnect_mark = false;
    static shawn::STime auto_disconnect_timer;
    static bool remote_disconnect_mark = false;
    static shawn::STime remote_disconnect_timer;

    if (!this->remoteControl_.time_check()) {
        this->udp_send_switch_ = false;
        this->control_mode_ = three_one_feedback::control_mode::ERROR;
        LOG_ERROR << "WORK MODE ERROR!";
        return;
    }

    bool udp_recv_check = true;
    bool msg_update_check = true;
    bool remote_update_check = true;

    if (this->yaml_params_.lower_layer_receive) {
        udp_recv_check = udpReceiveCheck();
        if (!udp_recv_check) {
            LOG_ERROR << "udp receive from car abnormal";
        }
    }

    this->control_mode_mutex_.lock();
    int tmp_control_mode = (int)this->control_mode_;
    this->control_mode_mutex_.unlock();
    switch (tmp_control_mode) {
        case (int)three_one_feedback::control_mode::autonomous: {
            if (this->yaml_params_.upper_layer_receive) {
                msg_update_check = this->autonomousControl_.rosmsgUpdateCheck();
                if (!msg_update_check) {
                    LOG_ERROR << "msg receive from ros abnormal";
                    if (!auto_disconnect_mark) {
                        auto_disconnect_timer.setSimpleStartTimePoint();
                        auto_disconnect_mark = true;
                    }
                } else {
                    if (auto_disconnect_mark) {
                        if (auto_disconnect_timer.getSimpleDurationMs() < 10000) {
                            msg_update_check = false;
                        } else {
                            auto_disconnect_mark = false;
                        }
                    }
                }
            }
            this->autonomousControl_.send_switch_ = udp_recv_check;
            this->remoteControl_.send_switch_ = udp_recv_check;
            this->udp_send_switch_ = msg_update_check;
            this->autonomousControl_.receive_switch_ = true;
            this->remoteControl_.receive_switch_ = false;
            break;
        }
        case (int)three_one_feedback::control_mode::remote: {
            if (this->yaml_params_.remote_receive) {
                remote_update_check = this->remoteControl_.time_check();
                if (!remote_update_check) {
                    LOG_ERROR << "udp receive from remote abnormal";
                    if (!remote_disconnect_mark) {
                        remote_disconnect_timer.setSimpleStartTimePoint();
                        remote_disconnect_mark = true;
                    }
                } else {
                    if (remote_disconnect_mark) {
                        if (remote_disconnect_timer.getSimpleDurationMs() < 10000) {
                            remote_update_check = false;
                        } else {
                            remote_disconnect_mark = false;
                        }
                    }
                }
            }
            this->autonomousControl_.send_switch_ = udp_recv_check;
            this->remoteControl_.send_switch_ = udp_recv_check;
            this->udp_send_switch_ = remote_update_check;
            this->autonomousControl_.receive_switch_ = false;
            this->remoteControl_.receive_switch_ = true;
            break;
        }
        default: {
            this->autonomousControl_.send_switch_ = udp_recv_check;
            this->remoteControl_.send_switch_ = udp_recv_check;
            this->udp_send_switch_ = false;
            this->autonomousControl_.receive_switch_ = false;
            this->remoteControl_.receive_switch_ = false;
            LOG_ERROR << "error mode";
            break;
        }
    }

    if (udp_recv_check && msg_update_check && remote_update_check) {
        switch (tmp_control_mode) {
            case (int)three_one_feedback::control_mode::autonomous: {
                ROS_INFO_STREAM_THROTTLE(1, "work well: autonomous");
                break;
            }
            case (int)three_one_feedback::control_mode::remote: {
                ROS_INFO_STREAM_THROTTLE(1, "work well: remote");
                break;
            }
            default: {
                LOG_ERROR << "work mode logic error";
                break;
            }
        }
    }
}

bool CommunicationProcess::udpReceiveCheck() {
    bool udp_recv_duration_check = true;
    bool udp_recv_till_now_check = true;
    bool pack_recv_duration_check = true;
    bool pack_recv_till_now_check = true;
    //// todo modify parameter of check
//    udp_recv_duration_check = this->udp_recv_times_.checkTimestampsDuration(-1, -1);
    udp_recv_till_now_check = this->udp_recv_times_.checkTimestampsTillNow(-1, 500);
//    pack_recv_duration_check = this->pack_recv_times_.checkTimestampsDuration(-1, -1);
//    pack_recv_till_now_check = this->pack_recv_times_.checkTimestampsTillNow(-1, -1);

    return (udp_recv_duration_check && udp_recv_till_now_check && pack_recv_duration_check && pack_recv_till_now_check);
}

void CommunicationProcess::fake_issue() {
    if (this->params_.fake_drive) {
        this->data_download_.pack_one.work_mode = this->params_.work_mode;
        this->data_download_.pack_one.vehicle_gear = this->params_.driving_gear;
        this->data_download_.pack_one.expect_vehicle_speed = (int)(this->params_.vehicle_speed * 10);
        this->data_download_.pack_one.vehicle_turn_to = this->params_.turn_to_left? 0: 1;
        this->data_download_.pack_one.thousand_times_curvature = (int)(this->params_.vehicle_curvature * 1000);
        this->data_download_.pack_one.left_wheel_rotate = this->params_.left_wheel_forward? 0: 1;
        this->data_download_.pack_one.right_wheel_rotate = this->params_.right_wheel_forward? 0: 1;
        this->data_download_.pack_one.expect_left_speed = (int)(this->params_.left_wheel_speed * 10);
        this->data_download_.pack_one.expect_right_speed = (int)(this->params_.right_wheel_speed * 10);
        this->data_download_.pack_two.brake = this->params_.vehicle_brake;
        this->data_download_.pack_one.parking_control = this->params_.park? 1: 0;
    }
    if (this->params_.fake_suspension) {
        this->data_download_.pack_two.cylinder_select = this->params_.cylinder_select;
        this->data_download_.pack_two.suspension_select = this->params_.suspension_select;
        this->data_download_.pack_two.suspension_work_mode = this->params_.suspension_mode;
        this->data_download_.pack_two.suspension_work_mode_detail = this->params_.suspension_mode_detail;
        this->data_download_.pack_two.suspension_cylinder_select_mode = this->params_.suspension_cylinder_select;
        this->data_download_.pack_two.suspension_cylinder_motor_control = this->params_.suspension_motor? 1: 0;
        this->data_download_.pack_two.vertical_wall_mode = this->params_.vertical_wall_mode;
        this->data_download_.pack_two.fix_two_chamber_valve = this->params_.fix_two_chamber? 1: 0;
    }
    if (this->params_.fake_functions) {
        this->data_download_.pack_one.ring_control = this->params_.ring? 1: 0;
        this->data_download_.pack_one.forward_big_light = this->params_.forward_light;
        this->data_download_.pack_one.wide_taillight = this->params_.wide_taillight? 1: 0;
        this->data_download_.pack_one.turn_light = this->params_.turn_light;
        this->data_download_.pack_two.tailgate_control = this->params_.tailgate;
    }
}

void CommunicationProcess::fuck_send() {
    static std::string tmp1 = "$GEAR,";
    static std::string tmp2 = ",";
    static std::string tmp3 = ",END";
    static std::string qianjing = "0";
    static std::string houtui = "1";
    std::string data_send = tmp1;
    static int left_gear;
    static int right_gear;

    this->data_upload_mutex_.lock();
    left_gear = this->data_upload_.pack_two.left_motor_gear;
    right_gear = this->data_upload_.pack_two.right_motor_gear;
    this->data_upload_mutex_.unlock();

    data_send += (left_gear == 0? qianjing : houtui);
    data_send += tmp2;
    data_send += (right_gear == 0? qianjing : houtui);
    data_send += tmp3;

    this->serialPortCommunication.send(data_send.c_str(), 13);
}

}