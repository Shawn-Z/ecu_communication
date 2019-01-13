#include "CommunicationProcess.hpp"


namespace ecu_communication {
//// todo some callback apply to current framework
CommunicationProcess::CommunicationProcess(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : reconfigSrv_{private_node_handle}, params_{private_node_handle} {

    this->nh_ = node_handle;
    this->private_nh_ = private_node_handle;

    this->glogInit();
    this->paramsInit();
    this->setTimeCheckHandle();

    if (this->yaml_params_.reconfig) {
        this->reconfigSrv_.setCallback(boost::bind(&CommunicationProcess::reconfigureRequest, this, _1, _2));
    }
    if (this->yaml_params_.publish_rawdata) {
        this->udp_recv_rawdata_publisher_ = this->nh_.advertise<three_one_msgs::rawdata_recv>("/udp_recv_rawdata", 1);
        this->udp_send_rawdata_publisher_ = this->nh_.advertise<three_one_msgs::rawdata_send>("/udp_send_rawdata", 1);
    }
    if (this->yaml_params_.lower_layer_receive) {
        this->udp_receive_thread = std::thread(&CommunicationProcess::udpReceive, this);
        this->udp_receive_thread.detach();
    }
    if (this->yaml_params_.lower_layer_send) {
        this->udpSendInit();
        this->udp_send_timer_ = this->nh_.createTimer(ros::Duration(UDP_SEND_PERIOD),
                                                      boost::bind(&CommunicationProcess::udpSend, this));
    }
    if (this->yaml_params_.upper_layer_send) {
        this->recv_data_publisher_ = this->nh_.advertise<three_one_msgs::report>("/ecudatareport", 1);
        this->data_process_timer_ = this->nh_.createTimer(ros::Duration(PUBLISH_PERIOD),
                                                          boost::bind(&CommunicationProcess::dataProcess, this));
    }
    if (this->yaml_params_.upper_layer_receive) {
        //// add subscriber here
    }
    this->time_check_timer_ = this->nh_.createTimer(ros::Duration(CHECK_PERIOD),
                                                    boost::bind(&CommunicationProcess::timeCheck, this));
}

void CommunicationProcess::udpReceive() {
    while (!(this->udp_server_.init(this->yaml_params_.udp_server_port))) {
        ROS_ERROR_STREAM("UDP RECEIVE INIT FAILURE, KEEP TRYING!");
    }
    while (ros::ok()) {
        this->udp_server_.process();
        this->data_upload_.recv_rawdata.data.clear();
        this->data_upload_.recv_rawdata.data.assign(this->udp_server_.buffer, this->udp_server_.buffer + this->udp_server_.get_recv_len());
        this->udp_recv_times_.pushTimestamp(this->udp_recv_handle_);
        if (this->udp_server_.get_recv_len() == 14) {
            this->udp_recv_times_.pushTimestamp(this->udp_recv_correct_handle_);
            if (this->data_upload_.dataIDCheck((char *) this->udp_server_.buffer)) {
                this->pack_recv_times_.pushTimestamp(this->data_upload_.pack_handle);
                this->data_upload_mutex_.lock();
                this->data_upload_.dataDistribution();
                this->data_upload_mutex_.unlock();
            } else {
                LOG_ERROR << "UDP receive ID error, receive raw data as following:";
                this->sLog_.logUint8Vector(this->data_upload_.recv_rawdata.data, google::ERROR);
            }
        } else {
            LOG_ERROR << "UDP receive length error, length is: " << this->udp_server_.get_recv_len() << ". raw data as following:";
            this->sLog_.logUint8Vector(this->data_upload_.recv_rawdata.data, google::ERROR);
        }
        if (this->yaml_params_.log_rawdata) {
            LOG_INFO << "UDP receive raw data log";
            this->sLog_.logUint8Vector(this->data_upload_.recv_rawdata.data, google::INFO);
        }
        if (this->yaml_params_.publish_rawdata) {
            this->udp_recv_rawdata_publisher_.publish(this->data_upload_.recv_rawdata);
        }
    }
}

void CommunicationProcess::udpSendInit() {
    while (!(this->udp_client_.init(this->yaml_params_.ecu_ip.data(), this->yaml_params_.ecu_port))) {
        ROS_ERROR_STREAM("UDP SEND INIT FAILURE, KEEP TRYING!");
    }
    this->udp_send_proportion_.inputProportion(1, 1);
}

void CommunicationProcess::udpSend() {
    if (!(this->udp_send_switch_ || this->yaml_params_.send_default_when_no_msg || this->params_.fake_issue)) {
        return;
    }
    this->udp_pack_handle_.setID(this->udp_send_proportion_.stepping());
    if (!this->udp_send_switch_) {
        this->data_download_.init();
    }
    if (this->params_.fake_issue) {
        fake_issue();
    }
    this->data_download_.prepareSend(this->udp_pack_handle_);
    if (!this->udp_client_.process(this->data_download_.data_to_send, sizeof(this->data_download_.data_to_send))) {
        LOG_ERROR << "UDP send error, send length: " << this->udp_client_.get_send_len() << ". raw data as following:";
        this->sLog_.logUint8Array((char *)this->data_download_.data_to_send, sizeof(this->data_download_.data_to_send), google::ERROR);
        return;
    }
    this->data_download_.send_rawdata.data.clear();
    this->data_download_.send_rawdata.data.assign(this->data_download_.data_to_send, this->data_download_.data_to_send + this->udp_client_.get_send_len());
    if (this->yaml_params_.log_rawdata) {
        LOG_INFO << "UDP send raw data log";
        this->sLog_.logUint8Vector(this->data_download_.send_rawdata.data, google::INFO);
    }
    if (this->yaml_params_.publish_rawdata) {
        this->udp_send_rawdata_publisher_.publish(this->data_download_.send_rawdata);
    }
}

void CommunicationProcess::dataProcess() {
    if (!this->ros_publish_switch_) {
        return;
    }
    this->data_upload_mutex_.lock();
    if (!this->data_upload_.dataCheck()) {
        LOG_ERROR << "upload data check error, raw data as following:";
        this->sLog_.logUint8Array((char *)this->data_upload_.pack_one.pack, sizeof(this->data_upload_.pack_one.pack), google::ERROR);
        this->sLog_.logUint8Array((char *)this->data_upload_.pack_two.pack, sizeof(this->data_upload_.pack_two.pack), google::ERROR);
        this->sLog_.logUint8Array((char *)this->data_upload_.pack_three.pack, sizeof(this->data_upload_.pack_three.pack), google::ERROR);
        this->sLog_.logUint8Array((char *)this->data_upload_.pack_four.pack, sizeof(this->data_upload_.pack_four.pack), google::ERROR);
        this->data_upload_mutex_.unlock();
        return;
    }
    this->data_upload_.dataToMsg();
    this->data_upload_mutex_.unlock();
    this->reportControlData();
    this->recv_data_publisher_.publish(this->data_upload_.report);
}

void CommunicationProcess::reconfigureRequest(ecu_communication::ecu_communicationConfig &config, uint32_t level) {
    if (config.params_lock) {
        ROS_WARN_STREAM("params locked, not set!");
        return;
    }
    this->params_.fromConfig(config);
    ROS_WARN_STREAM("params set!");
}

void CommunicationProcess::glogInit() {
    this->sLog_.init("ThreeOne", "log_three_one", google::ERROR);
    LOG_INFO << "glog start";
    LOG_WARN << "glog start";
    LOG_ERROR << "glog start";
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

bool CommunicationProcess::udpReceiveCheck() {
    bool udp_recv_duration_check = true;
    bool udp_recv_till_now_check = true;
    bool pack_recv_duration_check = true;
    bool pack_recv_till_now_check = true;
    //// todo modify parameter of check
//    udp_recv_duration_check = this->udp_recv_times_.checkTimestampsDuration(-1, -1);
//    udp_recv_till_now_check = this->udp_recv_times_.checkTimestampsTillNow(-1, -1);
//    pack_recv_duration_check = this->pack_recv_times_.checkTimestampsDuration(-1, -1);
//    pack_recv_till_now_check = this->pack_recv_times_.checkTimestampsTillNow(-1, -1);
    if (udp_recv_duration_check && udp_recv_till_now_check && pack_recv_duration_check && pack_recv_till_now_check) {
        this->ros_publish_switch_ = true;
    } else {
        this->ros_publish_switch_ = false;
        ROS_ERROR_STREAM("ERROR in UDP Receive!");
        //// todo log details
        return false;
    }
    return true;
}

bool CommunicationProcess::rosmsgUpdateCheck() {
    //// todo modify check
    bool msg_duration_check = true;
    bool msg_till_now_check = true;
    msg_duration_check = this->msg_update_times.checkTimestampsDuration(-1, -1);
    msg_till_now_check = this->msg_update_times.checkTimestampsTillNow(-1, -1);
    if (msg_duration_check && msg_till_now_check) {
        this->udp_send_switch_ = true;
    } else {
        this->udp_send_switch_ = false;
        ROS_ERROR_STREAM("ERROR NO ROS msg Receive!");
        return false;
    }
    return true;
}

void CommunicationProcess::timeCheck() {
    bool udp_recv_check = true;
    bool msg_update_check = true;
    if (yaml_params_.lower_layer_receive) {
        udp_recv_check = udpReceiveCheck();
    }
    if (yaml_params_.upper_layer_receive) {
        msg_update_check = rosmsgUpdateCheck();
    }
    if (udp_recv_check && msg_update_check) {
        ROS_INFO_STREAM_THROTTLE(1, "work well");
    }
}

void CommunicationProcess::paramsInit() {
    bool tmp_result = true;
    tmp_result &= this->private_nh_.getParam("upper_layer_send", this->yaml_params_.upper_layer_send);
    tmp_result &= this->private_nh_.getParam("upper_layer_receive", this->yaml_params_.upper_layer_receive);
    tmp_result &= this->private_nh_.getParam("lower_layer_send", this->yaml_params_.lower_layer_send);
    tmp_result &= this->private_nh_.getParam("lower_layer_receive", this->yaml_params_.lower_layer_receive);

    tmp_result &= this->private_nh_.getParam("ecu_ip", this->yaml_params_.ecu_ip);
    tmp_result &= this->private_nh_.getParam("ecu_port", this->yaml_params_.ecu_port);
    tmp_result &= this->private_nh_.getParam("udp_server_port", this->yaml_params_.udp_server_port);

    tmp_result &= this->private_nh_.getParam("reconfig", this->yaml_params_.reconfig);
    tmp_result &= this->private_nh_.getParam("send_default_when_no_msg", this->yaml_params_.send_default_when_no_msg);
    tmp_result &= this->private_nh_.getParam("log_rawdata", this->yaml_params_.log_rawdata);
    tmp_result &= this->private_nh_.getParam("publish_rawdata", this->yaml_params_.publish_rawdata);

    if (!tmp_result) {
        ROS_ERROR_STREAM("param not retrieved");
        ros::shutdown();
    }

    this->udp_send_switch_ = false;
    this->ros_publish_switch_ = false;
    //// todo log params
}

CommunicationProcess::~CommunicationProcess() {
    this->time_check_timer_.stop();
    this->data_process_timer_.stop();

    this->udp_send_switch_ = false;
    this->ros_publish_switch_ = false;
    this->yaml_params_.reconfig = false;
    this->params_.fake_issue = false;
    this->yaml_params_.send_default_when_no_msg = false;

    //// todo shutdown publisher and subscriber
    this->recv_data_publisher_.shutdown();
    this->udp_recv_rawdata_publisher_.shutdown();
    this->udp_send_rawdata_publisher_.shutdown();

    LOG_INFO << "program end";
    LOG_WARN << "program end";
    LOG_ERROR << "program end";
    google::ShutdownGoogleLogging();
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

void CommunicationProcess::reportControlData() {
    this->data_upload_.report.control.curvature = this->data_download_.pack_one.thousand_times_curvature / 1000.0;
    this->data_upload_.report.control.speed = this->data_download_.pack_one.expect_vehicle_speed / 10.0;
    if (this->data_download_.pack_one.vehicle_gear == (int)three_one_control::vehicle_gear::R) {
        this->data_upload_.report.control.speed = -this->data_upload_.report.control.speed;
    }
    if (this->data_download_.pack_one.vehicle_gear == (int)three_one_control::vehicle_gear::N) {
        this->data_upload_.report.control.speed = 0;
    }
    this->data_upload_.report.control.rpm = (uint16_t)(fabs(this->data_upload_.report.control.speed / this->data_upload_.rpm_to_speed));
    this->data_upload_.report.control.work_mode = this->data_download_.pack_one.work_mode;
    this->data_upload_.report.control.gear = this->data_download_.pack_one.vehicle_gear;
    this->data_upload_.report.control.turn_to = this->data_download_.pack_one.vehicle_turn_to;
    this->data_upload_.report.control.brake = this->data_download_.pack_two.brake;
    this->data_upload_.report.control.park = this->data_download_.pack_one.parking_control;

    this->data_upload_.report.control.cylinder_select = this->data_download_.pack_two.cylinder_select;
    this->data_upload_.report.control.suspension_select = this->data_download_.pack_two.suspension_select;
    this->data_upload_.report.control.vertical_wall_mode = this->data_download_.pack_two.vertical_wall_mode;
    this->data_upload_.report.control.suspension_work_mode = this->data_download_.pack_two.suspension_work_mode;
    this->data_upload_.report.control.suspension_work_mode_detail = this->data_download_.pack_two.suspension_work_mode_detail;
    this->data_upload_.report.control.suspension_cylinder_select_mode = this->data_download_.pack_two.suspension_cylinder_select_mode;
    this->data_upload_.report.control.suspension_cylinder_motor_control = this->data_download_.pack_two.suspension_cylinder_motor_control;
    this->data_upload_.report.control.fix_two_chamber_valve = this->data_download_.pack_two.fix_two_chamber_valve;
}

}