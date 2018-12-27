#include "CommunicationProcess.hpp"

#define LOG_INFO LOG(INFO)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_WARN LOG(WARNING)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_ERROR LOG(ERROR)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_FATAL LOG(FATAL)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define FIXED std::setiosflags(std::ios::fixed)<<

namespace ecu_communication {
//// todo some callback apply to current framework
CommunicationProcess::CommunicationProcess(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : reconfigSrv_{private_node_handle}, params_{private_node_handle} {

    this->nh_ = node_handle;
    this->private_nh_ = private_node_handle;

    paramsInit();
    glogInit();
    setTimeCheckHandle();

    if (this->yaml_params_.reconfig) {
        this->reconfigSrv_.setCallback(boost::bind(&CommunicationProcess::reconfigureRequest, this, _1, _2));
    }
    if (this->yaml_params_.publish_rawdata) {
        this->udp_recv_rawdata_publisher_ = this->nh_.advertise<three_one_msgs::recv_rawdata>("/udp_recv_rawdata", 1);
        this->udp_send_rawdata_publisher_ = this->nh_.advertise<three_one_msgs::send_rawdata>("/udp_send_rawdata", 1);
    }
    if (this->yaml_params_.lower_layer_receive) {
        this->udp_receive_thread = std::thread(&CommunicationProcess::udpReceive, this);
        this->udp_receive_thread.detach();
    }
    if (this->yaml_params_.lower_layer_send) {
        this->udp_send_thread = std::thread(&CommunicationProcess::udpSend, this);
        this->udp_send_thread.detach();
    }
    if (this->yaml_params_.upper_layer_send) {
        this->recv_data_publisher_ = this->nh_.advertise<three_one_msgs::report>("/ecudatareport", 1);
        this->data_process_timer_ = this->nh_.createTimer(ros::Duration(this->yaml_params_.publish_period),
                                                          boost::bind(&CommunicationProcess::dataProcess, this));
    }
    if (this->yaml_params_.upper_layer_receive) {
        //// add subscriber here
    }
    ros::Duration(this->yaml_params_.check_period).sleep();
    this->time_check_timer_ = this->nh_.createTimer(ros::Duration(this->yaml_params_.check_period),
                                                    boost::bind(&CommunicationProcess::timeCheck, this));
}

void CommunicationProcess::udpReceive() {
    while (!(this->udp_server_.init(this->yaml_params_.udp_server_port))) {
        ROS_ERROR_STREAM("UDP RECEIVE INIT FAILURE, KEEP TRYING!");
    }
    while (ros::ok()) {
        this->udp_server_.process();
        this->data_upload_.recv_rawdata.recv_rawdata.clear();
        this->data_upload_.recv_rawdata.recv_rawdata.assign(this->udp_server_.buffer, this->udp_server_.buffer + this->udp_server_.get_recv_len());
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
                this->sLog_.logUint8Vector(this->data_upload_.recv_rawdata.recv_rawdata, google::ERROR);
            }
        } else {
            LOG_ERROR << "UDP receive length error, length is: " << this->udp_server_.get_recv_len() << ". raw data as following:";
            this->sLog_.logUint8Vector(this->data_upload_.recv_rawdata.recv_rawdata, google::ERROR);
        }
        if (this->yaml_params_.log_rawdata) {
            LOG_INFO << "UDP receive raw data log";
            this->sLog_.logUint8Vector(this->data_upload_.recv_rawdata.recv_rawdata, google::INFO);
        }
        if (this->yaml_params_.publish_rawdata) {
            this->udp_recv_rawdata_publisher_.publish(this->data_upload_.recv_rawdata);
        }
    }
}

void CommunicationProcess::udpSend() {
    while (!(this->udp_client_.init(this->yaml_params_.ecu_ip.data(), this->yaml_params_.ecu_port))) {
        ROS_ERROR_STREAM("UDP SEND INIT FAILURE, KEEP TRYING!");
    }
    shawn::SProportion send_proportion;
    send_proportion.inputProportion(1, 1);
    shawn::handle pack_handle;
    ros::Rate loop_rate(this->yaml_params_.udp_send_rate);
    while (ros::ok()) {
        if (!(this->udp_send_switch_ || this->yaml_params_.send_default_when_no_msg || this->params_.fake_issue)) {
            continue;
        }
        pack_handle.setID(send_proportion.stepping());
        if (!this->udp_send_switch_) {
            this->data_download_.init();
        }
        if (this->params_.fake_issue) {
            fake_issue();
        }
        this->data_download_mutex_.lock();
        this->data_download_.prepareSend(pack_handle);
        this->data_download_mutex_.unlock();
        if (!this->udp_client_.process(this->data_download_.data_to_send, sizeof(this->data_download_.data_to_send))) {
            LOG_ERROR << "UDP send error, send length: " << this->udp_client_.get_send_len() << ". raw data as following:";
            this->sLog_.logUint8Array((char *)this->data_download_.data_to_send, sizeof(this->data_download_.data_to_send), google::ERROR);
        }
        this->data_download_.send_rawdata.send_rawdata.clear();
        this->data_download_.send_rawdata.send_rawdata.assign(this->data_download_.data_to_send, this->data_download_.data_to_send + this->udp_client_.get_send_len());
        if (this->yaml_params_.log_rawdata) {
            LOG_INFO << "UDP send raw data log";
            this->sLog_.logUint8Vector(this->data_download_.send_rawdata.send_rawdata, google::INFO);
        }
        if (this->yaml_params_.publish_rawdata) {
            this->udp_send_rawdata_publisher_.publish(this->data_download_.send_rawdata);
        }
        loop_rate.sleep();
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
    udp_recv_duration_check = this->udp_recv_times_.checkTimestampsDuration(-1, -1);
    udp_recv_till_now_check = this->udp_recv_times_.checkTimestampsTillNow(-1, -1);
    pack_recv_duration_check = this->pack_recv_times_.checkTimestampsDuration(-1, -1);
    pack_recv_till_now_check = this->pack_recv_times_.checkTimestampsTillNow(-1, -1);
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
    while (!(this->private_nh_.getParam("upper_layer_send", this->yaml_params_.upper_layer_send))) {
        ROS_ERROR_STREAM("param not retrieved");
    }
    while (!(this->private_nh_.getParam("upper_layer_receive", this->yaml_params_.upper_layer_receive))) {
        ROS_ERROR_STREAM("param not retrieved");
    }
    while (!(this->private_nh_.getParam("lower_layer_send", this->yaml_params_.lower_layer_send))) {
        ROS_ERROR_STREAM("param not retrieved");
    }
    while (!(this->private_nh_.getParam("lower_layer_receive", this->yaml_params_.lower_layer_receive))) {
        ROS_ERROR_STREAM("param not retrieved");
    }

    while (!(this->private_nh_.getParam("ecu_ip", this->yaml_params_.ecu_ip))) {
        ROS_ERROR_STREAM("param not retrieved");
    }
    while (!(this->private_nh_.getParam("ecu_port", this->yaml_params_.ecu_port))) {
        ROS_ERROR_STREAM("param not retrieved");
    }
    while (!(this->private_nh_.getParam("udp_server_port", this->yaml_params_.udp_server_port))) {
        ROS_ERROR_STREAM("param not retrieved");
    }

    while (!(this->private_nh_.getParam("publish_period", this->yaml_params_.publish_period))) {
        ROS_ERROR_STREAM("param not retrieved");
    }
    while (!(this->private_nh_.getParam("check_period", this->yaml_params_.check_period))) {
        ROS_ERROR_STREAM("param not retrieved");
    }
    while (!(this->private_nh_.getParam("udp_send_rate", this->yaml_params_.udp_send_rate))) {
        ROS_ERROR_STREAM("param not retrieved");
    }

    while (!(this->private_nh_.getParam("reconfig", this->yaml_params_.reconfig))) {
        ROS_ERROR_STREAM("param not retrieved");
    }
    while (!(this->private_nh_.getParam("send_default_when_no_msg", this->yaml_params_.send_default_when_no_msg))) {
        ROS_ERROR_STREAM("param not retrieved");
    }
    while (!(this->private_nh_.getParam("log_rawdata", this->yaml_params_.log_rawdata))) {
        ROS_ERROR_STREAM("param not retrieved");
    }
    while (!(this->private_nh_.getParam("publish_rawdata", this->yaml_params_.publish_rawdata))) {
        ROS_ERROR_STREAM("param not retrieved");
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
    this->data_download_.pack_one.work_mode = (int)three_one_control::work_mode::curvature_and_vehicle_speed;
    if (this->params_.issue_speed_fake) {
        this->data_download_.pack_one.expect_vehicle_speed = (uint8_t)this->params_.fake_issue_speed * 10;
    }
    if (this->params_.issue_steer_fake) {
        if (this->params_.fake_issue_steer > 0) {
            this->data_download_.pack_one.vehicle_turn_to = int(three_one_control::vehicle_turn_to::right);
        } else {
            this->data_download_.pack_one.vehicle_turn_to = int(three_one_control::vehicle_turn_to::left);
        }
        double_t vehicle_length = 5.0;
        this->data_download_.pack_one.thousand_times_curvature = (uint16_t)(tan(this->params_.fake_issue_steer * M_PI / 180.0) / vehicle_length * 1000);
    }
    if (this->params_.issue_gear_fake) {
        switch (this->params_.fake_issue_gear) {
            case (this->params_.fake_issue_gear_D): {
                this->data_download_.pack_one.vehicle_gear = (int)three_one_control::vehicle_gear::D;
                break;
            }
            case (this->params_.fake_issue_gear_R): {
                this->data_download_.pack_one.vehicle_gear = (int)three_one_control::vehicle_gear::R;
                break;
            }
            default: {
                this->data_download_.pack_one.vehicle_gear = (int)three_one_control::vehicle_gear::N;
                break;
            }
        }
    }
}

}