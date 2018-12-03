#include "CommunicationProcess.hpp"

#define LOG_INFO LOG(INFO)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_WARN LOG(WARNING)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_ERROR LOG(ERROR)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_FATAL LOG(FATAL)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define FIXED std::setiosflags(std::ios::fixed)<<

namespace ecu_communication {
//// todo data download check
//// todo some callback apply to current framework
CommunicationProcess::CommunicationProcess(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : reconfigSrv_{private_node_handle},
      params_{private_node_handle} {

    google::InitGoogleLogging("Three_One_Communication");
    FLAGS_log_dir = getenv("HOME");
    LOG_INFO << "program start";
    LOG_WARN << "program start";
    LOG_ERROR << "program start";

    this->nh_ = node_handle;
    this->private_nh_ = private_node_handle;

    paramsInit();

    if (this->reconfig_) {
        this->reconfigSrv_.setCallback(boost::bind(&CommunicationProcess::reconfigureRequest, this, _1, _2));
    }

    if (this->lower_layer_receive_) {
        //// udp receive
        this->udp_receive_thread = std::thread(&CommunicationProcess::udpReceive, this);
        this->udp_receive_thread.detach();
    }

    if (this->lower_layer_send_) {
        //// udp send
        this->udp_send_thread = std::thread(&CommunicationProcess::udpSend, this);
        this->udp_send_thread.detach();
    }

    if (this->upper_layer_send_) {
        //// add publisher here
        this->recv_data_publisher_ = this->nh_.advertise<three_one_msgs::report>("/ecudatareport", 1);
        //// timer for process and publish
        this->data_process_timer_ = this->nh_.createTimer(ros::Duration(this->publish_period_),
                                                          boost::bind(&CommunicationProcess::dataProcess, this));
    }

    if (this->upper_layer_receive_) {
        //// add subscriber here
    }

    this->udp_receive_switch_ = true;

    //// start time check after a little while
    ros::Duration(this->check_period_).sleep();
    this->time_check_timer_ = this->nh_.createTimer(ros::Duration(this->check_period_),
                                                    boost::bind(&CommunicationProcess::timeCheck, this));

    logMarkers();
}


void CommunicationProcess::udpReceive() {
    while (!(this->udp_server_.init(this->udp_server_port_))) {
        ROS_ERROR_STREAM("UDP RECEIVE INIT FAILURE, KEEP TRYING!");
    }
    while (ros::ok()) {
        if (!this->udp_receive_switch_) {
            continue;
        }
        this->udp_server_.process();
        this->udp_recv_times_.pushTimestamp(0); //// simple udp recv time
        if (this->udp_server_.get_recv_len() == 14) {
            this->udp_recv_times_.pushTimestamp(1); //// simple udp recv correct time
            if (this->data_upload_.dataPackCheck((char *)this->udp_server_.buffer) > 0) {
                this->pack_recv_times_.pushTimestamp((size_t)this->data_upload_.data_pack_num - 1);
                this->data_upload_mutex_.lock();
                this->data_upload_.dataDistribution();
                this->data_upload_mutex_.unlock();
            } else {
                errorLog(communication_process_error_type::udp_recv_ID_error);
            }
        } else {
            errorLog(communication_process_error_type::udp_recv_len_error);
        }
    }
}

void CommunicationProcess::udpSend() {
    while (!(this->udp_client_.init(this->ecu_ip_.data(), this->ecu_port_))) {
        ROS_ERROR_STREAM("UDP SEND INIT FAILURE, KEEP TRYING!");
    }
    ros::Rate loop_rate(20);
    while (ros::ok()) {
        if (!(this->udp_send_switch_ || this->send_default_when_no_msg_ || this->params_.fake_issue)) {
            continue;
        }

        dataDownloadCopy(1);
        if (!this->udp_send_switch_) {
            this->data_download_copied_.init();
        }
        if (this->params_.fake_issue) {
            fake_issue();
        }
        if (this->udp_client_.process(this->data_download_copied_.data_download_pack_one.result_data,
                                      sizeof(this->data_download_copied_.data_download_pack_one.result_data))) {
            this->udp_send_times_.pushTimestamp(0);
            this->pack_send_times_.pushTimestamp(0);
        } else {
            errorLog(communication_process_error_type::udp_send_pack_one_len_error);
        }

        ros::Duration(0.025).sleep();

        dataDownloadCopy(2);
        if (!this->udp_send_switch_) {
            this->data_download_copied_.init();
        }
        if (this->params_.fake_issue) {
            fake_issue();
        }
        if (this->udp_client_.process(this->data_download_copied_.data_download_pack_two.result_data,
                                      sizeof(this->data_download_copied_.data_download_pack_two.result_data))) {
            this->udp_send_times_.pushTimestamp(0);
            this->pack_send_times_.pushTimestamp(1);
        } else {
            errorLog(communication_process_error_type::udp_send_pack_two_len_error);
        }

        loop_rate.sleep();
    }
}

CommunicationProcess::~CommunicationProcess() {
    this->time_check_timer_.stop();
    this->data_process_timer_.stop();

    this->udp_send_switch_ = false;
    this->udp_receive_switch_ = false;
    this->ros_publish_switch_ = false;
    this->reconfig_ = false;
    this->params_.fake_issue = false;
    ros::Duration(0.3).sleep();
    this->send_default_when_no_msg_ = false;

    //// todo shutdown publisher and subscriber
    this->recv_data_publisher_.shutdown();

    LOG_INFO << "program end";
    LOG_WARN << "program end";
    LOG_ERROR << "program end";
    google::ShutdownGoogleLogging();
}

void CommunicationProcess::dataProcess() {
    dataUploadCopy();
    if (!msgDistribution()) {
        errorLog(communication_process_error_type::udp_receive_data_illegal);
        return;
    }
    if (this->ros_publish_switch_) {
        this->recv_data_publisher_.publish(this->report_);
        this->ros_publish_times_.pushTimestamp(0);
    }
}

void CommunicationProcess::dataUploadCopy() {
    this->data_upload_mutex_.lock();
    this->data_upload_copied_.data_upload_pack_one = this->data_upload_.data_upload_pack_one;
    this->data_upload_copied_.data_upload_pack_two = this->data_upload_.data_upload_pack_two;
    this->data_upload_copied_.data_upload_pack_three = this->data_upload_.data_upload_pack_three;
    this->data_upload_copied_.data_upload_pack_four = this->data_upload_.data_upload_pack_four;
    this->data_upload_mutex_.unlock();
}

void CommunicationProcess::dataDownloadCopy(uint8_t pack_num) {
    switch (pack_num) {
        case 1: {
            this->data_download_pack_one_mutex_.lock();
            this->data_download_copied_.data_download_pack_one = this->data_download_.data_download_pack_one;
            this->data_download_pack_one_mutex_.unlock();
            break;
        }
        case 2: {
            this->data_download_pack_two_mutex_.lock();
            this->data_download_copied_.data_download_pack_two = this->data_download_.data_download_pack_two;
            this->data_download_pack_two_mutex_.unlock();
            break;
        }
        default: {
            return;
        }
    }
}

bool CommunicationProcess::msgDistribution() {
    if (this->data_upload_copied_.data_upload_pack_one.left_wheel_expect_speed > 25000) {
        return false;
    }
    this->report_.give_back.left_wheel_expect_speed = this->data_upload_copied_.data_upload_pack_one.left_wheel_expect_speed * 0.001;
    if (this->data_upload_copied_.data_upload_pack_one.right_wheel_expect_speed > 25000) {
        return false;
    }
    this->report_.give_back.right_wheel_expect_speed = this->data_upload_copied_.data_upload_pack_one.right_wheel_expect_speed * 0.001;
    if (this->data_upload_copied_.data_upload_pack_one.mechanical_brake > 1000) {
        return false;
    }
    this->report_.vehicle_state.mechanical_brake = this->data_upload_copied_.data_upload_pack_one.mechanical_brake * 0.1;
    if (this->data_upload_copied_.data_upload_pack_one.vehicle_speed > 250) {
        return false;
    }
    this->report_.vehicle_state.vehicle_speed = this->data_upload_copied_.data_upload_pack_one.vehicle_speed * 0.1;
    if ((this->data_upload_copied_.data_upload_pack_one.gear < 1) || (this->data_upload_copied_.data_upload_pack_one.gear > 3)) {
        return false;
    }
    this->report_.vehicle_state.current_gear = this->data_upload_copied_.data_upload_pack_one.gear;

    if (this->data_upload_copied_.data_upload_pack_two.left_motor_actual_speed > 10000) {
        return false;
    }
    this->report_.vehicle_state.left_motor_rpm = this->data_upload_copied_.data_upload_pack_two.left_motor_actual_speed;
    if (this->data_upload_copied_.data_upload_pack_two.right_motor_actual_speed > 10000) {
        return false;
    }
    this->report_.vehicle_state.right_motor_rpm = this->data_upload_copied_.data_upload_pack_two.right_motor_actual_speed;
    if (this->data_upload_copied_.data_upload_pack_two.left_motor_gear > 1) {
        return false;
    }
    this->report_.vehicle_state.left_wheel_rotate = this->data_upload_copied_.data_upload_pack_two.left_motor_gear;
    if (this->data_upload_copied_.data_upload_pack_two.right_motor_gear > 1) {
        return false;
    }
    this->report_.vehicle_state.right_wheel_rotate = this->data_upload_copied_.data_upload_pack_two.right_motor_gear;
    if (this->data_upload_copied_.data_upload_pack_two.SOC > 100) {
        return false;
    }
    this->report_.vehicle_state.SOC = this->data_upload_copied_.data_upload_pack_two.SOC;
    if (this->data_upload_copied_.data_upload_pack_two.tailgate_state > 2) {
        return false;
    }
    this->report_.vehicle_state.tailgate_state = this->data_upload_copied_.data_upload_pack_two.tailgate_state;

    if (this->data_upload_copied_.data_upload_pack_three.left_one_cylinder_position > 250) {
        return false;
    }
    this->report_.cylinder_position.left_one_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.left_one_cylinder_position * 2;
    if (this->data_upload_copied_.data_upload_pack_three.left_two_cylinder_position > 250) {
        return false;
    }
    this->report_.cylinder_position.left_two_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.left_two_cylinder_position * 2;
    if (this->data_upload_copied_.data_upload_pack_three.left_three_cylinder_position > 250) {
        return false;
    }
    this->report_.cylinder_position.left_three_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.left_three_cylinder_position * 2;
    if (this->data_upload_copied_.data_upload_pack_three.left_four_cylinder_position > 250) {
        return false;
    }
    this->report_.cylinder_position.left_four_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.left_four_cylinder_position * 2;
    if (this->data_upload_copied_.data_upload_pack_three.right_one_cylinder_position > 250) {
        return false;
    }
    this->report_.cylinder_position.right_one_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.right_one_cylinder_position * 2;
    if (this->data_upload_copied_.data_upload_pack_three.right_two_cylinder_position > 250) {
        return false;
    }
    this->report_.cylinder_position.right_two_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.right_two_cylinder_position * 2;
    if (this->data_upload_copied_.data_upload_pack_three.right_three_cylinder_position > 250) {
        return false;
    }
    this->report_.cylinder_position.right_three_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.right_three_cylinder_position * 2;
    if (this->data_upload_copied_.data_upload_pack_three.right_four_cylinder_position > 250) {
        return false;
    }
    this->report_.cylinder_position.right_four_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.right_four_cylinder_position * 2;

    if (this->data_upload_copied_.data_upload_pack_four.left_one_cylinder_pressure > 250) {
        return false;
    }
    this->report_.cylinder_pressure.left_one_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.left_one_cylinder_pressure;
    if (this->data_upload_copied_.data_upload_pack_four.left_two_cylinder_pressure > 250) {
        return false;
    }
    this->report_.cylinder_pressure.left_two_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.left_two_cylinder_pressure;
    if (this->data_upload_copied_.data_upload_pack_four.left_three_cylinder_pressure > 250) {
        return false;
    }
    this->report_.cylinder_pressure.left_three_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.left_three_cylinder_pressure;
    if (this->data_upload_copied_.data_upload_pack_four.left_four_cylinder_pressure > 250) {
        return false;
    }
    this->report_.cylinder_pressure.left_four_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.left_four_cylinder_pressure;
    if (this->data_upload_copied_.data_upload_pack_four.right_one_cylinder_pressure > 250) {
        return false;
    }
    this->report_.cylinder_pressure.right_one_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.right_one_cylinder_pressure;
    if (this->data_upload_copied_.data_upload_pack_four.right_two_cylinder_pressure > 250) {
        return false;
    }
    this->report_.cylinder_pressure.right_two_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.right_two_cylinder_pressure;
    if (this->data_upload_copied_.data_upload_pack_four.right_three_cylinder_pressure > 250) {
        return false;
    }
    this->report_.cylinder_pressure.right_three_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.right_three_cylinder_pressure;
    if (this->data_upload_copied_.data_upload_pack_four.right_four_cylinder_pressure > 250) {
        return false;
    }
    this->report_.cylinder_pressure.right_four_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.right_four_cylinder_pressure;
}

void CommunicationProcess::errorLog(communication_process_error_type p_error) {
    logMarkers();

    switch ((int)p_error) {
        case 1: {
            LOG_ERROR << "UDP received length error: " << this->udp_server_.get_recv_len();
            break;
        }
        case 2: {
            LOG_ERROR << "UDP received ID error: 0x" << std::hex << this->data_upload_.ID_calculate.result;
            break;
        }
        case 3: {
            LOG_ERROR << "UDP send pack one length error: " << this->udp_client_.get_send_len();
            break;
        }
        case 4: {
            LOG_ERROR << "UDP send pack two length error: " << this->udp_client_.get_send_len();
            break;
        }
        case 5: {
            LOG_ERROR << "last_udp_receive_interval_: " << FIXED this->udp_recv_times_.outputTimestampsDurationMs(0);
            LOG_ERROR << "last_udp_receive_till_now_: " << FIXED this->udp_recv_times_.outputTimestampsTillNowMs(0);
            LOG_ERROR << "last_udp_receive_correct_interval_: " << FIXED this->udp_recv_times_.outputTimestampsDurationMs(1);
            LOG_ERROR << "last_udp_receive_correct_till_now_: " << FIXED this->udp_recv_times_.outputTimestampsTillNowMs(1);
            LOG_ERROR << "last_udp_recv_1_correct_interval_: " << FIXED this->pack_recv_times_.outputTimestampsDurationMs(0);
            LOG_ERROR << "last_udp_recv_1_correct_till_now_: " << FIXED this->pack_recv_times_.outputTimestampsTillNowMs(0);
            LOG_ERROR << "last_udp_recv_2_correct_interval_: " << FIXED this->pack_recv_times_.outputTimestampsDurationMs(1);
            LOG_ERROR << "last_udp_recv_2_correct_till_now_: " << FIXED this->pack_recv_times_.outputTimestampsTillNowMs(1);
            LOG_ERROR << "last_udp_recv_3_correct_interval_: " << FIXED this->pack_recv_times_.outputTimestampsDurationMs(2);
            LOG_ERROR << "last_udp_recv_3_correct_till_now_: " << FIXED this->pack_recv_times_.outputTimestampsTillNowMs(2);
            LOG_ERROR << "last_udp_recv_4_correct_interval_: " << FIXED this->pack_recv_times_.outputTimestampsDurationMs(3);
            LOG_ERROR << "last_udp_recv_4_correct_till_now_: " << FIXED this->pack_recv_times_.outputTimestampsTillNowMs(3);
            break;
        }
        case 6: {
            LOG_ERROR << "last_udp_send_interval_: " << FIXED this->udp_send_times_.outputTimestampsDurationMs(0);
            LOG_ERROR << "last_udp_send_till_now_: " << FIXED this->udp_send_times_.outputTimestampsTillNowMs(0);
            LOG_ERROR << "last_udp_send_one_interval_: " << FIXED this->pack_recv_times_.outputTimestampsDurationMs(0);
            LOG_ERROR << "last_udp_send_one_till_now_: " << FIXED this->pack_recv_times_.outputTimestampsTillNowMs(0);
            LOG_ERROR << "last_udp_send_two_interval_: " << FIXED this->pack_recv_times_.outputTimestampsDurationMs(1);
            LOG_ERROR << "last_udp_send_two_till_now_: " << FIXED this->pack_recv_times_.outputTimestampsTillNowMs(1);
            break;
        }
        case 7: {
            //// todo may log details
            LOG_ERROR << "udp_receive_data_illegal!";
            break;
        }
        case 8: {
            LOG_ERROR << "last_publish_interval_" << FIXED this->ros_publish_times_.outputTimestampsDurationMs(0);
            LOG_ERROR << "last_publish_till_now_" << FIXED this->ros_publish_times_.outputTimestampsTillNowMs(0);
            break;
        }
        case 9: {
            LOG_ERROR << "ros msg receive time over!";
            break;
        }
        default: {
            LOG_FATAL << "INVALID ERROR CODE!";
            break;
        }
    }
}

void CommunicationProcess::timeCheck() {
    this->time_check_no_error_ = true;

    if (this->lower_layer_send_) {
        udpSendCheck();
    }
    if (this->upper_layer_send_) {
        rosPublishCheck();
    }
    if (lower_layer_receive_) {
        udpReceiveCheck();
    }
    if (upper_layer_receive_) {
        rosmsgUpdateCheck();
    }

    if (this->time_check_no_error_) {
        ROS_INFO_STREAM_THROTTLE(std::max((int)(this->well_work_display_period_ / this->check_period_), 1), "work well");
    }

    if (this->verbose_log_) {
        logVerboseInfo();
    }
}

void CommunicationProcess::logVerboseInfo() {
    logMarkers();
    LOG_INFO << "last_udp_receive_interval_: " << FIXED this->udp_recv_times_.outputTimestampsDurationMs(0);
    LOG_INFO << "last_udp_receive_till_now_: " << FIXED this->udp_recv_times_.outputTimestampsTillNowMs(0);
    LOG_INFO << "last_udp_receive_correct_interval_: " << FIXED this->udp_recv_times_.outputTimestampsDurationMs(1);
    LOG_INFO << "last_udp_receive_correct_till_now_: " << FIXED this->udp_recv_times_.outputTimestampsTillNowMs(1);
    LOG_INFO << "last_udp_recv_1_correct_interval_: " << FIXED this->pack_recv_times_.outputTimestampsDurationMs(0);
    LOG_INFO << "last_udp_recv_1_correct_till_now_: " << FIXED this->pack_recv_times_.outputTimestampsTillNowMs(0);
    LOG_INFO << "last_udp_recv_2_correct_interval_: " << FIXED this->pack_recv_times_.outputTimestampsDurationMs(1);
    LOG_INFO << "last_udp_recv_2_correct_till_now_: " << FIXED this->pack_recv_times_.outputTimestampsTillNowMs(1);
    LOG_INFO << "last_udp_recv_3_correct_interval_: " << FIXED this->pack_recv_times_.outputTimestampsDurationMs(2);
    LOG_INFO << "last_udp_recv_3_correct_till_now_: " << FIXED this->pack_recv_times_.outputTimestampsTillNowMs(2);
    LOG_INFO << "last_udp_recv_4_correct_interval_: " << FIXED this->pack_recv_times_.outputTimestampsDurationMs(3);
    LOG_INFO << "last_udp_recv_4_correct_till_now_: " << FIXED this->pack_recv_times_.outputTimestampsTillNowMs(3);
    LOG_INFO << "last_udp_send_interval_: " << FIXED this->udp_send_times_.outputTimestampsDurationMs(0);
    LOG_INFO << "last_udp_send_till_now_: " << FIXED this->udp_send_times_.outputTimestampsTillNowMs(0);
    LOG_INFO << "last_udp_send_one_interval_: " << FIXED this->pack_recv_times_.outputTimestampsDurationMs(0);
    LOG_INFO << "last_udp_send_one_till_now_: " << FIXED this->pack_recv_times_.outputTimestampsTillNowMs(0);
    LOG_INFO << "last_udp_send_two_interval_: " << FIXED this->pack_recv_times_.outputTimestampsDurationMs(1);
    LOG_INFO << "last_udp_send_two_till_now_: " << FIXED this->pack_recv_times_.outputTimestampsTillNowMs(1);
    LOG_INFO << "last_publish_interval_" << FIXED this->ros_publish_times_.outputTimestampsDurationMs(0);
    LOG_INFO << "last_publish_till_now_" << FIXED this->ros_publish_times_.outputTimestampsTillNowMs(0);
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
    this->yaml_params_.verbose_log = this->private_nh_.param("verbose_log", false);
    this->yaml_params_.reconfig = this->private_nh_.param("reconfig", false);
    this->yaml_params_.send_default_when_no_msg = this->private_nh_.param("send_default_when_no_msg", false);
    this->yaml_params_.publish_period = this->private_nh_.param("publish_period", 20) * 0.001;
    this->yaml_params_.check_period = this->private_nh_.param("check_period", 200) * 0.001;
    this->yaml_params_.essential_msg_max_period = this->private_nh_.param("essential_msg_max_period", 100) * 0.001;
    this->yaml_params_.well_work_display_period = this->private_nh_.param("well_work_display_period", 1000) * 0.001;

    //// todo params check
    //// todo log params

    this->upper_layer_send_ = this->yaml_params_.upper_layer_send;
    this->upper_layer_receive_ = this->yaml_params_.upper_layer_receive;
    this->lower_layer_send_ = this->yaml_params_.lower_layer_send;
    this->lower_layer_receive_ = this->yaml_params_.lower_layer_receive;
    this->verbose_log_ = this->yaml_params_.verbose_log;
    this->reconfig_ = this->yaml_params_.reconfig;
    this->send_default_when_no_msg_ = this->yaml_params_.send_default_when_no_msg;
    this->ecu_ip_ = this->yaml_params_.ecu_ip;
    this->ecu_port_ = (uint16_t)this->yaml_params_.ecu_port;
    this->udp_server_port_ = (uint16_t)this->yaml_params_.udp_server_port;
    this->publish_period_ = this->yaml_params_.publish_period;
    this->check_period_ = this->yaml_params_.check_period;
    this->essential_msg_max_period_ = this->yaml_params_.essential_msg_max_period;
    this->well_work_display_period_ = this->yaml_params_.well_work_display_period;

    //// check_period > essential_msg_max_period
    if (this->check_period_ < (2.0 * this->essential_msg_max_period_ + 0.01)) {
        this->check_period_ = (2.0 * this->essential_msg_max_period_ + 0.01);
    }
    if (this->check_period_ < (2.0 * this->publish_period_ + 0.01)) {
        this->check_period_ = (2.0 * this->publish_period_ + 0.01);
    }
    if (this->check_period_ < 0.11) {
        this->check_period_ = 0.11;
    }

    this->udp_send_switch_ = false;
    this->udp_receive_switch_ = false;
    this->ros_publish_switch_ = false;

    this->ros_msg_update_.essential.result = 255;
    this->ros_msg_update_.essential_yes = 255;
    setROSmsgUpdateFalse();
}

void CommunicationProcess::setROSmsgUpdateFalse() {
    //// todo add more msg false
    this->ros_msg_update_.essential.path_plan = 0;
}

void CommunicationProcess::reconfigureRequest(ecu_communication::ecu_communicationConfig &config, uint32_t level) {
    if (config.params_lock) {
        ROS_WARN_STREAM("params locked, not set!");
        return;
    }
    this->params_.fromConfig(config);
    ROS_WARN_STREAM("params set!");
}

void CommunicationProcess::fake_issue() {
    //// todo cancel halt
    this->data_download_copied_.data_download_pack_one.work_mode = (int)three_one_control::work_mode::curvature_and_vehicle_speed;
    if (this->params_.issue_speed_fake) {
        this->data_download_copied_.data_download_pack_one.expect_vehicle_speed = (uint8_t)this->params_.fake_issue_speed * 10;
    }
    if (this->params_.issue_steer_fake) {
        if (this->params_.fake_issue_steer > 0) {
            this->data_download_copied_.data_download_pack_one.vehicle_turn_to = int(three_one_control::vehicle_turn_to::right);
        } else {
            this->data_download_copied_.data_download_pack_one.vehicle_turn_to = int(three_one_control::vehicle_turn_to::left);
        }
        double_t vehicle_length = 5.0;
        this->data_download_copied_.data_download_pack_one.thousand_times_curvature = (uint16_t)(tan(this->params_.fake_issue_steer * M_PI / 180.0) / vehicle_length * 1000);
    }
    if (this->params_.issue_gear_fake) {
        switch (this->params_.fake_issue_gear) {
            case (this->params_.fake_issue_gear_D): {
                this->data_download_copied_.data_download_pack_one.vehicle_gear = (int)three_one_control::vehicle_gear::D;
                break;
            }
            case (this->params_.fake_issue_gear_R): {
                this->data_download_copied_.data_download_pack_one.vehicle_gear = (int)three_one_control::vehicle_gear::R;
                break;
            }
            default: {
                this->data_download_copied_.data_download_pack_one.vehicle_gear = (int)three_one_control::vehicle_gear::N;
                break;
            }
        }
    }
}

void CommunicationProcess::logMarkers() {
    LOG_INFO << "ros msg update result: " << (int)this->ros_msg_update_.essential.result;
    LOG_INFO << "udp_receive_switch_: " << this->udp_receive_switch_;
    LOG_INFO << "udp_send_switch_: " << this->udp_send_switch_;
    LOG_INFO << "ros_publish_switch_: " << this->ros_publish_switch_;

    LOG_INFO << "params_lock: " << this->params_.params_lock;
    LOG_INFO << "fake_issue: " << this->params_.fake_issue;
}

void CommunicationProcess::udpSendCheck() {
    if ((this->udp_send_switch_) &&
        ((!this->udp_send_times_.checkTimestampsDuration(20, 30)) ||
         (!this->udp_send_times_.checkTimestampsTillNow(-1, 30)) ||
         (!this->pack_send_times_.checkTimestampsDuration(45, 55)) ||
         (!this->pack_send_times_.checkTimestampsTillNow(-1, 55)))) {
        this->time_check_no_error_ = false;
        ROS_ERROR_STREAM("ERROR in UDP Send!");
        errorLog(communication_process_error_type::udp_send_time_error);
    }
}

void CommunicationProcess::rosPublishCheck() {
    if ((this->ros_publish_switch_) &&
        ((!this->ros_publish_times_.checkTimestampsDuration(45, 55)) ||
         (!this->ros_publish_times_.checkTimestampsTillNow(-1, 55)))) {
        this->time_check_no_error_ = false;
        ROS_ERROR_STREAM("ERROR in ROS Publish!");
        errorLog(communication_process_error_type::ros_publish_time_error);
    }
}

void CommunicationProcess::udpReceiveCheck() {
    if (//(this->udp_receive_switch_) &&
        ((!this->udp_recv_times_.checkTimestampsDuration(5, 25)) ||
         (!this->udp_recv_times_.checkTimestampsTillNow(-1, 25)) ||
         (!this->pack_recv_times_.checkTimestampsDuration(45, 55)) ||
         (!this->pack_recv_times_.checkTimestampsTillNow(-1, 55)))) {
        this->time_check_no_error_ = false;
        this->ros_publish_switch_ = false;
        ROS_ERROR_STREAM("ERROR in UDP Receive!");
        errorLog(communication_process_error_type::udp_receive_time_error);
    } else {
        this->ros_publish_switch_ = this->udp_receive_switch_;
    }
}

void CommunicationProcess::rosmsgUpdateCheck() {
    if (this->ros_msg_update_.essential.result != this->ros_msg_update_.essential_yes) {
        this->time_check_no_error_ = false;
        this->udp_send_switch_ = false;
        ROS_ERROR_STREAM("ERROR NO ROS msg Receive!");
        errorLog(communication_process_error_type::ros_msg_receive_time_over);
    } else {
        this->udp_send_switch_ = true;
    }
    setROSmsgUpdateFalse();
}

}