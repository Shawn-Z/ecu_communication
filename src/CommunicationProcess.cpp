#include "CommunicationProcess.hpp"

#define LOG_INFO LOG(INFO)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_WARN LOG(WARNING)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_ERROR LOG(ERROR)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_FATAL LOG(FATAL)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define FIXED std::setiosflags(std::ios::fixed)<<

namespace ecu_communication {
//// full done todo data download init or emergency value when need
//// full done todo time check
//// todo data download check
//// todo add a mark in msg to check if valid msg
//// full done todo data upload init
//// todo some callback apply to current framework
//// full done todo start with self driving of not
//// todo fake issue
//// todo log marks
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

    //// udp receive
    this->udp_receive_thread = std::thread(&CommunicationProcess::udpReceive, this);
    this->udp_receive_thread.detach();

    //// udp send
    this->udp_send_thread = std::thread(&CommunicationProcess::udpSend, this);
    this->udp_send_thread.detach();

    //// timer for process and publish
    this->data_process_timer_ = this->nh_.createTimer(ros::Duration(this->publish_period_),
                                                      boost::bind(&CommunicationProcess::dataProcess, this));

    //// add publisher here
    this->recv_data_publisher_ = this->nh_.advertise<three_one_msgs::report>("/ecudatareport", 1);

    //// add subscriber here


    this->udp_receive_switch_ = true;

    //// start time check after a little while
    ros::Duration(this->check_period_).sleep();
    this->time_check_timer_ = this->nh_.createTimer(ros::Duration(this->check_period_),
                                                    boost::bind(&CommunicationProcess::timeCheck, this));
}


void CommunicationProcess::udpReceive() {
    while (!(this->udp_server_.init(this->udp_server_port_))) {
        ROS_ERROR_STREAM("UDP RECEIVE INIT FAILURE, KEEP TRYING!");
    }
    while (this->udp_receive_switch_) {
        if (this->udp_server_.process(this->data_upload_.recv_raw_data, sizeof(this->data_upload_.recv_raw_data))) {
            this->last_udp_receive_time_[0] = this->last_udp_receive_time_[1];
            this->last_udp_receive_time_[1] = ros::Time::now().toSec() * 1000;
            this->data_upload_mutex_.lock();
            if (this->data_upload_.dataDistribution()) {
                this->last_udp_receive_correct_time_[0] = this->last_udp_receive_correct_time_[1];
                this->last_udp_receive_correct_time_[1] = ros::Time::now().toSec() * 1000;
            } else {
                errorLog(communication_process_error_type::udp_recv_ID_error);
            }
            this->data_upload_mutex_.unlock();
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
    while (this->udp_send_switch_ || this->send_default_when_no_msg_ || this->fake_issue_) {
        dataDownloadCopy(1);
        if (!this->udp_send_switch_) {
            this->data_download_copied_.init();
        }
        if (this->fake_issue_) {
            fake_issue();
        }
        this->last_udp_send_one_time_[0] = this->last_udp_send_one_time_[1];
        this->last_udp_send_one_time_[1] = ros::Time::now().toSec() * 1000;
        if (this->udp_client_.process(this->data_download_copied_.data_download_pack_one.result_data,
                                      sizeof(this->data_download_copied_.data_download_pack_one.result_data))) {
            this->last_udp_send_correct_one_time_[0] = this->last_udp_send_correct_one_time_[1];
            this->last_udp_send_correct_one_time_[1] = ros::Time::now().toSec() * 1000;
        } else {
            errorLog(communication_process_error_type::udp_send_pack_one_len_error);
        }

        ros::Duration(0.025).sleep();

        dataDownloadCopy(2);
        if (!this->udp_send_switch_) {
            this->data_download_copied_.init();
        }
        if (this->fake_issue_) {
            fake_issue();
        }
        this->last_udp_send_two_time_[0] = this->last_udp_send_two_time_[1];
        this->last_udp_send_two_time_[1] = ros::Time::now().toSec() * 1000;
        if (this->udp_client_.process(this->data_download_copied_.data_download_pack_two.result_data,
                                      sizeof(this->data_download_copied_.data_download_pack_two.result_data))) {
            this->last_udp_send_correct_two_time_[0] = this->last_udp_send_correct_two_time_[1];
            this->last_udp_send_correct_two_time_[1] = ros::Time::now().toSec() * 1000;
        } else {
            errorLog(communication_process_error_type::udp_send_pack_two_len_error);
        }

        loop_rate.sleep();
    }
}

CommunicationProcess::~CommunicationProcess() {
    //// todo
    LOG_INFO << "program end";
    LOG_WARN << "program end";
    LOG_ERROR << "program end";
    google::ShutdownGoogleLogging();
    //// todo shutdown more switch
    this->udp_receive_switch_ = false;
    this->udp_send_switch_ = false;
    this->ros_publish_switch_ = false;
    //// todo
//    ros::Duration(0.1).sleep();
}

void CommunicationProcess::dataProcess() {
    dataUploadCopy();
    if (!msgDistribution()) {
        errorLog(communication_process_error_type::udp_receive_data_illegal);
        return;
    }
    if (this->ros_publish_switch_) {
        this->recv_data_publisher_.publish(this->report_);
        this->last_publish_time_[0] = this->last_publish_time_[1];
        this->last_publish_time_[1] = ros::Time::now().toSec() * 1000;
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
    switch ((int)p_error) {
        case 1: {
            //// todo check log uint8
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
            LOG_ERROR << "last_udp_receive_interval_: " << FIXED this->last_udp_receive_interval_;
            LOG_ERROR << "last_udp_receive_till_now_: " << FIXED this->last_udp_receive_till_now_;
            LOG_ERROR << "last_udp_receive_correct_interval_: " << FIXED this->last_udp_receive_correct_interval_;
            LOG_ERROR << "last_udp_receive_correct_till_now_: " << FIXED this->last_udp_receive_correct_till_now_;
            break;
        }
        case 6: {
            LOG_ERROR << "last_udp_send_one_interval_: " << FIXED this->last_udp_send_one_interval_;
            LOG_ERROR << "last_udp_send_one_till_now_: " << FIXED this->last_udp_send_one_till_now_;
            LOG_ERROR << "last_udp_send_correct_one_interval_: " << FIXED this->last_udp_send_correct_one_interval_;
            LOG_ERROR << "last_udp_send_correct_one_till_now_: " << FIXED this->last_udp_send_correct_one_till_now_;
            LOG_ERROR << "last_udp_send_two_interval_: " << FIXED this->last_udp_send_two_interval_;
            LOG_ERROR << "last_udp_send_two_till_now_: " << FIXED this->last_udp_send_two_till_now_;
            LOG_ERROR << "last_udp_send_correct_two_interval_: " << FIXED this->last_udp_send_correct_two_interval_;
            LOG_ERROR << "last_udp_send_correct_two_till_now_: " << FIXED this->last_udp_send_correct_two_till_now_;
            LOG_ERROR << "last_udp_send_interval_: " << FIXED this->last_udp_send_interval_;
            break;
        }
        case 7: {
            //// todo check no error, error try new way
//            LOG(ERROR) << "udp_receive_data_illegal: " << std::hex << this->data_upload_copied_.data_upload_pack_one.pack;
//            LOG(ERROR) << "udp_receive_data_illegal: " << std::hex << this->data_upload_copied_.data_upload_pack_two.pack;
//            LOG(ERROR) << "udp_receive_data_illegal: " << std::hex << this->data_upload_copied_.data_upload_pack_three.pack;
//            LOG(ERROR) << "udp_receive_data_illegal: " << std::hex << this->data_upload_copied_.data_upload_pack_four.pack;
            break;
        }
        case 8: {
            LOG_ERROR << "last_publish_interval_" << FIXED this->last_publish_interval_;
            LOG_ERROR << "last_publish_till_now_" << FIXED this->last_publish_till_now_;
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
    this->tmp_ros_time_now_ = ros::Time::now().toSec() * 1000;

    this->last_udp_receive_interval_ = this->last_udp_receive_time_[1] - this->last_udp_receive_time_[0];
    this->last_udp_receive_till_now_ = this->tmp_ros_time_now_ - this->last_udp_receive_time_[1];

    this->last_udp_receive_correct_interval_ = this->last_udp_receive_correct_time_[1] - this->last_udp_receive_correct_time_[0];
    this->last_udp_receive_correct_till_now_ = this->tmp_ros_time_now_ - this->last_udp_receive_correct_time_[1];

    this->last_udp_send_one_interval_ = this->last_udp_send_one_time_[1] - this->last_udp_send_one_time_[0];
    this->last_udp_send_one_till_now_ = this->tmp_ros_time_now_ - this->last_udp_send_one_time_[1];

    this->last_udp_send_correct_one_interval_ = this->last_udp_send_correct_one_time_[1] - this->last_udp_send_correct_one_time_[0];
    this->last_udp_send_correct_one_till_now_ = this->tmp_ros_time_now_ - this->last_udp_send_correct_one_time_[1];

    this->last_udp_send_two_interval_ = this->last_udp_send_two_time_[1] - this->last_udp_send_two_time_[0];
    this->last_udp_send_two_till_now_ = this->tmp_ros_time_now_ - this->last_udp_send_two_time_[1];

    this->last_udp_send_correct_two_interval_ = this->last_udp_send_correct_two_time_[1] - this->last_udp_send_correct_two_time_[0];
    this->last_udp_send_correct_two_till_now_ = this->tmp_ros_time_now_ - this->last_udp_send_correct_two_time_[1];

    this->last_udp_send_interval_ = fabs(this->last_udp_send_one_time_[1] - this->last_udp_send_two_time_[1]);

    this->last_publish_interval_ = this->last_publish_time_[1] - this->last_publish_time_[0];
    this->last_publish_till_now_ = this->tmp_ros_time_now_ - this->last_publish_time_[1];


    bool no_error = true;

    if ((this->udp_send_switch_) &&
        ((this->last_udp_send_one_interval_ < 45) ||
         (this->last_udp_send_correct_one_interval_ > 55) ||
         (this->last_udp_send_two_interval_ < 45) ||
         (this->last_udp_send_correct_two_interval_ > 55) ||
         (this->last_udp_send_correct_one_till_now_ > 60) ||
         (this->last_udp_send_correct_two_till_now_ > 60) ||
         (this->last_udp_send_interval_ < 20) ||
         (this->last_udp_send_interval_ > 30))) {
        no_error = false;
        ROS_ERROR_STREAM("ERROR in UDP Send!");
        errorLog(communication_process_error_type::udp_send_time_error);
    }

    if ((this->ros_publish_switch_) &&
        ((this->last_publish_interval_ < 45) ||
         (this->last_publish_interval_ > 55) ||
         (this->last_publish_till_now_ > 55))) {
        no_error = false;
        ROS_ERROR_STREAM("ERROR in ROS Publish!");
        errorLog(communication_process_error_type::ros_publish_time_error);
    }

    if ((this->udp_receive_switch_) &&
        ((this->last_udp_receive_interval_ < 8) ||
         (this->last_udp_receive_correct_interval_ > 25) ||
         (this->last_udp_receive_correct_till_now_ > 25))) {
        no_error = false;
        this->ros_publish_switch_ = false;
        ROS_ERROR_STREAM("ERROR in UDP Receive!");
        errorLog(communication_process_error_type::udp_receive_time_error);
    } else {
        this->ros_publish_switch_ = true;
    }

    if (this->ros_msg_update_.essential.result != this->ros_msg_update_.essential_yes) {
        no_error = false;
        this->udp_send_switch_ = false;
        ROS_ERROR_STREAM("ERROR NO ROS msg Receive!");
        errorLog(communication_process_error_type::ros_msg_receive_time_over);
    } else {
        this->udp_send_switch_ = true;
    }
    setROSmsgUpdateFalse();

    if (no_error) {
        ROS_INFO_STREAM_THROTTLE(std::max((int)(this->well_work_display_period_ / this->check_period_), 1), "work well");
    }
}

void CommunicationProcess::logVerboseInfo() {
    //// following rules are not correct always, only serves at same time t.
    //// last_udp_receive_interval_ <= last_udp_receive_correct_interval_
    //// last_udp_receive_till_now_ <= last_udp_receive_correct_till_now_
    //// last_udp_send_one_interval_ <= last_udp_send_correct_one_interval_
    //// last_udp_send_one_till_now_ <= last_udp_send_correct_one_till_now_
    //// last_udp_send_two_interval_ <= last_udp_send_correct_two_interval_
    //// last_udp_send_two_till_now_ <= last_udp_send_correct_two_till_now_
    LOG(INFO) << "last_udp_receive_interval_: " << this->last_udp_receive_interval_;
    LOG(INFO) << "last_udp_receive_till_now_: " << this->last_udp_receive_till_now_;
    LOG(INFO) << "last_udp_receive_correct_interval_: " << this->last_udp_receive_correct_interval_;
    LOG(INFO) << "last_udp_receive_correct_till_now_: " << this->last_udp_receive_correct_till_now_;
    LOG(INFO) << "last_udp_send_one_interval_: " << this->last_udp_send_one_interval_;
    LOG(INFO) << "last_udp_send_one_till_now_: " << this->last_udp_send_one_till_now_;
    LOG(INFO) << "last_udp_send_correct_one_interval_: " << this->last_udp_send_correct_one_interval_;
    LOG(INFO) << "last_udp_send_correct_one_till_now_: " << this->last_udp_send_correct_one_till_now_;
    LOG(INFO) << "last_udp_send_two_interval_: " << this->last_udp_send_two_interval_;
    LOG(INFO) << "last_udp_send_two_till_now_: " << this->last_udp_send_two_till_now_;
    LOG(INFO) << "last_udp_send_correct_two_interval_: " << this->last_udp_send_correct_two_interval_;
    LOG(INFO) << "last_udp_send_correct_two_till_now_: " << this->last_udp_send_correct_two_till_now_;
    LOG(INFO) << "last_udp_send_interval_: " << this->last_udp_send_interval_;
    LOG(INFO) << "last_publish_interval_: " << this->last_publish_interval_;
    LOG(INFO) << "last_publish_till_now_: " << this->last_publish_till_now_;
}

void CommunicationProcess::paramsInit() {
    //// todo change default value
    this->yaml_params_.reconfig = this->private_nh_.param("reconfig", false);
    this->yaml_params_.fake_issue = this->private_nh_.param("fake_issue", false);
    this->yaml_params_.send_default_when_no_msg = this->private_nh_.param("send_default_when_no_msg", false);
    this->yaml_params_.ecu_ip = this->private_nh_.param<std::string>("ecu_ip", "192.168.1.22");
    this->yaml_params_.ecu_port = (uint16_t)this->private_nh_.param("ecu_port", 8080);
    this->yaml_params_.udp_server_port = (uint16_t)this->private_nh_.param("udp_server_port", 8081);
    this->yaml_params_.publish_period = this->private_nh_.param("publish_period", 20) * 0.001;
    this->yaml_params_.check_period = this->private_nh_.param("check_period", 200) * 0.001;
    this->yaml_params_.essential_msg_max_period = this->private_nh_.param("essential_msg_max_period", 100) * 0.001;
    this->yaml_params_.well_work_display_period = this->private_nh_.param("well_work_display_period", 1000) * 0.001;

    this->reconfig_ = this->yaml_params_.reconfig;
    this->fake_issue_ = this->yaml_params_.fake_issue;
    this->send_default_when_no_msg_ = this->yaml_params_.send_default_when_no_msg;
    this->ecu_ip_ = this->yaml_params_.ecu_ip;
    this->ecu_port_ = this->yaml_params_.ecu_port;
    this->udp_server_port_ = this->yaml_params_.udp_server_port;
    this->publish_period_ = this->yaml_params_.publish_period;
    this->check_period_ = this->yaml_params_.check_period;
    this->essential_msg_max_period_ = this->yaml_params_.essential_msg_max_period;
    this->well_work_display_period_ = this->yaml_params_.well_work_display_period;

    //// check_period > essential_msg_max_period
    if (this->check_period_ < (this->essential_msg_max_period_ + 0.01)) {
        this->check_period_ = (this->essential_msg_max_period_ + 0.01);
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
    //// todo
}

void CommunicationProcess::fake_issue() {
    //// todo
}

void CommunicationProcess::logMarkers() {
    LOG_INFO <<
}

}