#include "CommunicationProcess.hpp"

#define LOG_INFO LOG(INFO)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_WARN LOG(WARNING)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_ERROR LOG(ERROR)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_FATAL LOG(FATAL)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define FIXED std::setiosflags(std::ios::fixed)<<

namespace ecu_communication {
//// done todo data download init or emergency value when need
//// done todo time check
//// todo data upload and download check
//// todo add a mark in msg to check if valid msg
//// todo data upload init
//// todo some callback apply to current framework
//// todo start with self driving of not
//// todo fake issue
//// done todo bit opration push bit
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
                                                    boost::bind(&CommunicationProcess::time_check, this));
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
                error_handle(communication_process_error_type::udp_recv_ID_error);
            }
            this->data_upload_mutex_.unlock();
        } else {
            error_handle(communication_process_error_type::udp_recv_len_error);
        }
    }
}

void CommunicationProcess::udpSend() {
    while (!(this->udp_client_.init(this->ecu_ip_.data(), this->ecu_port_))) {
        ROS_ERROR_STREAM("UDP SEND INIT FAILURE, KEEP TRYING!");
    }
    ros::Rate loop_rate(20);
    while (this->udp_send_switch_) {
        dataDownloadCopy(1);
        this->last_udp_send_one_time_[0] = this->last_udp_send_one_time_[1];
        this->last_udp_send_one_time_[1] = ros::Time::now().toSec() * 1000;
        if (this->udp_client_.process(this->data_download_copied_.data_download_pack_one.result_data,
                                      sizeof(this->data_download_copied_.data_download_pack_one.result_data))) {
            this->last_udp_send_correct_one_time_[0] = this->last_udp_send_correct_one_time_[1];
            this->last_udp_send_correct_one_time_[1] = ros::Time::now().toSec() * 1000;
        } else {
            error_handle(communication_process_error_type::udp_send_pack_one_len_error);
        }

        ros::Duration(0.025).sleep();

        dataDownloadCopy(2);
        this->last_udp_send_two_time_[0] = this->last_udp_send_two_time_[1];
        this->last_udp_send_two_time_[1] = ros::Time::now().toSec() * 1000;
        if (this->udp_client_.process(this->data_download_copied_.data_download_pack_two.result_data,
                                      sizeof(this->data_download_copied_.data_download_pack_two.result_data))) {
            this->last_udp_send_correct_two_time_[0] = this->last_udp_send_correct_two_time_[1];
            this->last_udp_send_correct_two_time_[1] = ros::Time::now().toSec() * 1000;
        } else {
            error_handle(communication_process_error_type::udp_send_pack_two_len_error);
        }

        loop_rate.sleep();
    }
}

CommunicationProcess::~CommunicationProcess() {
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
    if (!msg_distribution()) {
        error_handle(communication_process_error_type::udp_receive_data_illegal);
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

bool CommunicationProcess::msg_distribution() {
    //// todo data check if right

    this->tmp_union_16_data.data[0] = this->data_upload_copied_.data_upload_pack_one.cell.left_wheel_expect_speed_low_byte;
    this->tmp_union_16_data.data[1] = this->data_upload_copied_.data_upload_pack_one.cell.left_wheel_expect_speed_high_byte;
    if (this->tmp_union_16_data.result > 25000) {
        return false;
    }
    this->report_.give_back.left_wheel_expect_speed = this->tmp_union_16_data.result * 0.001;
    this->tmp_union_16_data.data[0] = this->data_upload_copied_.data_upload_pack_one.cell.right_wheel_expect_speed_low_byte;
    this->tmp_union_16_data.data[1] = this->data_upload_copied_.data_upload_pack_one.cell.right_wheel_expect_speed_high_byte;
    if (this->tmp_union_16_data.result > 25000) {
        return false;
    }
    this->report_.give_back.right_wheel_expect_speed = this->tmp_union_16_data.result * 0.001;
    this->tmp_union_16_data.data[0] = this->data_upload_copied_.data_upload_pack_one.cell.mechanical_brake_low_byte;
    this->tmp_union_16_data.data[1] = this->data_upload_copied_.data_upload_pack_one.cell.mechanical_brake_high_byte;
    if (this->tmp_union_16_data.result > 1000) {
        return false;
    }
    this->report_.vehicle_state.mechanical_brake = this->tmp_union_16_data.result * 0.1;
    this->report_.vehicle_state.vehicle_speed = this->data_upload_copied_.data_upload_pack_one.cell.vehicle_speed * 0.1;
    if (this->data_upload_copied_.data_upload_pack_one.cell.vehicle_speed > 250) {
        return false;
    }
    this->report_.vehicle_state.current_gear = this->data_upload_copied_.data_upload_pack_one.cell.gear;
    if ((this->data_upload_copied_.data_upload_pack_one.cell.gear < 1) || (this->data_upload_copied_.data_upload_pack_one.cell.gear > 3)) {
        return false;
    }

    this->tmp_union_16_data.data[0] = this->data_upload_copied_.data_upload_pack_two.cell.left_motor_actual_speed_low_byte;
    this->tmp_union_16_data.data[1] = this->data_upload_copied_.data_upload_pack_two.cell.left_motor_actual_speed_high_byte;
    if (this->tmp_union_16_data.result > 10000) {
        return false;
    }
    this->report_.vehicle_state.left_motor_rpm = this->tmp_union_16_data.result;
    this->tmp_union_16_data.data[0] = this->data_upload_copied_.data_upload_pack_two.cell.right_motor_actual_speed_low_byte;
    this->tmp_union_16_data.data[1] = this->data_upload_copied_.data_upload_pack_two.cell.right_motor_actual_speed_high_byte;
    if (this->tmp_union_16_data.result > 10000) {
        return false;
    }
    this->report_.vehicle_state.right_motor_rpm = this->tmp_union_16_data.result;
    this->report_.vehicle_state.left_wheel_rotate = this->data_upload_copied_.data_upload_pack_two.cell.left_motor_gear;
    if (this->data_upload_copied_.data_upload_pack_two.cell.left_motor_gear > 1) {
        return false;
    }
    this->report_.vehicle_state.right_wheel_rotate = this->data_upload_copied_.data_upload_pack_two.cell.right_motor_gear;
    if (this->data_upload_copied_.data_upload_pack_two.cell.right_motor_gear > 1) {
        return false;
    }
    this->report_.vehicle_state.SOC = this->data_upload_copied_.data_upload_pack_two.cell.SOC;
    if (this->data_upload_copied_.data_upload_pack_two.cell.SOC > 100) {
        return false;
    }
    this->report_.vehicle_state.tailgate_state = this->data_upload_copied_.data_upload_pack_two.cell.tailgate_state;
    if (this->data_upload_copied_.data_upload_pack_two.cell.tailgate_state > 2) {
        return false;
    }

    this->report_.cylinder_position.left_one_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.cell.left_one_cylinder_position * 2;
    if (this->data_upload_copied_.data_upload_pack_three.cell.left_one_cylinder_position > 250) {
        return false;
    }
    this->report_.cylinder_position.left_two_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.cell.left_two_cylinder_position * 2;
    if (this->data_upload_copied_.data_upload_pack_three.cell.left_two_cylinder_position > 250) {
        return false;
    }
    this->report_.cylinder_position.left_three_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.cell.left_three_cylinder_position * 2;
    if (this->data_upload_copied_.data_upload_pack_three.cell.left_three_cylinder_position > 250) {
        return false;
    }
    this->report_.cylinder_position.left_four_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.cell.left_four_cylinder_position * 2;
    if (this->data_upload_copied_.data_upload_pack_three.cell.left_four_cylinder_position > 250) {
        return false;
    }
    this->report_.cylinder_position.right_one_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.cell.right_one_cylinder_position * 2;
    if (this->data_upload_copied_.data_upload_pack_three.cell.right_one_cylinder_position > 250) {
        return false;
    }
    this->report_.cylinder_position.right_two_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.cell.right_two_cylinder_position * 2;
    if (this->data_upload_copied_.data_upload_pack_three.cell.right_two_cylinder_position > 250) {
        return false;
    }
    this->report_.cylinder_position.right_three_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.cell.right_three_cylinder_position * 2;
    if (this->data_upload_copied_.data_upload_pack_three.cell.right_three_cylinder_position > 250) {
        return false;
    }
    this->report_.cylinder_position.right_four_cylinder_position =
            this->data_upload_copied_.data_upload_pack_three.cell.right_four_cylinder_position * 2;
    if (this->data_upload_copied_.data_upload_pack_three.cell.right_four_cylinder_position > 250) {
        return false;
    }

    this->report_.cylinder_pressure.left_one_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.cell.left_one_cylinder_pressure;
    if (this->data_upload_copied_.data_upload_pack_four.cell.left_one_cylinder_pressure > 250) {
        return false;
    }
    this->report_.cylinder_pressure.left_two_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.cell.left_two_cylinder_pressure;
    if (this->data_upload_copied_.data_upload_pack_four.cell.left_two_cylinder_pressure > 250) {
        return false;
    }
    this->report_.cylinder_pressure.left_three_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.cell.left_three_cylinder_pressure;
    if (this->data_upload_copied_.data_upload_pack_four.cell.left_three_cylinder_pressure > 250) {
        return false;
    }
    this->report_.cylinder_pressure.left_four_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.cell.left_four_cylinder_pressure;
    if (this->data_upload_copied_.data_upload_pack_four.cell.left_four_cylinder_pressure > 250) {
        return false;
    }
    this->report_.cylinder_pressure.right_one_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.cell.right_one_cylinder_pressure;
    if (this->data_upload_copied_.data_upload_pack_four.cell.right_one_cylinder_pressure > 250) {
        return false;
    }
    this->report_.cylinder_pressure.right_two_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.cell.right_two_cylinder_pressure;
    if (this->data_upload_copied_.data_upload_pack_four.cell.right_two_cylinder_pressure > 250) {
        return false;
    }
    this->report_.cylinder_pressure.right_three_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.cell.right_three_cylinder_pressure;
    if (this->data_upload_copied_.data_upload_pack_four.cell.right_three_cylinder_pressure > 250) {
        return false;
    }
    this->report_.cylinder_pressure.right_four_cylinder_pressure =
            this->data_upload_copied_.data_upload_pack_four.cell.right_four_cylinder_pressure;
    if (this->data_upload_copied_.data_upload_pack_four.cell.right_four_cylinder_pressure > 250) {
        return false;
    }
}

void CommunicationProcess::error_handle(communication_process_error_type p_error) {
    //// todo add ros time in each log
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

void CommunicationProcess::time_check() {
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
        error_handle(communication_process_error_type::udp_send_time_error);
    }

    if ((this->ros_publish_switch_) &&
        ((this->last_publish_interval_ < 45) ||
         (this->last_publish_interval_ > 55) ||
         (this->last_publish_till_now_ > 55))) {
        no_error = false;
        ROS_ERROR_STREAM("ERROR in ROS Publish!");
        error_handle(communication_process_error_type::ros_publish_time_error);
    }

    if ((this->udp_receive_switch_) &&
        ((this->last_udp_receive_interval_ < 8) ||
         (this->last_udp_receive_correct_interval_ > 25) ||
         (this->last_udp_receive_correct_till_now_ > 25))) {
        no_error = false;
        this->ros_publish_switch_ = false;
        ROS_ERROR_STREAM("ERROR in UDP Receive!");
        error_handle(communication_process_error_type::udp_receive_time_error);
    } else {
        this->ros_publish_switch_ = true;
    }

    if (this->ros_msg_update_.essential.result != this->ros_msg_update_.essential_yes) {
        no_error = false;
        this->udp_send_switch_ = false;
        ROS_ERROR_STREAM("ERROR NO ROS msg Receive!");
        error_handle(communication_process_error_type::ros_msg_receive_time_over);
    } else {
        this->udp_send_switch_ = true;
    }
    setROSmsgUpdateFalse();

    if (no_error) {
        //// todo well_work_display_period / check_period
        ROS_INFO_STREAM_THROTTLE(5, "work well");
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

void CommunicationProcess::variablesInit() {

    this->udp_send_switch_ = true;


}

void CommunicationProcess::paramsInit() {
    //// todo change default value
    this->yaml_params_.send_default_when_no_msg = this->private_nh_.param("send_default_when_no_msg", false);
    this->yaml_params_.ecu_ip = this->private_nh_.param<std::string>("ecu_ip", "192.168.1.22");
    this->yaml_params_.ecu_port = (uint16_t)this->private_nh_.param("ecu_port", 8080);
    this->yaml_params_.udp_server_port = (uint16_t)this->private_nh_.param("udp_server_port", 8081);
    this->yaml_params_.publish_period = this->private_nh_.param("publish_period", 20) * 0.001;
    this->yaml_params_.check_period = this->private_nh_.param("check_period", 200) * 0.001;
    //// todo more than essential msg period

    this->send_default_when_no_msg_ = this->yaml_params_.send_default_when_no_msg;
    this->ecu_ip_ = this->yaml_params_.ecu_ip;
    this->ecu_port_ = this->yaml_params_.ecu_port;
    this->udp_server_port_ = this->yaml_params_.udp_server_port;
    this->publish_period_ = this->yaml_params_.publish_period;
    this->check_period_ = this->yaml_params_.check_period;

    this->udp_send_switch_ = false;
    this->udp_receive_switch_ = false;
    this->ros_publish_switch_ = false;

    this->ros_msg_update_.essential.result = 255;
    this->ros_msg_update_.essential_yes = 255;
    setROSmsgUpdateFalse();
}

void CommunicationProcess::setROSmsgUpdateFalse() {
    this->ros_msg_update_.essential.path_plan = 0;
}

}