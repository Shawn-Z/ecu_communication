#ifndef ECU_COMMUNICATION_AUTONOMOUSCONTROL_HPP
#define ECU_COMMUNICATION_AUTONOMOUSCONTROL_HPP

#include <ros/ros.h>
#include <cmath>
#include <thread>
#include <mutex>
#include <vector>

#include "speed_ctrl_msgs/speed_ctrl.h"

#include "STime.hpp"
#include "SLog.hpp"
#include "SHandle.hpp"

#include "DEFINEs.hpp"
#include "DataUpload.hpp"
#include "DataDownload.hpp"

namespace ecu_communication {

class AutonomousControl {
public:
    //// time check
    shawn::STime msg_update_times;
    shawn::handle speed_sub_handle_;
    shawn::handle steer_sub_handle_;

    //// ROS Variables
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Timer data_process_timer_;
    ros::Subscriber speed_sub_;

    //// markers
    bool send_switch_;
    bool receive_switch_;

    //// communication data and variables
    DataUpload *p_data_upload_;
    DataDownload *p_data_download_;
    std::mutex *p_data_upload_mutex_;


    void init(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle,
              DataDownload *p_data_download, DataUpload *p_data_upload, std::mutex *p_data_upload_mutex);
    void setHandles();
    void receive_init();
    void send_init();
    void dataProcess();
    void reportControlData();
    bool rosmsgUpdateCheck();
    void speedCb(speed_ctrl_msgs::speed_ctrl msg);
};

}

#endif //ECU_COMMUNICATION_AUTONOMOUSCONTROL_HPP