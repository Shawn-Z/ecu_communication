#ifndef ECU_COMMUNICATION_AUTONOMOUSCONTROL_HPP
#define ECU_COMMUNICATION_AUTONOMOUSCONTROL_HPP

#include <ros/ros.h>
#include <cmath>
#include <thread>
#include <mutex>
#include <vector>

#include "three_one_msgs/Control.h"
#include "sensor_driver_msgs/VehicleState.h"

#include "STime.hpp"
#include "SLog.hpp"
#include "SHandle.hpp"

#include "ThreeOne.hpp"
#include "DEFINEs.hpp"
#include "DataUpload.hpp"
#include "DataDownload.hpp"

#include "Transform6t.hpp"

namespace ecu_communication {

class AutonomousControl {
public:
    struct {
        struct {
            int current;
            int spare;
        } total;
        struct {
            uint8_t current;
            uint8_t spare;
        } speed;
        struct {
            uint8_t current;
            uint8_t spare;
        } steer;
    } msg_priority;

    ros::Timer priority_check_timer_;
    void priorityCheck();

    //// time check
    shawn::STime msg_update_times;
    shawn::handle speed_sub_handle_;
    shawn::handle steer_sub_handle_;
    shawn::handle gps_sub_handle_;
    shawn::handle suspension_sub_handle_;
    shawn::handle weapon_sub_handle_;

    //// ROS Variables
    ros::NodeHandle nh_;
    ros::Subscriber speed_sub_;
    ros::Subscriber steer_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber suspension_sub_;
    ros::Subscriber weapon_sub_;

    //// markers
    bool send_switch_;
    bool receive_switch_;

    //// communication data and variables
    DataUpload *p_data_upload_;
    DataDownload *p_data_download_;
    std::mutex *p_data_upload_mutex_;
    std::mutex *p_data_download_mutex_;
    sensor_driver_msgs::VehicleState *p_gps_;
    weapon::cmd *p_weapon_cmd_;

    three_one_feedback::control_mode *p_control_mode_;
    std::mutex *p_control_mode_mutex_;


    void init(ros::NodeHandle node_handle,
              DataDownload *p_data_download, DataUpload *p_data_upload,
              std::mutex *p_data_upload_mutex, std::mutex *p_data_download_mutex,
              three_one_feedback::control_mode *p_control_mode, std::mutex *p_control_mode_mutex,
              sensor_driver_msgs::VehicleState *p_gps,
              weapon::cmd *p_weapon_cmd);
    void setHandles();
    void receive_init();
    void dataProcess();
    void reportControlData();
    bool rosmsgUpdateCheck();
    void speedCb(three_one_msgs::ControlSpeed msg);
    void steerCb(three_one_msgs::ControlSteer msg);
    void gpsCb(sensor_driver_msgs::VehicleState msg);
    void suspensionCb(three_one_msgs::ControlSuspension msg);
    void weaponCb(three_one_msgs::ControlWeapon msg);
    bool gpsCheck();
    void gpsInit();
    bool weaponCheck();
    void weaponInit();
    bool suspensionCheck();
    void suspensionInit();

    Transform6t transform6t;
};

}

#endif //ECU_COMMUNICATION_AUTONOMOUSCONTROL_HPP