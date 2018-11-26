#ifndef ECU_COMMUNICATION_COMMUNICATIONPROCESS_HPP
#define ECU_COMMUNICATION_COMMUNICATIONPROCESS_HPP

#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <vector>
#include <glog/logging.h>
#include <dynamic_reconfigure/server.h>
#include "ecu_communication/ecu_communicationParameters.h"
#include "three_one_msgs/control.h"
#include "three_one_msgs/report.h"
#include "UDPClient.hpp"
#include "UDPServer.hpp"
#include "DataUpload.hpp"
#include "DataDownload.hpp"

namespace ecu_communication {

enum class communication_process_error_type {
    udp_recv_len_error = 1,
    udp_recv_ID_error = 2,
    udp_send_pack_one_len_error = 3,
    udp_send_pack_two_len_error = 4,
    udp_receive_time_error = 5,
    udp_send_time_error = 6,
    udp_receive_data_illegal = 7,
    ros_publish_time_error = 8,
    ros_msg_receive_time_over = 9
};

struct ros_msg_update_type {
    union msg_update_type {
        struct {
             uint8_t path_plan: 1;
        };
        uint8_t result;
    } essential;
    uint8_t essential_yes;
};

struct yaml_params_type {
    double_t publish_period;
    double_t check_period;
    std::string ecu_ip;
    uint16_t ecu_port;
    uint16_t udp_server_port;
    bool send_default_when_no_msg;
    double_t well_work_display_period;
    double_t essential_msg_max_period;
    bool fake_issue;
    bool reconfig;
};


class CommunicationProcess {
public:
    CommunicationProcess(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
    ~CommunicationProcess();

private:
    //// ROS Variables
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Timer data_process_timer_;
    ros::Timer time_check_timer_;
    ros::Publisher recv_data_publisher_;

    //// ROS msgs
    three_one_msgs::report report_;

    //// rosparam handler
    ecu_communicationParameters params_;
    dynamic_reconfigure::Server<ecu_communicationConfig> reconfigSrv_;
    yaml_params_type yaml_params_;

    //// markers
    ros_msg_update_type ros_msg_update_;
    bool udp_receive_switch_;
    bool udp_send_switch_;
    bool ros_publish_switch_;
    bool send_default_when_no_msg_;
    bool reconfig_;
    bool fake_issue_;

    //// UDP communication
    UDPServer udp_server_;
    UDPClient udp_client_;
    uint16_t udp_server_port_;
    std::string ecu_ip_;
    uint16_t ecu_port_;
    std::thread udp_receive_thread;
    std::thread udp_send_thread;

    //// UDP communication data and variables
    DataUpload data_upload_;
    DataUpload data_upload_copied_;
    DataDownload data_download_;
    DataDownload data_download_copied_;
    std::mutex data_upload_mutex_;
    std::mutex data_download_pack_one_mutex_;
    std::mutex data_download_pack_two_mutex_;

    //// periods
    double_t check_period_;
    double_t publish_period_;
    double_t essential_msg_max_period_;
    double_t well_work_display_period_;

    //// for time check
    double_t last_udp_receive_time_[2] = {0.0, 0.0};
    double_t last_udp_receive_interval_ = 0;
    double_t last_udp_receive_till_now_ = 0;
    double_t last_udp_receive_correct_time_[2] = {0.0, 0.0};
    double_t last_udp_receive_correct_interval_ = 0;
    double_t last_udp_receive_correct_till_now_ = 0;

    double_t last_udp_send_one_time_[2] = {0.0, 0.0};
    double_t last_udp_send_one_interval_ = 0;
    double_t last_udp_send_one_till_now_ = 0;
    double_t last_udp_send_correct_one_time_[2] = {0.0, 0.0};
    double_t last_udp_send_correct_one_interval_ = 0;
    double_t last_udp_send_correct_one_till_now_ = 0;

    double_t last_udp_send_two_time_[2] = {0.0, 0.0};
    double_t last_udp_send_two_interval_ = 0;
    double_t last_udp_send_two_till_now_ = 0;
    double_t last_udp_send_correct_two_time_[2] = {0.0, 0.0};
    double_t last_udp_send_correct_two_interval_ = 0;
    double_t last_udp_send_correct_two_till_now_ = 0;

    double_t last_udp_send_interval_ = 0;

    double_t last_publish_time_[2] = {0.0, 0.0};
    double_t last_publish_interval_ = 0;
    double_t last_publish_till_now_ = 0;

    double_t tmp_ros_time_now_ = 0;

    void paramsInit();
    void reconfigureRequest(ecu_communicationConfig &config, uint32_t level);
    void udpReceive();
    void udpSend();
    void dataProcess();
    void dataUploadCopy();
    bool msgDistribution();
    void dataDownloadCopy(uint8_t pack_num);
    void fake_issue(); //// todo implement
    void timeCheck();
    void errorLog(communication_process_error_type p_error);
    void logMarkers();
    void setROSmsgUpdateFalse();

    void logVerboseInfo(); //// todo
};

}
#endif //ECU_COMMUNICATION_COMMUNICATIONPROCESS_HPP