#ifndef ECU_COMMUNICATION_COMMUNICATIONPROCESS_HPP
#define ECU_COMMUNICATION_COMMUNICATIONPROCESS_HPP

#include <ros/ros.h>
#include <cmath>
#include <thread>
#include <mutex>
#include <vector>
#include <glog/logging.h>
#include <dynamic_reconfigure/server.h>
#include "ecu_communication/ecu_communicationParameters.h"
#include "three_one_msgs/control.h"
#include "three_one_msgs/report.h"
#include "ThreeOne.hpp"
#include "UDPClient.hpp"
#include "UDPServer.hpp"
#include "DataUpload.hpp"
#include "DataDownload.hpp"
#include "shawn/ShawnTime.hpp"

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
    bool upper_layer_send;
    bool upper_layer_receive;
    bool lower_layer_send;
    bool lower_layer_receive;
    bool send_default_when_no_msg;
    bool reconfig;
    bool verbose_log;
    std::string ecu_ip;
    int ecu_port;
    int udp_server_port;
    double_t publish_period;
    double_t check_period;
    double_t well_work_display_period;
    double_t essential_msg_max_period;
};


class CommunicationProcess {
public:
    CommunicationProcess(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
    ~CommunicationProcess();

public:
    //// time check
    //// 0 simple udp recv time
    //// 1 simple udp recv length correct time
    shawn::ShawnTime udp_recv_times_;
    //// 0 simple udp send time
    shawn::ShawnTime udp_send_times_;
    //// 0 1 2 3 for each pack recv time
    shawn::ShawnTime pack_recv_times_;
    //// 0 1 for each pack send time
    shawn::ShawnTime pack_send_times_;
    //// 0 simple ros publish time
    shawn::ShawnTime ros_publish_times_;

    //// Functions Switch
    bool upper_layer_send_;
    bool upper_layer_receive_;
    bool lower_layer_send_;
    bool lower_layer_receive_;

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
    bool verbose_log_;
    bool time_check_no_error_;

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

    void paramsInit();
    void reconfigureRequest(ecu_communicationConfig &config, uint32_t level);
    void udpReceive();
    void udpSend();
    void dataProcess();
    void dataUploadCopy();
    bool msgDistribution();
    void dataDownloadCopy(uint8_t pack_num);
    void fake_issue();
    void timeCheck();
    void udpReceiveCheck();
    void udpSendCheck();
    void rosPublishCheck();
    void rosmsgUpdateCheck();
    void errorLog(communication_process_error_type p_error);
    void logMarkers();
    void logVerboseInfo();
    void setROSmsgUpdateFalse();
};

}
#endif //ECU_COMMUNICATION_COMMUNICATIONPROCESS_HPP