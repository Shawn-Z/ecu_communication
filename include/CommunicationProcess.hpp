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
#include "three_one_msgs/recv_rawdata.h"
#include "three_one_msgs/send_rawdata.h"
#include "ThreeOne.hpp"
#include "UDPClient.hpp"
#include "UDPServer.hpp"
#include "DataUpload.hpp"
#include "DataDownload.hpp"
#include "STime.hpp"
#include "SLog.hpp"
#include "SProportion.hpp"

namespace ecu_communication {

struct yaml_params_type {
    bool upper_layer_send;
    bool upper_layer_receive;
    bool lower_layer_send;
    bool lower_layer_receive;

    std::string ecu_ip;
    int ecu_port;
    int udp_server_port;

    double_t publish_period;
    double_t check_period;
    double_t udp_send_rate;

    bool reconfig;
    bool send_default_when_no_msg;
    bool log_rawdata;
    bool publish_rawdata;
};


class CommunicationProcess {
public:
    CommunicationProcess(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
    ~CommunicationProcess();

public:
    shawn::SLog sLog_;

    //// time check
    shawn::STime udp_recv_times_;
    shawn::handle udp_recv_handle_;
    shawn::handle udp_recv_correct_handle_;

    shawn::STime pack_recv_times_;
    shawn::handle pack1_recv_handle_;
    shawn::handle pack2_recv_handle_;
    shawn::handle pack3_recv_handle_;
    shawn::handle pack4_recv_handle_;
    shawn::handle pack5_recv_handle_;
    shawn::handle pack6_recv_handle_;
    shawn::handle pack7_recv_handle_;

    shawn::STime msg_update_times;

    //// ROS Variables
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Timer data_process_timer_;
    ros::Timer time_check_timer_;
    ros::Publisher recv_data_publisher_;
    ros::Publisher udp_recv_rawdata_publisher_;
    ros::Publisher udp_send_rawdata_publisher_;

    //// rosparam handler
    ecu_communicationParameters params_;
    dynamic_reconfigure::Server<ecu_communicationConfig> reconfigSrv_;
    yaml_params_type yaml_params_;

    //// markers
    bool udp_send_switch_;
    bool ros_publish_switch_;

    //// UDP communication
    UDPServer udp_server_;
    UDPClient udp_client_;
    std::thread udp_receive_thread;
    std::thread udp_send_thread;

    //// UDP communication data and variables
    DataUpload data_upload_;
    DataDownload data_download_;
    std::mutex data_upload_mutex_;
    std::mutex data_download_mutex_;

    void paramsInit();
    void reconfigureRequest(ecu_communicationConfig &config, uint32_t level);
    void udpReceive();
    void udpSend();
    void dataProcess();
    void fake_issue();
    void timeCheck();
    bool udpReceiveCheck();
    bool rosmsgUpdateCheck();
    void setTimeCheckHandle();
    void glogInit();
};

}
#endif //ECU_COMMUNICATION_COMMUNICATIONPROCESS_HPP