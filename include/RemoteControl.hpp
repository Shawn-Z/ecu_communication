#ifndef ECU_COMMUNICATION_REMOTECONTROL_HPP
#define ECU_COMMUNICATION_REMOTECONTROL_HPP

#include <mutex>
#include <ros/ros.h>

#include "STime.hpp"
#include "SProportion.hpp"
#include "SLog.hpp"

#include "DEFINEs.hpp"
#include "UDPServer.hpp"
#include "UDPClient.hpp"
#include "DataDownload.hpp"
#include "DataUpload.hpp"

#include "RemoteReceive.hpp"
#include "RemoteSend.hpp"

namespace ecu_communication {

class RemoteControl {

public:
    bool send_switch_;
    bool receive_switch_;

    uint8_t work_mode_;
    shawn::SLog *p_log_;

    uint8_t *p_work_mode_;

    ros::Timer data_send_timer_;
    ros::NodeHandle nh_;

    //// UDP communication data and variables
    uint16_t server_port_;

    UDPServer udp_server_;
    UDPClient udp_client_;

    shawn::SProportion udp_send_proportion_;
    shawn::handle udp_pack_handle_;

    DataUpload *p_data_upload_;
    DataDownload *p_data_download_;
    std::mutex *p_data_upload_mutex_;
    std::mutex *p_data_download_mutex_;

    shawn::STime udp_recv_times_;
    shawn::handle udp_recv_handle_;
    shawn::handle udp_recv_correct_handle_;

    shawn::STime pack_recv_times_;
    shawn::handle pack1_recv_handle_;
    shawn::handle pack2_recv_handle_;


    void init(ros::NodeHandle node_handle, DataDownload *p_data_download, DataUpload *p_data_upload, std::mutex *p_data_download_mutex, std::mutex *p_data_upload_mutex, shawn::SLog *p_log);
    void receiveInit(uint16_t p_server_port);
    void sendInit(std::string p_remote_ip, uint16_t p_remote_port);
    void dataReceive();
    void dataSend();
    bool time_check();
    void setWorkMode();
    uint8_t getWorkMode();

    RemoteControl();

    void setHandles();


    RemoteReceive remoteReceive_;
    RemoteSend remoteSend_;
};

}

#endif //ECU_COMMUNICATION_REMOTECONTROL_HPP