#ifndef ECU_COMMUNICATION_REMOTECONTROL_HPP
#define ECU_COMMUNICATION_REMOTECONTROL_HPP

#include <mutex>
#include <ros/ros.h>

#include "STime.hpp"
#include "SProportion.hpp"
#include "SLog.hpp"

#include "DEFINEs.hpp"
#include "UDPCommunication.hpp"
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

    //// UDP communication data and variables
    UDPCommunication udp_;
    RemoteReceive remoteReceive_;
    RemoteSend remoteSend_;

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


    RemoteControl();
    void init(DataDownload *p_data_download, DataUpload *p_data_upload,
              std::mutex *p_data_download_mutex, std::mutex *p_data_upload_mutex,
              shawn::SLog *p_log, uint8_t *p_work_mode);
    void setHandles();
    void dataReceive();
    void dataSend();
    void setWorkMode();
    uint8_t getWorkMode();
    bool time_check();
};

}

#endif //ECU_COMMUNICATION_REMOTECONTROL_HPP