#include <ros/ros.h>
#include "ros/console.h"
#include "UDPServer.hpp"
#include "UDPClient.hpp"
#include <iostream>
#include <DataDownload.hpp>
#include <DataUpload.hpp>
#include "DataDownload.hpp"
#include "DataUpload.hpp"
#include "bit_operation.hpp"
#include <glog/logging.h>
#include "stdlib.h"
#include "stdio.h"
#include <bitset>
#include <CommunicationProcess.hpp>
#include <shawn_type.hpp>
#include "shawn_type.hpp"

char * funtst() {
    in_addr_t tmp1 = inet_addr("192.172.1.1");
    in_addr tmp2;
    tmp2.s_addr = tmp1;
    char * tmp3 = inet_ntoa(tmp2);
    std::cout<< tmp3;
    return tmp3;
}

int main(int argc, char** argv) {
//    ros::init(argc, argv, "ecu_communication");
//    ecu_communication::CommunicationProcess communicationProcess(ros::NodeHandle(), ros::NodeHandle("~"));
//    ros::spin();
    shawn::bit_16_type bit_16;
    bit_16.data[0] = 1;
    bit_16.data[1] = 0;
    shawn::example1.cell.data1 = 0;
    shawn::example1.cell.data2 = 1;
    shawn::example2.result = 0b10000100;
    shawn::example3.cell.data = 0b1111111100000000;
    std::cout<<(int)shawn::example3.result[0]<<std::endl;
    std::cout<<(int)shawn::example3.result[1]<<std::endl;

    std::cout<<shawn::example1.result<<std::endl;
    std::cout<<(int)shawn::example2.cell.data1<<std::endl;
    std::cout<<(int)shawn::example2.cell.data2<<std::endl;

    shawn::example4.result[0] = 0;
    shawn::example4.result[1] = 1;
    shawn::example4.result[2] = 2;
    shawn::example4.result[3] = 3;
    shawn::example4.result[4] = 4;
    std::cout<<(int)shawn::example4.cell.data1[0]<<std::endl;
    std::cout<<(int)shawn::example4.cell.data1[1]<<std::endl;
    std::cout<<(int)shawn::example4.cell.data2[0]<<std::endl;
    std::cout<<(int)shawn::example4.cell.data2[1]<<std::endl;
    std::cout<<(int)shawn::example4.cell.data2[2]<<std::endl;


    union data_upload_pack_one_type {
        struct {
            uint32_t data_ID;
            uint8_t valid_data_mark;
            uint8_t valid_data_length;
            uint16_t left_wheel_expect_speed;
            uint16_t mechanical_brake;
            uint16_t right_wheel_expect_speed;
            uint8_t vehicle_speed;
            uint8_t gear;
        } cell;
        uint8_t pack[14];
    }hah;
    hah.cell.left_wheel_expect_speed = 0xff00;
    std::cout<<(int)hah.pack[6]<<std::endl;
    std::cout<<(int)hah.pack[7]<<std::endl;

//    const char * tst = "haha";
//    google::InitGoogleLogging("hmp");
//    FLAGS_log_dir = getenv("HOME");
////    LOG(INFO)<<1;
////    LOG(INFO)<<1;
//    std::cout<<"hmp1"<<std::endl;
////    google::ShutdownGoogleLogging();
////    LOG(INFO)<<1;
////    LOG(INFO)<<1;
//    std::cout<<"hmp1"<<std::endl;
////    google::InitGoogleLogging("hmp");
//    LOG(INFO)<<1;
//    LOG(INFO)<<1;
//    std::cout<<"hmp1"<<std::endl;
//    google::ShutdownGoogleLogging();
//    std::cout<<"hmp1"<<std::endl;

}
