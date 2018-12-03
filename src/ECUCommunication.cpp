#include "CommunicationProcess.hpp"

bool process(char *buffer, size_t buffer_size) {
    ROS_INFO_STREAM(sizeof(buffer));
    ROS_INFO_STREAM((int)buffer[0]);
    ROS_INFO_STREAM((int)buffer[1]);
    ROS_INFO_STREAM((int)buffer[2]);
    ROS_INFO_STREAM((int)buffer[3]);
    return true;
}

uint8_t * tst() {
    uint8_t data[3] = {1,2,3};
    return data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ecu_communication");
    ecu_communication::CommunicationProcess communicationProcess(ros::NodeHandle(), ros::NodeHandle("~"));
    uint8_t data[9] = {1,2,3};
    process((char *)data, 1);



    communicationProcess.udpReceive();



    ros::spin();
}