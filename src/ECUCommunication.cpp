#include "CommunicationProcess.hpp"
#include "SFile.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "ecu_communication");
    ecu_communication::CommunicationProcess communicationProcess(ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
}