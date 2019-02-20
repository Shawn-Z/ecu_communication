#ifndef ECU_COMMUNICATION_DEFINES_HPP
#define ECU_COMMUNICATION_DEFINES_HPP

#define PUBLISH_PERIOD 0.008
#define CHECK_PERIOD 0.25
#define UDP_SEND_PERIOD 0.025
#define REMOTE_SEND_PERIOD 0.05

#define LOG_INFO LOG(INFO)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_WARN LOG(WARNING)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_ERROR LOG(ERROR)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_FATAL LOG(FATAL)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define FIXED std::setiosflags(std::ios::fixed)<<

#define DEFAULT_WORK_MODE 1

#endif //ECU_COMMUNICATION_DEFINES_HPP