#ifndef ECU_COMMUNICATION_DEFINES_HPP
#define ECU_COMMUNICATION_DEFINES_HPP

#define PUBLISH_PERIOD 0.008
#define CHECK_PERIOD 0.25
#define UDP_SEND_PERIOD 0.025
#define REMOTE_SEND_PERIOD 0.02

#define LOG_INFO LOG(INFO)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_WARN LOG(WARNING)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_ERROR LOG(ERROR)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define LOG_FATAL LOG(FATAL)<<std::setiosflags(std::ios::fixed)<<ros::Time::now().toSec()<<" "
#define FIXED std::setiosflags(std::ios::fixed)<<

#define DEFAULT_WORK_MODE 0


#define REDUCTION_RATIO 17
#define TIRE_RADIUS 0.348 //m

#define RPM_TO_SPEED 0.002143675   //TIRE_RADIUS * M_PI / REDUCTION_RATIO / 30.0
#define ONE_PULSE_DISTANCE 0.06431025   //TIRE_RADIUS * M_PI / 17

#endif //ECU_COMMUNICATION_DEFINES_HPP