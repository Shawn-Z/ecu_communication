cmake_minimum_required(VERSION 2.8.3)
project(ecu_communication)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(catkin_deps
    roscpp
    three_one_msgs
    sensor_driver_msgs
)

find_package(catkin REQUIRED
    COMPONENTS
    rosparam_handler
    dynamic_reconfigure
    ${catkin_deps}
)

generate_ros_parameter_files(config/ecu_communication.params)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${catkin_deps}
    #  CATKIN_DEPENDS other_catkin_pkg
    #  DEPENDS system_lib
)

include_directories(
    include
    include/shawn
    test
    ${catkin_INCLUDE_DIRS}
)

add_executable(ecu_communication
    src/DataUpload.cpp
    src/DataDownload.cpp
    src/UDPCommunication.cpp
    src/CommunicationProcess.cpp
    src/ECUCommunication.cpp
    src/RemoteControl.cpp
    src/AutonomousControl.cpp
    src/RemoteReceive.cpp
    src/RemoteSend.cpp
    src/Transform6t.cpp)

add_dependencies(ecu_communication
    ${PROJECT_NAME}_gencfg
)

target_link_libraries(ecu_communication
    ${catkin_LIBRARIES}
    glog
)