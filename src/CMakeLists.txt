cmake_minimum_required(VERSION 3.0.0)

find_package(YARP 3.2 REQUIRED)
find_package(WearableActuators REQUIRED)


# Add weight retargeting application
add_executable(WeightRetargetingModule WeightRetargeting.cpp)
target_link_libraries(WeightRetargetingModule PRIVATE
        WearableActuators::WearableActuators
        YARP::YARP_OS
        YARP::YARP_init
        YARP::YARP_dev)

install(TARGETS WeightRetargetingModule
        DESTINATION bin)
