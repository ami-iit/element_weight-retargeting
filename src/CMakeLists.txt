cmake_minimum_required(VERSION 3.0.0)

find_package(YARP 3.2 REQUIRED)
find_package(WearableActuators REQUIRED)

yarp_add_idl(WEIGHT_RETARGETING_SERVICE thrift/WeightRetargetingService.thrift)

# Add weight retargeting application
add_executable(WeightRetargetingModule WeightRetargeting.cpp ${WEIGHT_RETARGETING_SERVICE})
target_link_libraries(WeightRetargetingModule PRIVATE
        WearableActuators::WearableActuators
        YARP::YARP_OS
        YARP::YARP_init
        YARP::YARP_dev)

install(TARGETS WeightRetargetingModule
        DESTINATION bin)
