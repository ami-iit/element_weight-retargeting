find_package(YARP 3.2 REQUIRED)
find_package(WearableActuators REQUIRED)

yarp_add_idl(WEIGHT_RETARGETING_SERVICE thrift/WeightRetargetingService.thrift)

# Add weight retargeting module
add_executable(WeightRetargetingModule WeightRetargeting.cpp ${WEIGHT_RETARGETING_SERVICE})
target_include_directories(WeightRetargetingModule PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(WeightRetargetingModule PRIVATE
        WearableActuators::WearableActuators
        YARP::YARP_OS
        YARP::YARP_init
        YARP::YARP_dev)

# Add weight display module
add_executable(WeightDisplayModule WeightDisplayModule.cpp)
target_include_directories(WeightDisplayModule PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(WeightDisplayModule PRIVATE
        YARP::YARP_OS
        YARP::YARP_init
        YARP::YARP_sig
        YARP::YARP_dev)

# Install the modules
install(TARGETS WeightRetargetingModule WeightDisplayModule
        DESTINATION bin)
