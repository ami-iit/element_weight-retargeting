find_package(WearableActuators REQUIRED)

yarp_add_idl(WEIGHT_RETARGETING_SERVICE thrift/WeightRetargetingService.thrift)

file(GLOB_RECURSE WEIGHT_RETARGETING_SRC src/*.cpp)

# Add weight retargeting module
add_executable(WeightRetargetingModule ${WEIGHT_RETARGETING_SRC} ${WEIGHT_RETARGETING_SERVICE})
target_include_directories(WeightRetargetingModule PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include # specific includes
        ${CMAKE_SOURCE_DIR}/src/include) # project level includes

target_link_libraries(WeightRetargetingModule PRIVATE
        WearableActuators::WearableActuators
        YARP::YARP_OS
        YARP::YARP_init
        YARP::YARP_dev)
