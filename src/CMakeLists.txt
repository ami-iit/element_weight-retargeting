cmake_minimum_required(VERSION 3.0.0)

find_package(YARP 3.2 REQUIRED)
find_package(IWear REQUIRED)


# Add weight retargeting application
add_executable(WeightRetargetingModule WeightRetargeting.cpp)
target_link_libraries(WeightRetargetingModule PRIVATE
        IWear::IWear
        YARP::YARP_OS
        YARP::YARP_init)

install(TARGETS WeightRetargetingModule
        DESTINATION bin)
