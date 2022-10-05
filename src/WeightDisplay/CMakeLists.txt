# Add weight display module
add_executable(WeightDisplayModule src/WeightDisplayModule.cpp)
target_include_directories(WeightDisplayModule PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include # specific includes
        ${CMAKE_SOURCE_DIR}/src/include) # project level includes
target_link_libraries(WeightDisplayModule PRIVATE
        YARP::YARP_OS
        YARP::YARP_init
        YARP::YARP_sig
        YARP::YARP_dev
        filters)

# Install the modules
install(TARGETS WeightRetargetingModule WeightDisplayModule
        DESTINATION bin)
