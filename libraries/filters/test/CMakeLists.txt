cmake_minimum_required(VERSION 3.4)

# Create a project
set(TARGET_NAME test_filters)
project(${TARGET_NAME})

find_package(matioCpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(BipedalLocomotionFramework REQUIRED COMPONENTS matioCppConversions)

# Add source files
add_executable("${TARGET_NAME}"
    test_second_order_low_pass.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>")


include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC  ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${TARGET_NAME} PRIVATE
                      matioCpp::matioCpp
                      BipedalLocomotion::matioCppConversions
                      filters)

# Install the modules
install(TARGETS "${TARGET_NAME}"
        DESTINATION bin)

