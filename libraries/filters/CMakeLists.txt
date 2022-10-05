# Copyright (C) 2016 Istituto Italiano di Tecnologia  iCub Facility
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 3.5)

find_package(Eigen3 REQUIRED)
find_package(YARP 3.2 REQUIRED)

project(filters)

set(${PROJECT_NAME}_HDRS include/${PROJECT_NAME}/SecondOrderLowPass.h)

set(${PROJECT_NAME}_SRCS src/SecondOrderLowPass.cpp)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_HDRS} ${${PROJECT_NAME}_SRCS})

set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

target_include_directories(${PROJECT_NAME} PUBLIC
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>"
                           ${EIGEN3_INCLUDE_DIR})

set_property(TARGET ${PROJECT_NAME} PROPERTY PUBLIC_HEADER ${${PROJECT_NAME}_HELPERS_SRCS})

target_link_libraries(filters PRIVATE
                      YARP::YARP_sig)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

add_subdirectory(test)
