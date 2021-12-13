cmake_minimum_required(VERSION 3.0.0)

file (GLOB ${CMAKE_PROJECT_NAME}_CONF_FILES *.xml)

install(FILES ${${CMAKE_PROJECT_NAME}_CONF_FILES}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/WeightRetargeting)

