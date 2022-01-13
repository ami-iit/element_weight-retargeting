file (GLOB ${CMAKE_PROJECT_NAME}_CONF_FILES *.ini)

install(FILES ${${CMAKE_PROJECT_NAME}_CONF_FILES}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/yarp/contexts/WeightRetargeting)
