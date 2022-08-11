find_package(YARP 3.2 REQUIRED)

# add the modules 
add_subdirectory(WeightRetargeting)
add_subdirectory(WeightDisplay)

# install config files
file (GLOB_RECURSE ${CMAKE_PROJECT_NAME}_CONF_FILES *.ini)
install(FILES ${${CMAKE_PROJECT_NAME}_CONF_FILES}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/yarp/contexts/WeightRetargeting)
