cmake_minimum_required(VERSION 3.0.2)
project(rm_forecast)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS"-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED
        roscpp
        tf
        rm_common
        message_generation
        dynamic_reconfigure
        std_msgs
        geometry_msgs
        visualization_msgs
        image_transport
        cv_bridge
        image_geometry
        nodelet
        tf2
        tf2_geometry_msgs
        rm_msgs
        )

generate_dynamic_reconfigure_options(
        cfg/Forecast.cfg
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        rm_common
        roscpp
        tf
        message_generation
        dynamic_reconfigure
        std_msgs
        geometry_msgs
        visualization_msgs
        image_transport
        cv_bridge
        image_geometry
        nodelet
        tf2
        tf2_geometry_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

FILE(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

add_library(
        ${PROJECT_NAME}
        src/forecast_node.cpp
        src/kalman_filter.cpp
        src/tracker.cpp
        src/spin_observer.cpp
        include/kalman_filter.h
        include/forecast_node.h
        include/tracker.h
        include/spin_observer.h
)

target_link_libraries(
        ${PROJECT_NAME}
        ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation
install(
        FILES rm_forecast.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
