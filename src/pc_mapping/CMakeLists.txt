cmake_minimum_required(VERSION 2.8.3)
project(pc_mapping)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_core
        nav_msgs
        roscpp
        rospy
        std_msgs
        depth_image_proc
        sensor_msgs
        std_msgs
        tf
        tf2
        tf2_ros
        tf2_msgs
        tf2_sensor_msgs
        message_generation
        )


catkin_package(
        INCLUDE_DIRS
        include

        LIBRARIES
        pc_mapping

        CATKIN_DEPENDS
        std_msgs
        geometry_msgs
        nav_core
        nav_msgs
        roscpp
        rospy
        depth_image_proc
        sensor_msgs
        std_msgs
        tf
        tf2
        tf2_ros
        tf2_msgs
        tf2_sensor_msgs
        message_generation

        DEPENDS
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(pc_slam include/pc_mapping/pc_mapping.h src/pc_mapping.cpp) #for run
add_library(pc_mapping include/pc_mapping/pc_mapping.h src/pc_mapping.cpp) #for add as library


target_link_libraries(pc_slam ${catkin_LIBRARIES}) #for run
target_link_libraries(pc_mapping ${catkin_LIBRARIES}) #for add as library


install(TARGETS pc_mapping
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        )

install(DIRECTORY include/pc_mapping/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )
