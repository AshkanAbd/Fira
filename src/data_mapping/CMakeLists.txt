cmake_minimum_required(VERSION 2.8.3)
project(data_mapping)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        nav_msgs
        tf
        gazebo_msgs
        )


catkin_package(
        INCLUDE_DIRS
        include

        LIBRARIES
        data_slam

        CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        nav_msgs
        tf
        gazebo_msgs

        DEPENDS
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(data_mapping include/data_mapping/data_mapping.h src/data_mapping.cpp)
add_library(data_slam include/data_mapping/data_mapping.h src/data_mapping.cpp)

target_link_libraries(data_mapping ${catkin_LIBRARIES})
target_link_libraries(data_slam ${catkin_LIBRARIES})

install(TARGETS data_slam
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        )

install(DIRECTORY include/data_mapping/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

