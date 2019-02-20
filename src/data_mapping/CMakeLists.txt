cmake_minimum_required(VERSION 2.8.3)
project(data_mapping)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        nav_msgs
        gazebo_msgs
        )

add_executable(data_mapping include/data_mapping/data_mapping.h src/data_mapping.cpp)

target_link_libraries(data_mapping ${catkin_LIBRARIES})

catkin_package(
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

