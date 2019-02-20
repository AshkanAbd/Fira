cmake_minimum_required(VERSION 2.8.3)
project(fake_odometry)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        nav_msgs
        tf
        )

catkin_package(
        INCLUDE_DIRS
        include

        LIBRARIES
        fake_odometry

        CATKIN_DEPENDS
        roscpp
        geometry_msgs
        nav_msgs
        tf

        DEPENDS
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(fake_odometry1 src/fake_odometry.cpp include/fake_odometry/fake_odometry.h) # for run
add_library(fake_odometry src/fake_odometry.cpp include/fake_odometry/fake_odometry.h) # for add as library


target_link_libraries(fake_odometry1 ${catkin_LIBRARIES}) #for run
target_link_libraries(fake_odometry ${catkin_LIBRARIES}) # for add as library

install(TARGETS fake_odometry
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        )

install(DIRECTORY include/fake_odometry/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

