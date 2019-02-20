cmake_minimum_required(VERSION 2.8.3)
project(depth_detector)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        depth_image_proc
        geometry_msgs
        image_transport
        nav_msgs
        cv_bridge
        image_geometry
        )

find_package(OpenCV REQUIRED)


catkin_package(
        INCLUDE_DIRS
        include

        LIBRARIES
        rgb_to_point_cloud

        CATKIN_DEPENDS
        roscpp
        sensor_msgs
        depth_image_proc
        geometry_msgs
        image_transport
        nav_msgs
        cv_bridge
        image_geometry

        DEPENDS
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(test_code src/test.cpp) #for test
add_executable(rgb_to_point_cloud1 src/rgb_to_point_cloud.cpp include/depth_detector/rgb_to_point_cloud.h include/depth_detector/data_set.h src/data_set.cpp) #for run
add_library(rgb_to_point_cloud src/rgb_to_point_cloud.cpp include/depth_detector/rgb_to_point_cloud.h include/depth_detector/data_set.h src/data_set.cpp) #for add as library


target_link_libraries(test_code ${catkin_LIBRARIES} ${OpenCV_LIBS}) #for test
target_link_libraries(rgb_to_point_cloud1 ${catkin_LIBRARIES} ${OpenCV_LIBS} ) #for run
target_link_libraries(rgb_to_point_cloud ${catkin_LIBRARIES} ${OpenCV_LIBS} ) #for add as library

install(TARGETS rgb_to_point_cloud
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        )

install(DIRECTORY include/depth_detector/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )