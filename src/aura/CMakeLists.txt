cmake_minimum_required(VERSION 2.8.3)
project(aura)

set(CMAKE_CXX_STANDARD 11)


find_package(catkin REQUIRED COMPONENTS
        costmap_2d
        cv_bridge
        geometry_msgs
        image_transport
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
        pc_mapping
        )

find_package(OpenCV REQUIRED)

catkin_package(

)

include_directories(
        ${catkin_INCLUDE_DIRS}
)


add_executable(custom_tf src/custom_tf.cpp)
add_executable(aura_mapping src/custom_mapping.cpp include/aura/custom_mapping.h)

target_link_libraries(aura_mapping ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(custom_tf ${catkin_LIBRARIES} ${OpenCV_LIBS})

