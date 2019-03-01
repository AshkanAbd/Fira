#ifndef MAPPING_DATA_MAPPING_H
#define MAPPING_DATA_MAPPING_H

#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <nav_msgs/OccupancyGrid.h>
#include <gazebo_msgs/ModelStates.h>
<<<<<<< HEAD
#include <thread>
=======
#include <tf/transform_broadcaster.h>
>>>>>>> f69d126eaa879bc74065d3c36d83c83ceb618e43

namespace data_mapping {
    class DataMapping {
    protected:
        ros::NodeHandlePtr nh;
        ros::Publisher *map_publisher;
        ros::Rate *rate;
        ros::Subscriber *state_sub;
        nav_msgs::OccupancyGridPtr publish_obj;
<<<<<<< HEAD
        char *main_map;
=======
        char *arr;
        uint map_size, map_height, map_width;
        std::string map_frame, odom_frame;
>>>>>>> f69d126eaa879bc74065d3c36d83c83ceb618e43

    public:

<<<<<<< HEAD
        void get_state(const gazebo_msgs::ModelStatesConstPtr &state_msg);

        void publish();
=======
        DataMapping(const std::string &map_frame, const std::string &odom_frame, const std::string &publish_topic,
                    const std::string &state_topic, int hz, const ros::NodeHandlePtr &nh, uint map_height,
                    uint map_width, float resolution, float initial_x, float initial_y);

        virtual void get_state(const gazebo_msgs::ModelStatesConstPtr &state_msg);

        virtual void initialize_map(const std::string &map_frame, uint map_height, uint map_width, float resolution,
                                    float initial_x, float initial_y);

        virtual void spin();

        virtual void tf_thread();

        virtual void publish_thread();

>>>>>>> f69d126eaa879bc74065d3c36d83c83ceb618e43
    };

    ulong convert(const double &robot_y, const double &robot_x, const nav_msgs::OccupancyGridPtr &map_obj);

    void convert(const double &robot_y, const double &robot_x, ulong *dst, const nav_msgs::OccupancyGridPtr &map_obj);
};


#endif