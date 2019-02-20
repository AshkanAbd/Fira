#ifndef MAPPING_DATA_MAPPING_H
#define MAPPING_DATA_MAPPING_H

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <gazebo_msgs/ModelStates.h>

namespace data_mapping {
    class DataMapping {
    private:
        ros::NodeHandlePtr nh;
        ros::Publisher *map_publisher;
        ros::Rate *rate;
        ros::Subscriber *state_sub;
        nav_msgs::OccupancyGridPtr publish_obj;

    public:
        DataMapping(const ros::NodeHandlePtr &nh, const int &hz, const std::string &publish_topic,
                    const std::string &state_topic);

        void get_state(const gazebo_msgs::ModelStatesConstPtr &state_msg);
    };
};


#endif