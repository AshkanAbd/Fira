#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <gazebo_msgs/ModelStates.h>
#include "../../data_mapping/include/data_mapping/data_mapping.h"

using namespace data_mapping;

DataMapping::DataMapping(const ros::NodeHandlePtr &nh, const int &hz, const std::string &publish_topic,
                         const std::string &state_topic) {
    if (nh == nullptr) {
        DataMapping::nh = ros::NodeHandlePtr(new ros::NodeHandle);
    } else {
        DataMapping::nh = nh;
    }
    rate = new ros::Rate(hz);
    map_publisher = new ros::Publisher(DataMapping::nh->advertise<nav_msgs::OccupancyGrid>(publish_topic, 1000));
    state_sub = new ros::Subscriber(DataMapping::nh->subscribe(state_topic, 1000, &DataMapping::get_state, this));
    DataMapping::publish_obj = nav_msgs::OccupancyGridPtr(new nav_msgs::OccupancyGrid);
}

void DataMapping::get_state(const gazebo_msgs::ModelStatesConstPtr &state_msg) {

}

int main() {

}
