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
    std::thread thread(&DataMapping::publish, this);
    DataMapping::main_map = (char*)malloc(h*w*res* sizeof(char));
    memset(DataMapping::main_map,0,w*h*res* sizeof(char));
}

void DataMapping::get_state(const gazebo_msgs::ModelStatesConstPtr &state_msg) {

}

void DataMapping::publish() {

}

int main() {
}
