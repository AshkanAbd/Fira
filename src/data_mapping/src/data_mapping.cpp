#include "../../data_mapping/include/data_mapping/data_mapping.h"

using namespace data_mapping;

DataMapping::DataMapping(const std::string &map_frame, const std::string &odom_frame, const std::string &publish_topic,
                         const std::string &state_topic, int hz, const ros::NodeHandlePtr &nh, uint map_height,
                         uint map_width, float resolution, float initial_x, float initial_y) {
    if (nh == nullptr) {
        DataMapping::nh = ros::NodeHandlePtr(new ros::NodeHandle);
    } else {
        DataMapping::nh = nh;
    }
    DataMapping::rate = new ros::Rate(hz);
    DataMapping::initialize_map(map_frame, map_height, map_width, resolution, initial_x, initial_y);
    map_publisher = new ros::Publisher(DataMapping::nh->advertise<nav_msgs::OccupancyGrid>(publish_topic, 1000));
    new std::thread(&DataMapping::publish_thread, this);
    state_sub = new ros::Subscriber(DataMapping::nh->subscribe(state_topic, 1000, &DataMapping::get_state, this));
}

void DataMapping::get_state(const gazebo_msgs::ModelStatesConstPtr &state_msg) {
    for (int i = 0; i < state_msg->name.size(); i++) {
        if ((state_msg->name.data() + i)->find("newbox") == std::string::npos) continue;
        auto box_pos = (state_msg->pose.data() + i)->position;
        auto poses = (ulong *) malloc(2 * sizeof(ulong));
        data_mapping::convert(box_pos.y, box_pos.x, poses, DataMapping::publish_obj);
        for (int j = -3; j <= 3; j++) {
            for (int k = -3; k <= 3; k++) {
                auto pose = ((*(poses + 0) + j) * DataMapping::publish_obj->info.width) + (*(poses + 1) + k);
                *(arr + pose) = 100;
            }
        }
    }
}

void DataMapping::initialize_map(const std::string &map_frame, uint map_height, uint map_width, float resolution,
                                 float initial_x, float initial_y) {
    DataMapping::map_size = map_height * map_width;
    DataMapping::arr = (char *) malloc(DataMapping::map_size * sizeof(char));
    memset(arr, 0, map_size * sizeof(char));
    DataMapping::publish_obj = nav_msgs::OccupancyGridPtr(new nav_msgs::OccupancyGrid);
    DataMapping::publish_obj->info.width = map_width;
    DataMapping::publish_obj->info.height = map_height;
    DataMapping::publish_obj->info.resolution = resolution;
    DataMapping::publish_obj->info.origin.position.x = initial_x;
    DataMapping::publish_obj->info.origin.position.y = initial_y;
    DataMapping::publish_obj->header.frame_id = map_frame;
    DataMapping::publish_obj->data.resize(map_size);
}

void DataMapping::publish_thread() {
    uint counter = 0;
    while (ros::ok()) {
        std::copy(DataMapping::arr, DataMapping::arr + DataMapping::map_size, &(DataMapping::publish_obj->data[0]));
        DataMapping::publish_obj->header.seq = counter++;
        DataMapping::publish_obj->header.stamp = ros::Time::now();
        DataMapping::map_publisher->publish(*DataMapping::publish_obj);
        rate->sleep();
    }
}

void DataMapping::spin() {
    ros::spin();
}

ulong data_mapping::convert(const double &robot_y, const double &robot_x, const nav_msgs::OccupancyGridPtr &map_obj) {
    auto map_y = static_cast<ulong>(round((robot_y - map_obj->info.origin.position.y) / map_obj->info.resolution));
    auto map_x = static_cast<ulong>(round((robot_x - map_obj->info.origin.position.x) / map_obj->info.resolution));
    return (map_y * map_obj->info.width) + map_x;
}

void data_mapping::convert(const double &robot_y, const double &robot_x, ulong *dst,
                           const nav_msgs::OccupancyGridPtr &map_obj) {
    *(dst + 0) = static_cast<ulong>(round((robot_y - map_obj->info.origin.position.y) / map_obj->info.resolution));
    *(dst + 1) = static_cast<ulong>(round((robot_x - map_obj->info.origin.position.x) / map_obj->info.resolution));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Data_test");
    std::string map_frame = "map";
    std::string odom_frame = "odom";
    std::string map_topic = "map";
    std::string model_topic = "/gazebo/model_states";
    int hz = 10;
    ros::NodeHandlePtr nh(new ros::NodeHandle);
    int map_height = 4;
    int map_width = 6;
    float resolution = 0.1f;
    float initial_x = -0.8f;
    float initial_y = -2.0f;
    if (nh->hasParam(ros::this_node::getName() + "/map_topic")) {
        nh->getParam(ros::this_node::getName() + "/map_topic", map_topic);
    }
    if (nh->hasParam(ros::this_node::getName() + "/map_frame")) {
        nh->getParam(ros::this_node::getName() + "/map_frame", map_frame);
    }
    if (nh->hasParam(ros::this_node::getName() + "/odom_frame")) {
        nh->getParam(ros::this_node::getName() + "/odom_frame", odom_frame);
    }
    if (nh->hasParam(ros::this_node::getName() + "/model_topic")) {
        nh->getParam(ros::this_node::getName() + "/model_topic", model_topic);
    }
    if (nh->hasParam(ros::this_node::getName() + "/hz")) {
        nh->getParam(ros::this_node::getName() + "/hz", hz);
    }
    if (nh->hasParam(ros::this_node::getName() + "/height")) {
        nh->getParam(ros::this_node::getName() + "/height", map_height);
    }
    if (nh->hasParam(ros::this_node::getName() + "/width")) {
        nh->getParam(ros::this_node::getName() + "/width", map_width);
    }
    if (nh->hasParam(ros::this_node::getName() + "/resolution")) {
        nh->getParam(ros::this_node::getName() + "/resolution", resolution);
    }
    if (nh->hasParam(ros::this_node::getName() + "/initial_x")) {
        nh->getParam(ros::this_node::getName() + "/initial_x", initial_x);
    }
    if (nh->hasParam(ros::this_node::getName() + "/initial_y")) {
        nh->getParam(ros::this_node::getName() + "/initial_y", initial_y);
    }
    DataMapping data_mapping(map_frame, odom_frame, map_topic, model_topic, hz, nh,
                             static_cast<uint>(map_height / resolution), static_cast<uint>(map_width / resolution),
                             resolution, initial_x, initial_y);
    data_mapping.spin();
}
