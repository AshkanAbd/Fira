#include "../include/aura/custom_mapping.h"


AuraMapping::AuraMapping() = default;


void AuraMapping::initialize_map(uint height, uint width, float resolution, float initial_x, float initial_y) {
    map_size = height * width;
    arr_obj = (char *) malloc(map_size * sizeof(char));
    memset(arr_obj, AuraMapping::initial_value, map_size * sizeof(char));
    map_obj = nav_msgs::OccupancyGridPtr(new nav_msgs::OccupancyGrid);
    map_obj->header.frame_id = map_frame;
    map_obj->info.height = height;
    map_obj->info.width = width;
    map_obj->info.resolution = resolution;
    map_obj->info.origin.orientation.x = 0;
    map_obj->info.origin.orientation.y = 0;
    map_obj->info.origin.orientation.z = 0;
    map_obj->info.origin.position.x = initial_x;
    map_obj->info.origin.position.y = initial_y;
    map_obj->info.origin.position.z = 0;
    map_obj->data.resize(map_size);
    remove_invalid_area();
}

void AuraMapping::remove_invalid_area() {
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 15; ++j) {
            *(arr_obj + (i * AuraMapping::map_width) + j) = 100;
        }
    }
    for (ulong i = AuraMapping::map_height - 20; i < AuraMapping::map_height; ++i) {
        for (int j = 0; j < 15; ++j) {
            *(arr_obj + (i * AuraMapping::map_width) + j) = 100;
        }
    }
    for (int i = 0; i < 20; ++i) {
        for (ulong j = AuraMapping::map_width - 15; j < AuraMapping::map_width; ++j) {
            *(arr_obj + (i * AuraMapping::map_width) + j) = 100;
        }
    }
    for (ulong i = AuraMapping::map_height - 20; i < AuraMapping::map_height; ++i) {
        for (ulong j = AuraMapping::map_width - 15; j < AuraMapping::map_width; ++j) {
            *(arr_obj + (i * AuraMapping::map_width) + j) = 100;
        }
    }
}

int main(int argc, char **argv) {
    std::string node_name = "aura_slam";
    std::string map_topic = "/aura/base_map";
    std::string pc_topic = "/camera/depth/points";
    std::string map_frame = /*"/aura/map"*/ "map";
    std::string odom_frame = /*"/aura/odom"*/"/aura/map";
    int hz = 10;
    int map_height = 3;
    int map_width = 7;
    float resolution = 0.05;
    float initial_x = -1;
    auto initial_y = static_cast<float>(-1.5);
    double min_tolerance = 0.05;
    double max_tolerance = 10;
    int initial_value = 0;
    ros::init(argc, argv, node_name);
    ros::NodeHandlePtr nh(new ros::NodeHandle);
    if (nh->hasParam(ros::this_node::getName() + "/map_frame")) {
        nh->getParam(ros::this_node::getName() + "/map_frame", map_frame);
    }
    if (nh->hasParam(ros::this_node::getName() + "/odom_frame")) {
        nh->getParam(ros::this_node::getName() + "/odom_frame", odom_frame);
    }
    if (nh->hasParam(ros::this_node::getName() + "/map_topic")) {
        nh->getParam(ros::this_node::getName() + "/map_topic", map_topic);
    }
    if (nh->hasParam(ros::this_node::getName() + "/pc_topic")) {
        nh->getParam(ros::this_node::getName() + "/pc_topic", pc_topic);
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
    if (nh->hasParam(ros::this_node::getName() + "/min_tolerance")) {
        nh->getParam(ros::this_node::getName() + "/min_tolerance", min_tolerance);
    }
    if (nh->hasParam(ros::this_node::getName() + "/max_tolerance")) {
        nh->getParam(ros::this_node::getName() + "/max_tolerance", max_tolerance);
    }
    if (nh->hasParam(ros::this_node::getName() + "/initial_value")) {
        nh->getParam(ros::this_node::getName() + "/initial_value", initial_value);
    }
    AuraMapping a;
    a.initial_value = initial_value;
    a.initialize(map_frame, odom_frame, map_topic, pc_topic, hz, nh, static_cast<uint>(map_height / resolution),
                 static_cast<uint>(map_width / resolution), resolution, initial_x, initial_y, min_tolerance,
                 max_tolerance);
    a.spin();
}
