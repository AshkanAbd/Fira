#include "../include/pc_mapping/pc_mapping.h"

using namespace pc_mapping;

PCMapping::PCMapping(const std::string &map_frame, const std::string &odom_frame, const std::string &map_topic,
                     const std::string &pc_topic, int hz, const ros::NodeHandlePtr &nh1, uint map_height,
                     uint map_width, float resolution, float initial_x, float initial_y, double min_tolerance,
                     double max_tolerance) {
    PCMapping::initialize(map_frame, odom_frame, map_topic, pc_topic, hz, nh1, map_height, map_width, resolution,
                          initial_x, initial_y, min_tolerance, max_tolerance);
}

void PCMapping::initialize(const std::string &map_frame, const std::string &odom_frame, const std::string &map_topic,
                           const std::string &pc_topic, int hz, const ros::NodeHandlePtr &nh1, uint map_height,
                           uint map_width, float resolution, float initial_x, float initial_y, double min_tolerance,
                           double max_tolerance) {
    PCMapping::map_frame = map_frame;
    PCMapping::odom_frame = odom_frame;
    PCMapping::min_tolerance = min_tolerance;
    PCMapping::max_tolerance = max_tolerance;
    PCMapping::map_height = map_height;
    PCMapping::map_width = map_width;
    publish_counter = 0;
    nh = nh1;
    listener = new tf::TransformListener;
    stamped_transform = new tf::StampedTransform;
    map_publisher = new ros::Publisher(nh->advertise<nav_msgs::OccupancyGrid>(map_topic, 10000));
    rate = new ros::Rate(hz);
    initialize_map(map_height, map_width, resolution, initial_x, initial_y);
    publish_thread = new std::thread(&PCMapping::publish_callable, this);
    tf_thread = new std::thread(&PCMapping::setup_tf, this);
    pc_subscriber = new ros::Subscriber(nh->subscribe(pc_topic, 10000, &PCMapping::get_point_cloud, this));
    clear_sub = new ros::Subscriber(nh->subscribe("clear_map", 1000, &PCMapping::empty_sub, this));
}

void PCMapping::initialize_map(uint height, uint width, float resolution, float initial_x, float initial_y) {
    map_size = height * width;
    arr_obj = (char *) malloc(map_size * sizeof(char));
    memset(arr_obj, -1, map_size * sizeof(char));
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
}

void PCMapping::empty_sub(const std_msgs::EmptyConstPtr e) {
    memset(PCMapping::arr_obj, 0, PCMapping::map_size * sizeof(char));
}


void PCMapping::publish_callable() {
    while (ros::ok()) {
        map_obj->header.seq = publish_counter++;
        map_obj->header.stamp = ros::Time::now();
        std::copy(arr_obj, arr_obj + map_size, &(map_obj->data[0]));
        map_publisher->publish(*map_obj);
        rate->sleep();
    }
}

void PCMapping::setup_tf() {
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    ros::Rate tf_rate(100);
    while (ros::ok()) {
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frame, odom_frame));
        tf_rate.sleep();
    }
}

void PCMapping::get_point_cloud(const sensor_msgs::PointCloud2ConstPtr &msg) {
    sensor_msgs::PointCloud2Ptr msg1(new sensor_msgs::PointCloud2);
    msg1->data.resize(msg->data.size());
    try {
        listener->lookupTransform(msg->header.frame_id, odom_frame, ros::Time(0), *stamped_transform);
        pcl_ros::transformPointCloud(odom_frame, *msg, *msg1, *listener);
    } catch (std::exception &e) {
        ROS_ERROR("%s", e.what());
        return;
    }
    int32_t xi = findChannelIndex(msg1, "x");
    int32_t yi = findChannelIndex(msg1, "y");
    int32_t zi = findChannelIndex(msg1, "z");
    if (xi == -1 || yi == -1 || zi == -1)
        return;
    const uint32_t xoff = msg1->fields[xi].offset;
    const uint32_t yoff = msg1->fields[yi].offset;
    const uint32_t zoff = msg1->fields[zi].offset;
    const uint32_t point_step = msg1->point_step;
    const uint8_t *ptr = &msg1->data.front(), *ptr_end = &msg1->data.back();
    char *tmp_map = (char *) malloc(map_size * sizeof(char));
    memset(tmp_map, -1, map_size * sizeof(char));
    for (; ptr < ptr_end; ptr += point_step) {
        float x = *reinterpret_cast<const float *>(ptr + xoff);
        float y = *reinterpret_cast<const float *>(ptr + yoff);
        float z = *reinterpret_cast<const float *>(ptr + zoff);
        if (validateFloats(x) && validateFloats(y) && validateFloats(z)) {
            ulong pose_map[2];
            pc_mapping::convert(y, x, pose_map, PCMapping::map_obj);
            if (*(pose_map + 0) < PCMapping::map_height && *(pose_map + 1) < PCMapping::map_width) {
                auto pose = convert(y, x, PCMapping::map_obj);
                if (pose < PCMapping::map_size) {
                    if (z < min_tolerance) {
                        if (*(tmp_map + pose) == -1) {
                            *(tmp_map + pose) = 0;
                        }
                    } else if (z >= min_tolerance && z < max_tolerance) {
                        if (*(tmp_map + pose) != 10) {
                            *(tmp_map + pose) = 100;
                        }
                    } else if (z > max_tolerance) {
                        *(tmp_map + pose) = 10;
                    }
                }
            }
        }
    }
    pc_mapping::apply_on_map(tmp_map, PCMapping::map_size, PCMapping::arr_obj);
}

void pc_mapping::apply_on_map(const char *tmp_map, ulong map_size, char *arr_obj) {
    ulong pose = 0;
    for (; pose < map_size; pose++) {
        auto tmp = *(tmp_map + pose);
        if (tmp == 0)
            pc_mapping::add_map(pose, 0, arr_obj);
        else if (tmp == 10)
            pc_mapping::add_map(pose, 0, arr_obj);
        else if (tmp == 100)
            pc_mapping::add_map(pose, 100, arr_obj);
    }
}

void pc_mapping::add_map(ulong pose, char cond, char *arr_obj) {
    *(arr_obj + pose) = cond;
}

ulong pc_mapping::convert(const double &robot_y, const double &robot_x, const nav_msgs::OccupancyGridPtr &map_obj) {
    auto map_y = static_cast<ulong>(round((robot_y - map_obj->info.origin.position.y) / map_obj->info.resolution));
    auto map_x = static_cast<ulong>(round((robot_x - map_obj->info.origin.position.x) / map_obj->info.resolution));
    return (map_y * map_obj->info.width) + map_x;
}

void pc_mapping::convert(const double &robot_y, const double &robot_x, ulong *dst,
                         const nav_msgs::OccupancyGridPtr &map_obj) {
    *(dst + 0) = static_cast<ulong>(round((robot_y - map_obj->info.origin.position.y) / map_obj->info.resolution));
    *(dst + 1) = static_cast<ulong>(round((robot_x - map_obj->info.origin.position.x) / map_obj->info.resolution));
}

void PCMapping::spin() {
    ros::spin();
}

PCMapping::PCMapping() = default;

int main(int argc, char **argv) {
    std::string map_topic = "/base_map";
    std::string map_frame = "map";
    std::string odom_frame = "odom";
    std::string pc_topic = "/camera/depth/points";
    int hz = 10;
    int map_height = 6;
    int map_width = 10;
    float resolution = 0.05;
    float initial_x = -2;
    float initial_y = -3;
    double min_tolerance = 0.2;
    double max_tolerance = 10;
    ros::init(argc, argv, "pc_mapping");
    ros::NodeHandlePtr nh(new ros::NodeHandle);
    if (nh->hasParam(ros::this_node::getName() + "/map_topic")) {
        nh->getParam(ros::this_node::getName() + "/map_topic", map_topic);
    }
    if (nh->hasParam(ros::this_node::getName() + "/map_frame")) {
        nh->getParam(ros::this_node::getName() + "/map_frame", map_frame);
    }
    if (nh->hasParam(ros::this_node::getName() + "/odom_frame")) {
        nh->getParam(ros::this_node::getName() + "/odom_frame", odom_frame);
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
    PCMapping mapping(map_frame, odom_frame, map_topic, pc_topic, hz, nh, static_cast<uint>(map_height / resolution),
                      static_cast<uint>(map_width / resolution), resolution, initial_x, initial_y, min_tolerance,
                      max_tolerance);
    mapping.spin();
}
