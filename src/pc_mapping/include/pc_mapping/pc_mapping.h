#ifndef AURA_PC_MAPPING_H
#define AURA_PC_MAPPING_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <cmath>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Eigen>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Empty.h>

namespace pc_mapping {
    class PCMapping {
    protected:
        ulong map_size;
        ulong map_height;
        ulong map_width;
        ros::NodeHandlePtr nh;
        ros::Rate *rate;
        ros::Publisher *map_publisher;
        ros::Subscriber *pc_subscriber, *clear_sub;
        tf::TransformListener *listener;
        tf::StampedTransform *stamped_transform;
        char *arr_obj;
        nav_msgs::OccupancyGridPtr map_obj;
        unsigned int publish_counter;
        std::thread *publish_thread, *tf_thread;
        double min_tolerance, max_tolerance;
        std::string map_frame, odom_frame;

    public:
        PCMapping(const std::string &map_frame, const std::string &odom_frame, const std::string &map_topic,
                  const std::string &pc_topic, int hz, const ros::NodeHandlePtr &nh1, uint map_height, uint map_width,
                  float resolution, float initial_x, float initial_y, double min_tolerance, double max_tolerance);

        PCMapping();

        virtual void
        initialize(const std::string &map_frame, const std::string &odom_frame, const std::string &map_topic,
                   const std::string &pc_topic, int hz, const ros::NodeHandlePtr &nh1, uint map_height,
                   uint map_width, float resolution, float initial_x, float initial_y, double min_tolerance,
                   double max_tolerance);

        virtual void get_point_cloud(const sensor_msgs::PointCloud2ConstPtr &msg);

        virtual void publish_callable();

        virtual void setup_tf();

        virtual void spin();

        virtual void initialize_map(uint height, uint width, float resolution, float initial_x, float initial_y);

        virtual void empty_sub(const std_msgs::EmptyConstPtr e);
    };

    void apply_on_map(const char *tmp_map, ulong map_size, char *arr_obj);

    void add_map(ulong pose, char cond, char *arr_obj);

    inline bool validateFloats(float val) {
        return !(std::isnan(val) || std::isinf(val));
    }

    inline int32_t findChannelIndex(const sensor_msgs::PointCloud2ConstPtr &cloud, const std::string &channel) {
        for (size_t i = 0; i < cloud->fields.size(); ++i) {
            if (cloud->fields[i].name == channel) {
                return i;
            }
        }
        return -1;
    }

    inline int32_t findChannelIndex(const sensor_msgs::PointCloud2 &cloud, const std::string &channel) {
        for (size_t i = 0; i < cloud.fields.size(); ++i) {
            if (cloud.fields[i].name == channel) {
                return i;
            }
        }
        return -1;
    }


    ulong convert(const double &robot_y, const double &robot_x, const nav_msgs::OccupancyGridPtr &map_obj);

    void convert(const double &robot_y, const double &robot_x, ulong *dst, const nav_msgs::OccupancyGridPtr &map_obj);

};

#endif
