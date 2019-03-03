#ifndef MAPPING_RGB_MAPPING_H
#define MAPPING_RGB_MAPPING_H

#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.h>
#include "data_set.h"

namespace depth_detector {
    class RGBToPointCloud {
    protected:
        ros::NodeHandlePtr nh;
        ros::Subscriber *rgb_sub;
        ros::Publisher *pc_publisher, *depth_publisher;
        sensor_msgs::PointCloud2Ptr point_obj;
        cv::Scalar *lower, *upper;
        sensor_msgs::CameraInfoConstPtr camera_info;
        image_geometry::PinholeCameraModel *camera_model;
        int y_data_set_size, x_data_set_size, depth_data_set_size;
        RGBDataSet *y_data_set;
        RGBDataSet *x_data_set;
        DepthDataSet *depth_data_set;
        std::string data_set_dir, pc_frame;
        cv::UMat *sample_depth;

        inline double angle(const cv::Point &p1, const cv::Point &p2, const cv::Point &p0) {
            double dx1 = p1.x - p0.x;
            double dy1 = p1.y - p0.y;
            double dx2 = p2.x - p0.x;
            double dy2 = p2.y - p0.y;
            return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
        }

        virtual void read_data_set();

        virtual inline void read_y_data_set();

        virtual inline void read_x_data_set();

        virtual inline void read_depth_data();

        virtual inline void read_camera_info(sensor_msgs::CameraInfo &cam_info);

        virtual void create_camera_model(const std::string &camera_info_topic);

    public:

        RGBToPointCloud(const ros::NodeHandlePtr &nh, const std::string &rgb_topic,
                        const std::string &camera_info_topic, const std::string &depth_topic,
                        const std::string &pc_topic, const std::string &data_set_dir, int y_data_set_size,
                        int x_data_set_size, int depth_data_set_size, const std::string &color_lower,
                        const std::string &color_upper, const std::string &pc_frame);

        RGBToPointCloud();

        virtual void initialize(const ros::NodeHandlePtr &nh, const std::string &rgb_topic,
                                const std::string &camera_info_topic, const std::string &depth_topic,
                                const std::string &pc_topic, const std::string &data_set_dir, int y_data_set_size,
                                int x_data_set_size, int depth_data_set_size, const std::string &color_lower,
                                const std::string &color_upper, const std::string &pc_frame);

        virtual void publish_pc(const sensor_msgs::PointCloud2ConstPtr &point_cloud2);

        virtual void get_image(const sensor_msgs::ImageConstPtr &img_msg);

        virtual void spin();
    };

    float dNaN = std::numeric_limits<float>::quiet_NaN();

    void read_color(const std::string &color, cv::Scalar &scalar);

    int closest_data(const depth_detector::DepthDataSet &src, depth_detector::DepthDataSet &dst,
                     depth_detector::DepthDataSet *depth_data_set, int data_set_size);

    int closest_data(const depth_detector::RGBDataSet &src, depth_detector::RGBDataSet &dst,
                     depth_detector::RGBDataSet *data_set, int data_set_size);

    void create_depth_image(const depth_detector::DepthDataSet &src, const cv::Rect &rect, cv::UMat &frame);

    void create_pc(const sensor_msgs::ImageConstPtr &msg, sensor_msgs::PointCloud2Ptr &point_obj,
                   const image_geometry::PinholeCameraModel *camera_model, float range_max = 0.0);
};

#endif
