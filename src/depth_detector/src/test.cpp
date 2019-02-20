#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <vector>
#include <fstream>
#include <cmath>

void get_image(const sensor_msgs::ImageConstPtr &img_msg);


void get_odom(const nav_msgs::OdometryConstPtr &odom_msg);

double angle(const cv::Point &p1, const cv::Point &p2, const cv::Point &p0);

cv::Scalar lower(170, 75, 50);
cv::Scalar upper(180, 255, 255);
std::fstream *file;
nav_msgs::OdometryConstPtr odom;
double robot_angle = -100000;

int main(int argc, char **argv) {
    file = new std::fstream;
    file->open("/home/ashkan/fira_challenge/src/depth_detector/data/data_set_y111.aura", std::ios::out);
    if (file->fail()) {
        std::cout << strerror(errno) << std::endl;
        exit(0);
    }
    ros::init(argc, argv, "test1");
    ros::NodeHandlePtr nh(new ros::NodeHandle);
    ros::Subscriber rgb_sub = nh->subscribe("/camera/rgb/image_raw", 1000, get_image);
    ros::Subscriber odom_sub = nh->subscribe("/odom", 1000, get_odom);
    ros::spin();
}

double angle(const cv::Point &p1, const cv::Point &p2, const cv::Point &p0) {
    double dx1 = p1.x - p0.x;
    double dy1 = p1.y - p0.y;
    double dx2 = p2.x - p0.x;
    double dy2 = p2.y - p0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

void get_image(const sensor_msgs::ImageConstPtr &img_msg) {
    cv_bridge::CvImageConstPtr img_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    cv::UMat frame, frame_hsv, frame_blur, frame_py_down, frame_py_up, frame_thresh, frame_filter, frame_edge;
    img_ptr->image.copyTo(frame);
    cv::pyrDown(frame, frame_py_down);
    cv::pyrUp(frame_py_down, frame_py_up);
    cv::GaussianBlur(frame_py_up, frame_blur, cv::Size(5, 5), -1);
//    cv::imshow("a", frame_blur);
    cv::cvtColor(frame_blur, frame_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv, lower, upper, frame_thresh);
    cv::bitwise_and(frame_blur, frame_blur, frame_filter, frame_thresh);
    cv::Canny(frame_filter, frame_edge, 100, 200);
//    cv::imshow("b", frame_edge);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(frame_edge, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours.size(); i++) {
        std::vector<cv::Point> approx;
        double epsilon = 0.02 * cv::arcLength(contours[i], true);
        cv::approxPolyDP(contours[i], approx, epsilon, true);
        if (approx.size() == 4 && fabs(cv::contourArea(contours[i])) > 800) {
            double max_cos = 0;
            for (int j = 2; j < 5; j++) {
                double cos = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                max_cos = MAX(max_cos, cos);
            }
            cv::Rect rect = cv::boundingRect(approx);
            double d_w = fabs(((double) rect.width) / ((double) rect.height));
            if (max_cos < 0.3 /*&& d_w < 1.15 && d_w > 0.85*/) {
                cv::drawContours(frame, contours, i, cv::Scalar(255, 0, 0));
                std::stringstream ss;
                ss << rect.x << " | " << rect.y << " | " << rect.width << " | " << rect.height << " | ";
                ss << odom->pose.pose.position.x << " | " << odom->pose.pose.position.y << " | ";
                ss << robot_angle << " | " << 1 << " | " << 0;
                (*file) << ss.str() << std::endl;
                break;
            }
        }
    }
    cv::imshow("c", frame);
    cv::waitKey(1);
}

void get_odom(const nav_msgs::OdometryConstPtr &odom_msg) {
    odom = odom_msg;
    tf::Quaternion quaternion(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
                              odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    double yaw = tf::getYaw(quaternion);
    robot_angle = yaw * 180 / M_PI;
}