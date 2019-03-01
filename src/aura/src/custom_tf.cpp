#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>

tf::Transform *transform;
tf::TransformBroadcaster *br;
ros::NodeHandlePtr nh;
std::string *pre_fix;

tf::Vector3 *odom_to_base_footprint_v;
tf::Quaternion *odom_to_base_footprint_q;

tf::Vector3 *base_footprint_to_base_link_v;
tf::Quaternion *base_footprint_to_base_link_q;

tf::Vector3 *base_link_to_camera_rgb_v;
tf::Quaternion *base_link_to_camera_rgb_q;

tf::Vector3 *camera_rgb_to_camera_depth_v;
tf::Quaternion *camera_rgb_to_camera_depth_q;

tf::Vector3 *camera_depth_to_camera_depth_optical_v;
tf::Quaternion *camera_depth_to_camera_depth_optical_q;

tf::Vector3 *camera_rgb_to_camera_rgb_optical_v;
tf::Quaternion *camera_rgb_to_camera_rgb_optical_q;

tf::Vector3 *camera_rgb_to_camera_link_v;
tf::Quaternion *camera_rgb_to_camera_link_q;

void odom_to_base_footprint(const ros::Time &time) {
    transform->setOrigin(*odom_to_base_footprint_v);
    transform->setRotation(*odom_to_base_footprint_q);
    br->sendTransform(tf::StampedTransform(*transform, time, (*pre_fix) + "/odom", (*pre_fix) + "/base_footprint"));
}

void base_footprint_to_base_link(const ros::Time &time) {
    transform->setOrigin(*base_footprint_to_base_link_v);
    transform->setRotation(*base_footprint_to_base_link_q);
    br->sendTransform(
            tf::StampedTransform(*transform, time, (*pre_fix) + "/base_footprint", (*pre_fix) + "/base_link"));
}

void base_link_to_camera_rgb(const ros::Time &time) {
    transform->setOrigin(*base_link_to_camera_rgb_v);
    transform->setRotation(*base_link_to_camera_rgb_q);
    br->sendTransform(
            tf::StampedTransform(*transform, time, (*pre_fix) + "/base_link", (*pre_fix) + "/camera_rgb_frame"));
}

void camera_rgb_to_camera_depth(const ros::Time &time) {
    transform->setOrigin(*camera_rgb_to_camera_depth_v);
    transform->setRotation(*camera_rgb_to_camera_depth_q);
    br->sendTransform(tf::StampedTransform(*transform, time, (*pre_fix) + "/camera_rgb_frame",
                                           (*pre_fix) + "/camera_depth_frame"));
}


void camera_depth_to_camera_depth_optical(const ros::Time &time) {
    transform->setOrigin(*camera_depth_to_camera_depth_optical_v);
    transform->setRotation(*camera_depth_to_camera_depth_optical_q);
    br->sendTransform(tf::StampedTransform(*transform, time, (*pre_fix) + "/camera_depth_frame",
                                           (*pre_fix) + "/camera_depth_optical_frame"));
}

void camera_rgb_to_camera_rgb_optical(const ros::Time &time) {
    transform->setOrigin(*camera_rgb_to_camera_rgb_optical_v);
    transform->setRotation(*camera_rgb_to_camera_rgb_optical_q);
    br->sendTransform(tf::StampedTransform(*transform, time, (*pre_fix) + "/camera_rgb_frame",
                                           (*pre_fix) + "/camera_rgb_optical_frame"));
}

void camera_rgb_to_camera_link(const ros::Time &time) {
    transform->setOrigin(*camera_rgb_to_camera_link_v);
    transform->setRotation(*camera_rgb_to_camera_link_q);
    br->sendTransform(
            tf::StampedTransform(*transform, time, (*pre_fix) + "/camera_rgb_frame", (*pre_fix) + "/camera_link"));
}

void start() {
    transform = new tf::Transform;
    br = new tf::TransformBroadcaster;

    odom_to_base_footprint_v = new tf::Vector3(0.000, 0.000, 0.000);
    odom_to_base_footprint_q = new tf::Quaternion(0.000, 0.000, 0.001, 1.000);

    base_footprint_to_base_link_v = new tf::Vector3(0.000, 0.000, 0.010);
    base_footprint_to_base_link_q = new tf::Quaternion(0.000, 0.000, 0.000, 1.000);

    base_link_to_camera_rgb_v = new tf::Vector3(-0.087, 0.021, 0.287);
    base_link_to_camera_rgb_q = new tf::Quaternion(0.000, 0.000, 0.000, 1.000);

    camera_rgb_to_camera_depth_v = new tf::Vector3(0.000, 0.027, 0.000);
    camera_rgb_to_camera_depth_q = new tf::Quaternion(0.000, 0.000, 0.000, 1.000);

    camera_depth_to_camera_depth_optical_v = new tf::Vector3(0.000, 0.000, 0.000);
    camera_depth_to_camera_depth_optical_q = new tf::Quaternion(-0.500, 0.500, -0.500, 0.500);

    camera_rgb_to_camera_rgb_optical_v = new tf::Vector3(0.000, 0.000, 0.000);
    camera_rgb_to_camera_rgb_optical_q = new tf::Quaternion(-0.500, 0.500, -0.500, 0.500);

    camera_rgb_to_camera_link_v = new tf::Vector3(0.000, -0.022, 0.000);
    camera_rgb_to_camera_link_q = new tf::Quaternion(0.000, 0.000, 0.000, 1.000);

    ros::Rate r(1000);
    while (ros::ok()) {
        ros::Time ros_time = ros::Time::now();
        odom_to_base_footprint(ros_time);

        base_footprint_to_base_link(ros_time);

        base_link_to_camera_rgb(ros_time);

        camera_rgb_to_camera_depth(ros_time);

        camera_depth_to_camera_depth_optical(ros_time);

        camera_rgb_to_camera_rgb_optical(ros_time);

        camera_rgb_to_camera_link(ros_time);

        r.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "aura_tf");
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    pre_fix = new std::string("aura");
    if (nh->hasParam(ros::this_node::getName() + "/pre_fix")) {
        nh->getParam(ros::this_node::getName() + "/pre_fix", *pre_fix);
    }
    start();
}