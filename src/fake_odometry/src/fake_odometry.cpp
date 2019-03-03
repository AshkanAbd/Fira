#include "../include/fake_odometry/fake_odometry.h"

using namespace fake_odometry;

FakeOdometry::FakeOdometry(const std::string &odom_topic, const std::string &cmd_topic,
                           double hz, const ros::NodeHandlePtr &nh, const double &initial_x, const double &initial_y,
                           const double &initial_theta, const std::string &odom_frame, const std::string &child_frame) {
    FakeOdometry::initialize(odom_topic, cmd_topic, hz, nh, initial_x, initial_y, initial_theta, odom_frame,
                             child_frame);
}

void FakeOdometry::get_cmd(const geometry_msgs::TwistConstPtr &twist) {
    vx = twist->linear.x;
    yth = twist->angular.z;
}

void FakeOdometry::set_vel_to_pos() {
    // calculate position change between current time and last time
    double delta_time = current_time.toSec() - last_time.toSec();
    vel_x = (vx * cos(ang_z)) * delta_time;
    vel_y = (vx * sin(ang_z)) * delta_time;
    rot_z = yth * delta_time;
    // add changes to main odometry position
    pos_x += vel_x;
    pos_y += vel_y;
    ang_z += rot_z;
}

// Preparation and broadcasting odom stamp over the ROS
void FakeOdometry::broadcast_odom(const geometry_msgs::Quaternion &odom_quat) {
    transformStamped->header.stamp = current_time;
    transformStamped->header.frame_id = FakeOdometry::odom_frame;
    transformStamped->child_frame_id = FakeOdometry::child_frame;
    transformStamped->transform.rotation = odom_quat;
    transformStamped->transform.translation.x = pos_x;
    transformStamped->transform.translation.y = pos_y;
    transformStamped->transform.translation.z = 0;
    transformStamped->header.seq = FakeOdometry::seq;
    FakeOdometry::broadcaster->sendTransform(*FakeOdometry::transformStamped);
}

// Preparation and publishing odom message over the ROS
void FakeOdometry::publish_odom(const geometry_msgs::Quaternion &odom_quat) {
    publish_obj->header.stamp = current_time;
    publish_obj->header.frame_id = FakeOdometry::odom_frame;
    publish_obj->child_frame_id = FakeOdometry::child_frame;
    publish_obj->pose.pose.position.x = pos_x;
    publish_obj->pose.pose.position.y = pos_y;
    publish_obj->pose.pose.orientation = odom_quat;
    publish_obj->twist.twist.linear.x = vel_x;
    publish_obj->twist.twist.linear.y = vel_y;
    publish_obj->twist.twist.angular.z = rot_z;
    publish_obj->header.seq = FakeOdometry::seq;
    FakeOdometry::odom_publisher->publish(*FakeOdometry::publish_obj);
}

void FakeOdometry::spin() {
    ros::spin();
}

void FakeOdometry::initialize(const std::string &odom_topic, const std::string &cmd_topic, double hz,
                              const ros::NodeHandlePtr &nh, const double &initial_x, const double &initial_y,
                              const double &initial_theta, const std::string &odom_frame,
                              const std::string &child_frame) {
    FakeOdometry::pos_x = initial_x;
    FakeOdometry::pos_y = initial_y;
    FakeOdometry::ang_z = initial_theta;
    FakeOdometry::child_frame = child_frame;
    FakeOdometry::odom_frame = odom_frame;
    FakeOdometry::hz = hz;
    FakeOdometry::nh = nh;
    FakeOdometry::rate = new ros::Rate(hz);
    FakeOdometry::broadcaster = new tf::TransformBroadcaster;
    FakeOdometry::transformStamped = new geometry_msgs::TransformStamped;
    odom_publisher = new ros::Publisher(FakeOdometry::nh->advertise<nav_msgs::Odometry>(odom_topic, 10000));
    cmd_subscriber = new ros::Subscriber(FakeOdometry::nh->subscribe(cmd_topic, 10000, &FakeOdometry::get_cmd, this));
    publish_obj = nav_msgs::OdometryPtr(new nav_msgs::Odometry);
    last_time = ros::Time::now();
    current_time = ros::Time::now();

    // while ROS is not shutdown, in given rate, calculate odometry then publish and broadcast it... :)
    while (FakeOdometry::nh->ok()) {
        ros::spinOnce();
        current_time = ros::Time::now();

        FakeOdometry::set_vel_to_pos();
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(ang_z);
        FakeOdometry::broadcast_odom(odom_quat);
        FakeOdometry::publish_odom(odom_quat);
        FakeOdometry::seq++;

        FakeOdometry::rate->sleep();
        last_time = current_time;
    }
}

FakeOdometry::FakeOdometry() = default;


int main(int argc, char **argv) {
    ros::init(argc, argv, "fake_odom");
    ros::NodeHandlePtr nh(new ros::NodeHandle);
    std::string odom_topic = "fake_odom";
    std::string cmd_topic = "/cmd_vel_mux/input/navi";
    std::string odom_frame = "odom";
    std::string child_frame = "base_footprint";
    double hz = 10;
    double initial_x = 0;
    double initial_y = 0;
    double initial_theta = 0;
    if (nh->hasParam(ros::this_node::getName() + "/odom_topic")) {
        nh->getParam(ros::this_node::getName() + "/odom_topic", odom_topic);
    }
    if (nh->hasParam(ros::this_node::getName() + "/cmd_topic")) {
        nh->getParam(ros::this_node::getName() + "/cmd_topic", cmd_topic);
    }
    if (nh->hasParam(ros::this_node::getName() + "/hz")) {
        nh->getParam(ros::this_node::getName() + "/hz", hz);
    }
    if (nh->hasParam(ros::this_node::getName() + "/initial_x")) {
        nh->getParam(ros::this_node::getName() + "/initial_x", initial_x);
    }
    if (nh->hasParam(ros::this_node::getName() + "/initial_y")) {
        nh->getParam(ros::this_node::getName() + "/initial_y", initial_y);
    }
    if (nh->hasParam(ros::this_node::getName() + "/initial_theta")) {
        nh->getParam(ros::this_node::getName() + "/initial_theta", initial_theta);
    }
    if (nh->hasParam(ros::this_node::getName() + "/odom_frame")) {
        nh->getParam(ros::this_node::getName() + "/odom_frame", odom_frame);
    }
    if (nh->hasParam(ros::this_node::getName() + "/child_frame")) {
        nh->getParam(ros::this_node::getName() + "/child_frame", child_frame);
    }
    FakeOdometry fake_odom(odom_topic, cmd_topic, hz, nh, initial_x, initial_y, initial_theta, odom_frame, child_frame);
    fake_odom.spin();
}