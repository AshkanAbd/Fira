#ifndef FAKE_ODOMETRY_FAKE_ODOMETRY_H
#define FAKE_ODOMETRY_FAKE_ODOMETRY_H

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <thread>
#include <math.h>

namespace fake_odometry {
    class FakeOdometry {
    protected:
        // main objects and pointers
        ros::NodeHandlePtr nh;
        ros::Publisher *odom_publisher;
        ros::Subscriber *cmd_subscriber;
        ros::Rate *rate;
        nav_msgs::OdometryPtr publish_obj;
        ros::Time current_time, last_time;
        std::thread *publish_thread;
        std::string odom_frame, child_frame;
        tf::TransformBroadcaster *broadcaster;
        geometry_msgs::TransformStamped *transformStamped;
        double hz;
        uint seq = 0;
        // position and odometry of robot
        double vel_x = 0.0, vel_y = 0.0, rot_z = 0.0;
        double pos_x = 0.0, pos_y = 0.0, ang_z = 0.0;
        double vx = 0.0, vy = 0.0, yth = 0.0;

    public:

        FakeOdometry(const std::string &odom_topic, const std::string &cmd_topic, double hz,
                     const ros::NodeHandlePtr &nh, const double &initial_x, const double &initial_y,
                     const double &initial_theta, const std::string &odom_frame, const std::string &child_frame);

        FakeOdometry();

        virtual void initialize(const std::string &odom_topic, const std::string &cmd_topic, double hz,
                                const ros::NodeHandlePtr &nh, const double &initial_x, const double &initial_y,
                                const double &initial_theta, const std::string &odom_frame,
                                const std::string &child_frame);

        virtual void get_cmd(const geometry_msgs::TwistConstPtr &twist);

        virtual void publish_odom(const geometry_msgs::Quaternion &odom_quat);

        virtual void broadcast_odom(const geometry_msgs::Quaternion &odom_quat);

        virtual void publisher();

        virtual void set_vel_to_pos();

        virtual void spin();
    };
};
#endif