#ifndef MAPPING_DATA_SET_H
#define MAPPING_DATA_SET_H

#include <iostream>
#include <sstream>

namespace depth_detector {
    struct RGBDataSet {
    private:
        double robot_pose_x, robot_pose_y, robot_angle, obstacle_x, obstacle_y;
        int image_x, image_y, image_width, image_height;

        void setRobot_pose_x(double robot_pose_x);

        void setRobot_pose_y(double robot_pose_y);

        void setRobot_angle(double robot_angle);

        void setImage_x(int image_x);

        void setImage_y(int image_y);

        void setImage_width(int image_width);

        void setImage_height(int image_height);

        void setObstacle_x(double obstacle_x);

        void setObstacle_y(double obstacle_y);

        void fill(const std::string &str, int i);

    public:

        RGBDataSet();

        RGBDataSet(int image_x, int image_y, int image_width = 0, int image_height = 0, double robot_pose_x = 0,
                   double robot_pose_y = 0, double robot_angle = 0, double obstacle_x = 0, double obstacle_y = 0);

        explicit RGBDataSet(const std::string &line);

        double getRobot_pose_x() const;

        double getRobot_pose_y() const;

        double getRobot_angle() const;

        int getImage_x() const;

        int getImage_y() const;

        int getImage_width() const;

        int getImage_height() const;

        double getObstacle_x() const;

        double getObstacle_y() const;
    };

    struct DepthDataSet {
    private:
        double distance;
        float depth;

        void fill(const std::string &str, int i);

        void setDistance(double distance);

        void setDepth(float depth);

    public:
        DepthDataSet(double distance, float depth);

        explicit DepthDataSet(const std::string &line);

        DepthDataSet();

        double getDistance() const;

        float getDepth() const;
    };
};

#endif
