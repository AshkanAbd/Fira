#include "../include/depth_detector/data_set.h"

using namespace depth_detector;

RGBDataSet::RGBDataSet(int image_x, int image_y, int image_width, int image_height, double robot_pose_x,
                       double robot_pose_y, double robot_angle, double obstacle_x, double obstacle_y) {
    RGBDataSet::image_x = image_x;
    RGBDataSet::image_y = image_y;
    RGBDataSet::image_width = image_width;
    RGBDataSet::image_height = image_height;
    RGBDataSet::robot_pose_x = robot_pose_x;
    RGBDataSet::robot_pose_y = robot_pose_y;
    RGBDataSet::robot_angle = robot_angle;
    RGBDataSet::obstacle_x = obstacle_x;
    RGBDataSet::obstacle_y = obstacle_y;
}


double RGBDataSet::getRobot_pose_x() const {
    return robot_pose_x;
}

double RGBDataSet::getRobot_pose_y() const {
    return robot_pose_y;
}

double RGBDataSet::getRobot_angle() const {
    return robot_angle;
}

int RGBDataSet::getImage_x() const {
    return image_x;
}

int RGBDataSet::getImage_y() const {
    return image_y;
}

int RGBDataSet::getImage_width() const {
    return image_width;
}

int RGBDataSet::getImage_height() const {
    return image_height;
}

double RGBDataSet::getObstacle_x() const {
    return obstacle_x;
}

double RGBDataSet::getObstacle_y() const {
    return obstacle_y;
}

RGBDataSet::RGBDataSet(const std::string &line) {
    std::stringstream *ss = new std::stringstream;
    int i = 1;
    for (const auto &c : line) {
        if (c == ' ') {
            continue;
        }
        if (c == '|') {
            RGBDataSet::fill(ss->str(), i++);
            ss = new std::stringstream;
            continue;
        }
        (*ss) << c;
    }
    RGBDataSet::fill(ss->str(), i);
}

void RGBDataSet::fill(const std::string &str, int i) {
    switch (i) {
        case 1:
            RGBDataSet::setImage_x(std::stoi(str));
            break;
        case 2:
            RGBDataSet::setImage_y(std::stoi(str));
            break;
        case 3:
            RGBDataSet::setImage_width(std::stoi(str));
            break;
        case 4:
            RGBDataSet::setImage_height(std::stoi(str));
            break;
        case 5:
            RGBDataSet::setRobot_pose_x(std::stod(str));
            break;
        case 6:
            RGBDataSet::setRobot_pose_y(std::stod(str));
            break;
        case 7:
            RGBDataSet::setRobot_angle(std::stod(str));
            break;
        case 8:
            RGBDataSet::setObstacle_x(std::stod(str));
            break;
        case 9:
            RGBDataSet::setObstacle_y(std::stod(str));
            break;
        default:
            std::cout << "UNKNOWN" << std::endl;
            break;
    }
}

void RGBDataSet::setRobot_pose_x(double robot_pose_x) {
    RGBDataSet::robot_pose_x = robot_pose_x;
}

void RGBDataSet::setRobot_pose_y(double robot_pose_y) {
    RGBDataSet::robot_pose_y = robot_pose_y;
}

void RGBDataSet::setRobot_angle(double robot_angle) {
    RGBDataSet::robot_angle = robot_angle;
}

void RGBDataSet::setImage_x(int image_x) {
    RGBDataSet::image_x = image_x;
}

void RGBDataSet::setImage_y(int image_y) {
    RGBDataSet::image_y = image_y;
}

void RGBDataSet::setImage_width(int image_width) {
    RGBDataSet::image_width = image_width;
}

void RGBDataSet::setImage_height(int image_height) {
    RGBDataSet::image_height = image_height;
}

void RGBDataSet::setObstacle_x(double obstacle_x) {
    RGBDataSet::obstacle_x = obstacle_x;
}

void RGBDataSet::setObstacle_y(double obstacle_y) {
    RGBDataSet::obstacle_y = obstacle_y;
}

RGBDataSet::RGBDataSet() = default;

DepthDataSet::DepthDataSet(double distance, float depth) {
    DepthDataSet::distance = distance;
    DepthDataSet::depth = depth;
}

void DepthDataSet::fill(const std::string &str, int i) {
    switch (i) {
        case 1:
            DepthDataSet::setDepth(std::stof(str));
            break;
        case 2:
            DepthDataSet::setDistance(std::stod(str));
            break;
        default:
            std::cout << "UNKNOWN" << std::endl;
            break;
    }
}

DepthDataSet::DepthDataSet(const std::string &line) {
    std::stringstream *ss = new std::stringstream;
    int i = 1;
    for (const auto &c : line) {
        if (c == ' ') {
            continue;
        }
        if (c == '|') {
            DepthDataSet::fill(ss->str(), i++);
            ss = new std::stringstream;
            continue;
        }
        (*ss) << c;
    }
    DepthDataSet::fill(ss->str(), i);
}

double DepthDataSet::getDistance() const {
    return distance;
}

float DepthDataSet::getDepth() const {
    return depth;
}

void DepthDataSet::setDistance(double distance) {
    DepthDataSet::distance = distance;
}

void DepthDataSet::setDepth(float depth) {
    DepthDataSet::depth = depth;
};

DepthDataSet::DepthDataSet() = default;
