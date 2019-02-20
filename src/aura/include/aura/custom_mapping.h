#ifndef AURA_TEST_MAPPING_H
#define AURA_TEST_MAPPING_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pc_mapping/pc_mapping.h>

class AuraMapping : public pc_mapping::PCMapping {
public:
    int initial_value;

    AuraMapping();

    void initialize_map(uint height, uint width, float resolution, float initial_x, float initial_y) override;

};

#endif
