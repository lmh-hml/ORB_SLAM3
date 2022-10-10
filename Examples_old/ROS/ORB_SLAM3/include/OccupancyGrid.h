#pragma once
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<math.h>

#include<ros/ros.h>
#include <orb_slam3_msgs/orb_slam_map_msg.h>
#include <orb_slam3_msgs/keyframe_pointcloud.h>
#include <nav_msgs/OccupancyGrid.h>

#include<opencv2/core/core.hpp>

#include "../include/orb_slam3_utility.h"


class OccupancyGrid
{
    private:
    nav_msgs::OccupancyGrid occupancy_grid;
    cv::Mat image, free, occupied, prob;
    cv::Point2f map_origin;

    public:
    OccupancyGrid(size_t w, size_t h, double resolution);

    void resize_image(size_t up, size_t down, size_t left, size_t right);

    void clear();

    bool set(size_t x, size_t y, int value);

    void line(size_t x, size_t y, size_t x2, size_t y2, int value);

    void cv_line(size_t x, size_t y, size_t x2, size_t y2, int value);

    bool cv_mark_occupied(size_t x, size_t y,uint16_t value=1);

    bool cv_mark_free(size_t x, size_t y,uint16_t value=1);
    
    void cvMat_to_occupancy_grid();

    void blur_counters(int ksize);

    void process_free_and_occupied(uint16_t free_thresh, int ksize);

    size_t get_width(){return image.cols;};
    size_t get_height(){return image.rows;};
    void get_map_origin(size_t&x, size_t&y)
    {
        x = -occupancy_grid.info.origin.position.x/occupancy_grid.info.resolution;
        y = -occupancy_grid.info.origin.position.y/occupancy_grid.info.resolution;
    }

    nav_msgs::OccupancyGrid get_grid()
    {
        return occupancy_grid;
    }

};