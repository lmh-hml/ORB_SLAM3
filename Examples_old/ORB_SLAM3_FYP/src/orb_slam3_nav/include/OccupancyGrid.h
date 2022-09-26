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

    public:
    OccupancyGrid(size_t w, size_t h, double resolution)
    {
        occupancy_grid.header.frame_id = "map";
        occupancy_grid.info.width = w;
        occupancy_grid.info.height = h;
        occupancy_grid.info.resolution = resolution;
        occupancy_grid.info.origin.position.x = -(w*resolution)/2;
        occupancy_grid.info.origin.position.y = -(h*resolution)/2;
        occupancy_grid.data.resize(w*h);
        image = cv::Mat(w,h, CV_8U,50);

        free = cv::Mat::zeros(w,h,CV_16U);
        occupied = cv::Mat::zeros(w,h,CV_16U);
        prob = cv::Mat::zeros(w,h,CV_16U);
    }

    void resize_image(size_t up, size_t down, size_t left, size_t right)
    {
        up = max(0.0f,ceil(up/occupancy_grid.info.resolution - image.rows/2));
        down = max(0.0f,ceil(down/occupancy_grid.info.resolution - image.rows/2));
        left = max(0.0f,ceil(left/occupancy_grid.info.resolution - image.cols/2));
        right = max(0.0f,ceil(right/occupancy_grid.info.resolution - image.cols/2));
        printf("Padding image by: udlr: %zu, %zu, %zu, %zu\n",up,down,left,right);

        cv::copyMakeBorder(image, image, up, down, left, right,cv::BORDER_CONSTANT, 0);
        cv::copyMakeBorder(free, free, up, down, left, right,cv::BORDER_CONSTANT, 0);
        cv::copyMakeBorder(occupied, occupied, up, down, left, right,cv::BORDER_CONSTANT, 0);
        cv::copyMakeBorder(prob, prob, up, down, left, right,cv::BORDER_CONSTANT, 0);

        occupancy_grid.info.width = image.cols;
        occupancy_grid.info.height = image.rows;
        occupancy_grid.info.origin.position.x = -(image.cols*occupancy_grid.info.resolution)/2;
        occupancy_grid.info.origin.position.y = -(image.rows*occupancy_grid.info.resolution)/2;
    }

    void clear()
    {
        image = image.zeros(image.size(), image.type());
        free = free.zeros(free.size(),free.type());
        occupied = occupied.zeros(occupied.size(),occupied.type());
        prob = prob.zeros(prob.size(),prob.type());
    }

    bool set(size_t x, size_t y, int value)
    {
        if(x >= occupancy_grid.info.width|| y >= occupancy_grid.info.height)return false;
        size_t index = (y*occupancy_grid.info.width) + x;
        occupancy_grid.data.at(index) = value;
        return true;
    }

    void line(size_t x, size_t y, size_t x2, size_t y2, int value)
    {
        for( auto p : bresenham_line(x,y,x2,y2)) set(p.x,p.y,value);
    }

    void cv_line(size_t x, size_t y, size_t x2, size_t y2, int value)
    {
        cv::line(image,cv::Point(x,y),cv::Point(x2,y2),cv::Scalar(value),10,cv::LINE_4,0);
    }

    bool cv_mark_occupied(size_t x, size_t y)
    {
        if(x<0||y<0||x>=occupied.cols||y>=occupied.rows)return false;
        occupied.at<u_int16_t>(y,x)+=1;
        return true;
    }

    bool cv_mark_free(size_t x, size_t y)
    {
        if(x<0||y<0||x>=free.cols||y>=free.rows)return false; 
        free.at<u_int16_t>(y,x)+=1;
        return true;
    }
    
    void cvMat_to_occupancy_grid()
    {
        printf("image size is %d, %d\n",image.cols, image.rows);
        occupancy_grid.data = std::vector<signed char>(image.datastart, image.dataend);
    }

    nav_msgs::OccupancyGrid get_grid()
    {
        return occupancy_grid;
    }

    void process_free_and_occupied(uint16_t free_thresh=5)
    {
        for(int i=0; i!=image.rows;i++)
        {
            for(int j=0; j!=image.cols;j++)
            {
                float prob_free = 1.0 - float(occupied.at<uint16_t>(i,j)) / float(free.at<uint16_t>(i,j));
                if(prob_free>0.55 && free.at<uint16_t>(i,j)>free_thresh)image.at<int8_t>(i,j) = 0;
                else if(prob_free<0.5)image.at<int8_t>(i,j) = 100;
                else image.at<int8_t>(i,j) = 50;
            }
        }
    }

    size_t get_width(){return image.cols;};
    size_t get_height(){return image.rows;};

};