#pragma once
#include <mutex>
#include "../include/orb_slam3_utility.h"
#include "../include/OccupancyGrid.h"
#include <queue>
#include <limits>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>

#include <tf/tf.h>
#include <tf2_eigen/tf2_eigen.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "../../include/System.h"

class ORB_SLAM3_Mapper
{
    private:
    ORB_SLAM3::Map* map_ptr;    
    OccupancyGrid occupancy_grid;
    std::mutex map_mutex; //Mutex for the map of this class.
    std::mutex finsh_mutex;
    std::queue<ORB_SLAM3::Map*> map_queue;
    std::queue<ORB_SLAM3::KeyFrame*> keyframe_queue;
    bool running;

    size_t initial_kf_id;
    size_t last_kf_id;
    bool first_map;

    ros::Publisher* grid_pub;

    ORB_SLAM3::Map* pop_queue();    


    public:

    ORB_SLAM3_Mapper();

    void set_map(ORB_SLAM3::Map* map);
    void set_publisher(ros::Publisher* pub)
    {
        grid_pub = pub;
    }
    void run();
    void stop();
    bool check_stop();
    void queue_map(ORB_SLAM3::Map* map);

};