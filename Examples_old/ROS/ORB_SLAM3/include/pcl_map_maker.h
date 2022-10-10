#pragma once
#include <mutex>
#include "../include/orb_slam3_utility.h"
#include "../include/OccupancyGrid.h"
#include <queue>
#include <limits>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>

#include <tf/tf.h>
#include <tf2_eigen/tf2_eigen.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "../../include/System.h"
typedef pcl::PointCloud<pcl::PointXYZ> Pointcloud;
typedef pcl::PointCloud<pcl::PointXYZL> PointcloudXYZL;

class ORB_SLAM3_Mapper
{
    private:
    ORB_SLAM3::Map* map_ptr;    
    OccupancyGrid occupancy_grid;
    std::mutex map_mutex; //Mutex for the map of this class.
    std::mutex filter_mutex;
    std::mutex finsh_mutex;
    std::queue<ORB_SLAM3::Map*> map_queue;
    std::queue<ORB_SLAM3::KeyFrame*> keyframe_queue;
    bool running;

    size_t initial_kf_id;
    size_t last_kf_id;
    bool first_map;
    uint16_t free_thresh;
    double height_thresh; //Maximum acceptable height of points
    double time_thresh;   //Force map update every time_thresh in secs
    double dist_thresh;   //Maximum distance of a point from its keyframe to be considered as valid
    int gaussian_ksize;   //Used to smooth out counters in occupacy grids
    tf2::Transform point_cloud_offset;

    double filter_radius;
    int    filter_min_neighbors;
    double min_z, max_z;
    uint16_t free_value, occupied_value;

    ros::Publisher* grid_pub;
    ros::Publisher* pcl_pub;
    ORB_SLAM3::Map* pop_queue();    

    bool map_points_labeled_to_point_cloud(const std::vector<ORB_SLAM3::MapPoint *> &map_points, PointcloudXYZL::Ptr pcl);
    void filter_pointcloud(PointcloudXYZL::Ptr pcl_ptr);
    double get_kf_mp_dist(ORB_SLAM3::KeyFrame* kf, ORB_SLAM3::MapPoint* mp);


    public:

    ORB_SLAM3_Mapper();

    void set_map(ORB_SLAM3::Map* map);
    inline void set_grid_pub(ros::Publisher* pub)
    {
        grid_pub = pub;
    }

    inline void set_pcl_pub(ros::Publisher* pub)
    {
        pcl_pub = pub;
    }
    void run();
    void stop();
    bool check_stop();
    void queue_map(ORB_SLAM3::Map* map);

    void set_filter_radius(double rad)
    {
        filter_radius = rad;
    }

    void set_filter_min_neighbors(int n)
    {
        filter_min_neighbors = n;
    }

    void set_max_z(double max)
    {
        max_z = max;
    }

    void set_min_z(double min)
    {
        min_z = min;
    }

    inline void set_point_cloud_offset(tf2::Transform mat)
    {
        point_cloud_offset = mat;
    }

    inline void set_free_thresh(uint16_t thresh)
    {
        free_thresh = thresh;
    }

    inline void set_dist_thresh(double thresh)
    {
        dist_thresh = thresh;
    }

    inline void set_gauusian_ksize(int ksize)
    {
        gaussian_ksize=ksize;
    }

    inline void set_free_value(uint16_t val){ free_value = val;}
    inline void set_occupied_value(uint16_t val){ occupied_value = val;}

};