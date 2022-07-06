#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<math.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ORB_SLAM3_msgs/orb_slam_map_msg.h>



#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ORB_SLAM3_msgs/orb_slam_map_msg.h>
#include <ORB_SLAM3_msgs/keyframe_pointcloud.h>

#include<opencv2/core/core.hpp>

#include "utility.h"


class ORB_SLAM3_Map
{
    public:
    ORB_SLAM3_Map(uint64 width, uint64 height, float resolution):id(0)
    {
        init_occupancy_grid(occ_grid,"map",width, height, resolution);
        occupied = cv::Mat::zeros(width, height, CV_16U);
        free = cv::Mat::zeros(width, height, CV_16U);
        grid = cv::Mat(width, height, CV_8SC1,50);
        map_center = cv::Point2d(occ_grid.info.width/2, occ_grid.info.height/2);
    };

    ORB_SLAM3_Map(const ORB_SLAM3_msgs::orb_slam_map_msg& msg)
    {
        size_t new_width = msg.max_point.x - msg.min_point.x;
        size_t new_height = msg.max_point.y - msg.max_point.y;

        if(new_width > occ_grid.info.width && new_height > occ_grid.info.height)
        {


        }

    }

    ~ORB_SLAM3_Map(){};

    void init_occupancy_grid(nav_msgs::OccupancyGrid& occ_grid, const std::string& frame_id, uint64 width, uint64 height, float resolution)
    {
        occ_grid.header.frame_id = frame_id;
        occ_grid.info.width = (width);
        occ_grid.info.height = (height);
        occ_grid.info.resolution = resolution;
        occ_grid.info.origin.position.x = -(occ_grid.info.width*occ_grid.info.resolution)/2;
        occ_grid.info.origin.position.y = -(occ_grid.info.height*occ_grid.info.resolution)/2;
    }

    void world_to_grid_coordinates(double wx, double wy, int64& mx, int64& my)
    {
        mx = floor(wx / occ_grid.info.resolution + map_center.x);
        my = floor(wy / occ_grid.info.resolution + map_center.y);     
    }

    void update(const geometry_msgs::Pose& camera_pose, const geometry_msgs::PoseArray& poses,  double max_z, double min_z, double max_dist)
    {
        int64 cam_x, cam_y;
        world_to_grid_coordinates(camera_pose.position.x, camera_pose.position.y, cam_x, cam_y);

        for(int i = 0; i != poses.poses.size(); i++)
        {      
            const geometry_msgs::Point& curr_pnt = poses.poses[i].position;

            if(curr_pnt.z > max_z || curr_pnt.z < min_z) continue;
            double dist = pow(curr_pnt.x - camera_pose.position.x, 2) + pow(curr_pnt.y - camera_pose.position.y,2);
            if( dist > pow(max_dist,2))continue;

            int64 xh, yh;
            world_to_grid_coordinates(curr_pnt.x, curr_pnt.y, xh, yh);

            if( (xh>0 && xh < occ_grid.info.width-1 ) && (yh > 0 && yh < occ_grid.info.height-1 ))
            {
                occupied.at<u_int16_t>(yh,xh) += 1;
                for(cv::Point2d p : bresenham_line(cam_x,cam_y,xh,yh))
                {
                    free.at<u_int16_t>(p.y,p.x) += 1;
                }
            }
        }

        for(int i=0; i!=grid.rows;i++)
        {
            for(int j=0; j!=grid.cols;j++)
            {
                if(free.at<u_int16_t>(i,j)==occupied.at<u_int16_t>(i,j)&&free.at<u_int16_t>(i,j)==0)continue;
                printf("free: %d vs Occ: %d\n",free.at<u_int16_t>(i,j),occupied.at<u_int16_t>(i,j));
                if( free.at<u_int16_t>(i,j) > occupied.at<u_int16_t>(i,j) + 5)grid.at<schar>(i,j) = 0; 
                else if(occupied.at<u_int16_t>(i,j) > free.at<u_int16_t>(i,j)+15)grid.at<schar>(i,j) = 100; 
                else grid.at<schar>(i,j) = 50;
            }
        }

        occ_grid.data = std::vector<signed char>(grid.datastart, grid.dataend);
    }

    const nav_msgs::OccupancyGrid& get_occupancy_grid()
    {
        return occ_grid;
    }

    private:

    uint64 id;
    cv::Mat occupied;
    cv::Mat free;
    cv::Mat grid;
    nav_msgs::OccupancyGrid occ_grid;
    cv::Point2d map_center;
    
    ORB_SLAM3_msgs::orb_slam_map_msg map_msg;
};

class MapMaker
{
    public:
    MapMaker(int width, int height)
    {
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("orb_map",10);
        keyframe_poses_pub = nh.advertise<geometry_msgs::PoseArray>("kf_poses",10);
        min_pub = nh.advertise<geometry_msgs::PoseStamped>("min_point",10);
        max_pub = nh.advertise<geometry_msgs::PoseStamped>("max_point",10);

        nh.param<double>("max_z", max_z, 2.0);
        nh.param<double>("min_z", min_z, -1.0);
        nh.param<int>("map_factor", map_factor, 3);
        nh.param<double>("max_dist",max_dist,4.0);

        init_occupancy_grid(map,"map",width, height,0.01);
        map_center = cv::Point2d(width/2, height/2);

        init_occupancy_grid(height_map, "map",width/map_factor,height/map_factor, map.info.resolution * map_factor);
        height_map_center = cv::Point2d(height_map.info.width/2, height_map.info.height/2);
        ROS_INFO("Height map @ %f, %f",height_map.info.origin.position.x,height_map.info.origin.position.y);

//        grid = cv::Mat::zeros(width,height, CV_8SC1);
        height_grid = cv::Mat(width/map_factor, height/map_factor, CV_8SC1,50);
        free = cv::Mat::zeros(width/map_factor, height/map_factor, CV_16U);
        occupied = cv::Mat::zeros(width/map_factor, height/map_factor, CV_16U);

    } 

    ~MapMaker(){};


    void init_occupancy_grid(nav_msgs::OccupancyGrid& occ_grid, const std::string& frame_id, int width, int height, float resolution)
    {
        occ_grid.header.frame_id = frame_id;
        occ_grid.info.width = (width);
        occ_grid.info.height = (height);
        occ_grid.info.resolution = resolution;
        occ_grid.info.origin.position.x = -(occ_grid.info.width*occ_grid.info.resolution)/2;
        occ_grid.info.origin.position.y = -(occ_grid.info.height*occ_grid.info.resolution)/2;
    }

    void orb_slam_map_callback(const ORB_SLAM3_msgs::orb_slam_map_msgConstPtr& msg)
    {
        ORB_SLAM3_msgs::orb_slam_map_msg map = *msg;
        geometry_msgs::PoseStamped min_pose, max_pose;
        max_pose.header.frame_id = min_pose.header.frame_id = "map";
        min_pose.pose.position = map.min_point;
        max_pose.pose.position = map.max_point;
        max_pub.publish(max_pose);
        min_pub.publish(min_pose);

    }

    void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        ROS_INFO("Received point cloud!");
        sensor_msgs::PointCloud2 pcl = *msg;

        for(sensor_msgs::PointCloud2Iterator<float> pcl_iter(pcl, "x"); pcl_iter != pcl_iter.end();++pcl_iter)
        {      
            int x = ceil(pcl_iter[0] / map.info.resolution + map.info.width/2);
            int y = ceil(pcl_iter[1] / map.info.resolution + map.info.height/2);
            if( (x>0 && x < map.info.width ) && (y > 0 && y < map.info.height ))
            {
                ROS_INFO("Point xyz: %f, %f, %f; Inserting @ %d, %d", pcl_iter[0],pcl_iter[1],pcl_iter[2],y,x);
                grid.at<schar>(y,x) = 100;
            }
            printf("%d/n",grid.at<int8_t>(y,x));
        }

        ROS_INFO("Publishing map!");
        map.data = std::vector<signed char>(grid.datastart, grid.dataend);
        map.header.stamp = ros::Time();
        map_pub.publish(map);
    }

    void kf_pose_array_callback(const geometry_msgs::PoseArrayConstPtr& msg)
    {
        ROS_INFO("Received pose array!");
        geometry_msgs::PoseArray pose_array = *msg;

        geometry_msgs::Point min, max;
        getMinMax(pose_array,min,max);
        ROS_INFO("Min pt: %.1f, %.1f, %.1f // Max pt: %.1f %.1f %.1f", min.x, min.y, min.x,max.z,max.y,max.z);

        //The LAST element of the pose array is expected to be the keyframe pose.
        kf_poses.push_back(pose_array.poses.back());
        pose_array.poses.pop_back();
        kf_map_points_vector.push_back(move(pose_array.poses));

        update_map_with_pose_array(kf_map_points_vector.back(), kf_poses.back(),height_grid);
    
        ROS_INFO("Publishing map!");
        height_map.data = std::vector<signed char>(height_grid.datastart, height_grid.dataend);
        height_map.header.stamp = ros::Time();
        map_pub.publish(height_map);   
    }

    void update_map_with_pose_array(const std::vector<geometry_msgs::Pose>& poses, const geometry_msgs::Pose& camera_pose, cv::Mat& mat_grid)
    {    
        int cam_x = floor(camera_pose.position.x / height_map.info.resolution + height_map_center.x);
        int cam_y = floor(camera_pose.position.y / height_map.info.resolution + height_map_center.y);

        for(size_t i = 0; i != poses.size(); i++)
        {      
            const geometry_msgs::Point& curr_pnt = poses[i].position;
            if(curr_pnt.z > max_z || curr_pnt.z < min_z) continue;
            double dist = pow(curr_pnt.x - camera_pose.position.x, 2) + pow(curr_pnt.y - camera_pose.position.y,2);
            if( dist > pow(max_dist,2))continue;

            size_t xh = floor(curr_pnt.x / height_map.info.resolution + height_map_center.x);
            size_t yh = floor(curr_pnt.y / height_map.info.resolution + height_map_center.y);
            if( (xh>0 && xh < height_map.info.width-1 ) && (yh > 0 && yh < height_map.info.height-1 ))
            {
                //ROS_INFO("HeightMap: Point xyz: %f, %f; Inserting @ %d, %d", poses[i].position.x,poses[i].position.y,yh,xh);
                occupied.at<u_int16_t>(yh,xh) += 1;
                for(cv::Point2d p : bresenham_line(cam_x,cam_y,xh,yh))
                {
                    free.at<u_int16_t>(p.y,p.x) += 1;
                }
                // if(mat_grid.at<schar>(yh,xh)>100)mat_grid.at<schar>(yh,xh) = 100;
                // if(mat_grid.at<schar>(yh,xh)<=0)mat_grid.at<schar>(yh,xh) = 0;
            }
        }

        for(int i=0; i!=mat_grid.rows;i++)
        {
            for(int j=0; j!=mat_grid.cols;j++)
            {
                if(free.at<u_int16_t>(i,j)==occupied.at<u_int16_t>(i,j)&&free.at<u_int16_t>(i,j)==0)continue;
                printf("free: %d vs Occ: %d\n",free.at<u_int16_t>(i,j),occupied.at<u_int16_t>(i,j));
                if( free.at<u_int16_t>(i,j) > occupied.at<u_int16_t>(i,j) + 5)mat_grid.at<schar>(i,j) = 0; 
                else if(occupied.at<u_int16_t>(i,j) > free.at<u_int16_t>(i,j)+15)mat_grid.at<schar>(i,j) = 100; 
                else mat_grid.at<schar>(i,j) = 50;
            }
        }

    }

    private:

    ros::NodeHandle nh;
    ros::Publisher map_pub;
    ros::Publisher keyframe_poses_pub;
    ros::Publisher min_pub, max_pub;

    cv::Mat grid;
    cv::Mat height_grid;
    cv::Mat free, occupied;

    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid height_map;
    cv::Point2d map_center, height_map_center;

    double max_z, min_z,max_dist;
    int map_factor;

    std::vector<std::vector<geometry_msgs::Pose>> kf_map_points_vector;
    std::vector<geometry_msgs::Pose> kf_poses;

    std::map<int, ORB_SLAM3_Map> stored_maps;

};


ros::Publisher pose_pub;
ros::Publisher pcl_pub;

void kf_pcl_cb(ORB_SLAM3_msgs::keyframe_pointcloudConstPtr ptr)
{
    ORB_SLAM3_msgs::keyframe_pointcloud kf_pcl = *ptr;

    geometry_msgs::PoseStamped pose;
    pose.pose = kf_pcl.pose;
    pose.header = kf_pcl.header;

    pose_pub.publish(pose);
    pcl_pub.publish(kf_pcl.pointcloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_maker");
    ros::start();

    ros::NodeHandle nh;
    MapMaker map_maker(1001,1001);
    //ORB_SLAM_Mapper om(&nh);
    ros::Subscriber map_sub = nh.subscribe<ORB_SLAM3_msgs::keyframe_pointcloud>("/ORB_SLAM3_RGBD/keyframe_pointcloud",5,kf_pcl_cb);
    
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("kf_pose", 5);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl",5);

    ros::spin();

    ros::shutdown();

    return 0;
}