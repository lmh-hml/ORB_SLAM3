#pragma once
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

#include <ORB_SLAM3/orb_slam3Config.h>
#include "../include/orb_slam3_utility.h"

#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/transforms.h>

#include<opencv2/core/core.hpp>

#include "../../include/System.h"

#include "../include/pcl_map_maker.h"

typedef pcl::PointCloud<pcl::PointXYZ> Pointcloud;

class ORB_ROS_Node
{
    public:
    ORB_ROS_Node(std::string name, ORB_SLAM3::System::eSensor sensor, ros::NodeHandle* nh );
    ~ORB_ROS_Node();

    void init_ros();

    void init_threads();

    void shutdown();

    void publish_current_tracked_points(ros::Time frame_time);

    void publish_current_transform(ros::Time frame_time);

    void publish_current_global_map_points(ros::Time frame_time, const std::vector<ORB_SLAM3::MapPoint*>& map_points);

    void publish_orb_slam_debug_image(ros::Time frame_time);

    void publish_all_keyframe_pose_and_points(ros::Time frame_time, const std::vector<ORB_SLAM3::KeyFrame*>& keyframes);

    void publish_latest_keyframe_pose_and_points(ros::Time frame_time, const std::vector<ORB_SLAM3::KeyFrame*>& keyframes);
    
    void publish_tracking_state(ros::Time frame_time, int tracking_state);

    void check_atlas_status(ORB_SLAM3::Atlas* atlas, ORB_SLAM3::Map* current_map);
    
    int update_current_position( Sophus::SE3f position);

    void update(Sophus::SE3f position, ros::Time current_frame_time);

    protected:
    ORB_SLAM3::System* orb_system;
    ros::NodeHandle* node_handle;

    private:
    std::string node_name;
    std::string camera_frame_id;
    std::string map_frame_id;
    std::string target_frame_id;
    std::string orb_slam3_map_frame_id;
    std::string base_link_id;
    std::string odom_frame_id;
    ros::Time current_frame_time;

    ros::Publisher global_map_points_pub;
    ros::Publisher filtered_map_points_pub;
    ros::Publisher current_pose_pub;
    ros::Publisher tracking_state_pub;
    ros::Publisher grid_pub;
    ros::Publisher current_tracked_points_pub;

    ros::Subscriber goal_sub;
    ros::Subscriber save_map_sub;

    dynamic_reconfigure::Server<ORB_SLAM3::orb_slam3Config> dr_server;

    image_transport::ImageTransport* image_transport;
    image_transport::Publisher image_pub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    geometry_msgs::PoseStamped received_goal;
    geometry_msgs::Transform   camera_to_goal;
    geometry_msgs::Transform   map_to_camera_msg;

    //Looked up upon node initialization. Used to determine the tf from 2D map to orb_slam3's map origin
    tf2::Transform tf2_base_link_to_camera_origin;
    Eigen::Matrix4f mat4f_base_link_to_camera_origin;

    //Camera position from orb_slam3. Updates every time an image is processed.
    tf2::Transform tf2_orb_slam3_map_to_camera;

    //Used to track map and atlas changes
    size_t initial_kf_id;
    size_t last_kf_size;
    size_t last_map_id;
    size_t last_map_count;
    bool map_id_changed, map_count_changed, is_lost, map_changed;

    ORB_SLAM3_Mapper mapper;
    ORB_SLAM3::System::eSensor sensor_type;
    std::thread* mapper_thread;
    std::thread* point_cloud_thread;
    bool use_mapper;
    bool publish_pointcloud;
    bool publish_image;
    std::mutex point_cloud_thread_mutex;
    bool run_point_cloud_thread;
    bool point_cloud_thread_publish;
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    float tracked_point_min_z,tracked_point_max_z;


    double tf_tolerance;
    
    tf2::Transform get_transform_map_to_target(const tf2::Transform& map_to_camera, const std::string& target_frame_id);
    void send_transform(tf2::Transform tf,const std::string& src, const std::string& target, ros::Time time=ros::Time::now());
    geometry_msgs::Pose transform_to_pose(const tf2::Transform tf);
    tf2::Stamped<tf2::Transform> lookup_transform(const string& src, const string& target, ros::Time time=ros::Time());
    tf2::Stamped<tf2::Transform> lookup_transform_duration(const string& src, const string& target, ros::Time time=ros::Time::now(), ros::Duration duration = ros::Duration(1.0));
    void config_cb(ORB_SLAM3::orb_slam3Config& config, uint32_t level);
    void point_cloud_thread_run();
};