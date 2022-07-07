#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <orb_slam3_msgs/keyframe_msg.h>
#include <orb_slam3_msgs/keyframe_pointcloud.h>
#include "utility.h"


#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/transforms.h>

#include<opencv2/core/core.hpp>

#include "../../include/System.h"

class ORB_ROS_Node
{
    public:
    ORB_ROS_Node(std::string name, ORB_SLAM3::System* system, ros::NodeHandle* nh )
    {
        node_handle = nh;
        image_transport = new image_transport::ImageTransport(*node_handle);
        tfListener = new tf2_ros::TransformListener(tfBuffer);
        node_name = name;
        initial_kf_id = -1;
        last_kf_size = 0;
        init_ORB_SLAM(system);
        init_ros();

    };

    void init_ros()
    {
        ROS_INFO("INIT ROS!!");
        lastest_kf_map_points_pub = node_handle->advertise<sensor_msgs::PointCloud2>(node_name+"/kf_map_points",5);
        global_map_points_pub = node_handle->advertise<sensor_msgs::PointCloud2>(node_name+"/global_map_points",5);
        current_pose_pub = node_handle->advertise<geometry_msgs::PoseStamped>(node_name+"/pose",5);
        kf_pose_array_pub = node_handle->advertise<geometry_msgs::PoseArray>(node_name+"/kf_pose_array",5);
        initial_kf_pose_pub = node_handle->advertise<geometry_msgs::PoseStamped>(node_name+"/initial_kf_pose",5);
        orb_slam_goal_pub = node_handle->advertise<geometry_msgs::PoseStamped>(node_name+"/goal",5);
        orb_slam_map_pub = node_handle->advertise<orb_slam3_msgs::orb_slam_map_msg>(node_name+"/orb_slam3_map_msg",5);
        image_pub = image_transport->advertise(node_name+"/debug_image",10);
        kf_pcl_pub = node_handle->advertise<orb_slam3_msgs::keyframe_pointcloud>(node_name+"/keyframe_pointcloud",5);

        node_handle->param<std::string>("map_frame_id", map_frame_id, "map");
        node_handle->param<std::string>("orb_slam3_frame_id", orb_slam3_map_frame_id, "orb_slam3_map");
        node_handle->param<std::string>("camera_frame_id", camera_frame_id, "camera_link");
        node_handle->param<std::string>("target_frame_id", target_frame_id, "orb_pose");


        tf2_base_link_to_camera = lookup_transform_duration("base_footprint",camera_frame_id, ros::Time(), ros::Duration(3.0));

    }

    void init_ORB_SLAM(ORB_SLAM3::System* system)
    {
            orb_system =  system;
    }

    void publish_current_tracked_points(ros::Time frame_time)
    {
        std::vector<ORB_SLAM3::MapPoint*> map_points = orb_system->GetTrackedMapPoints();
        sensor_msgs::PointCloud2 cloud;
        cloud.header.stamp = frame_time;
        map_points_to_point_cloud(map_points, cloud);
        lastest_kf_map_points_pub.publish(cloud);
    }

    void publish_current_transform_and_pose(ros::Time frame_time)
    {
        //Publish current camera position as transform and pose messages.
        geometry_msgs::TransformStamped transform_msg;
        geometry_msgs::PoseStamped pose_msg;
        tf2_to_transform_and_pose_msg(tf2_orb_slam3_map_to_camera, transform_msg.transform, pose_msg.pose);
        pose_msg.header.frame_id = transform_msg.header.frame_id = orb_slam3_map_frame_id;
        pose_msg.header.stamp = transform_msg.header.stamp = frame_time;
        current_pose_pub.publish(pose_msg);

        tf2::Transform cam_to_odom = lookup_transform("camera_link","odom",frame_time);
        tf2::Transform orb_slam3_map_to_odom =  tf2_orb_slam3_map_to_camera * cam_to_odom;
        send_transform(tf2_base_link_to_camera,"map",orb_slam3_map_frame_id,frame_time);
        send_transform(orb_slam3_map_to_odom, orb_slam3_map_frame_id, "odom", frame_time);
    }

    void publish_current_global_map_points(ros::Time frame_time)
    {
        ORB_SLAM3::Atlas* atlas = orb_system->GetAtlas();
            
        std::vector<ORB_SLAM3::MapPoint*> map_points = atlas->GetCurrentMap()->GetAllMapPoints();
    
        sensor_msgs::PointCloud2 cloud, cloud_out;
        cloud.header.stamp = frame_time;
        cloud.header.frame_id = orb_slam3_map_frame_id;
        map_points_to_point_cloud(map_points, cloud);

        //Transform the map points from orb_slam3's frame to the map frame
        auto eigen = tf2::transformToEigen(tf2::toMsg(tf2_base_link_to_camera));
        pcl_ros::transformPointCloud( eigen.matrix().cast<float>(), cloud, cloud_out );
        cloud_out.header.stamp = frame_time;
        cloud_out.header.frame_id = "map";

        global_map_points_pub.publish(cloud_out);
    }

    void publish_orb_slam_debug_image(ros::Time frame_time)
    {
        cv::Mat img = orb_system->GetDebugImg()->DrawFrame();
        std_msgs::Header header;
        header.stamp = frame_time;
        header.frame_id = camera_frame_id;
        const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
        image_pub.publish(rendered_image_msg);
    }

    void publish_all_keyframe_pose_and_points(ros::Time frame_time)
    {
        ORB_SLAM3::Map* map = orb_system->GetAtlas()->GetCurrentMap();

        std::vector<ORB_SLAM3::KeyFrame*> keyframes = map->GetAllKeyFrames();
        sort(keyframes.begin(), keyframes.end(), ORB_SLAM3::KeyFrame::lId);
    
        for(ORB_SLAM3::KeyFrame* kf : keyframes)
        {
            geometry_msgs::PoseArray keyframe_points_and_poses;
            keyframe_points_and_poses.header.frame_id = "map";

             //Publish Keyframe map points
            auto map_points = kf->GetMapPoints();
            std::vector<ORB_SLAM3::MapPoint*> map_points_vect(map_points.begin(), map_points.end());
            sensor_msgs::PointCloud2 pcl;
            pcl.header.stamp = frame_time;
            std::vector<ORB_SLAM3::MapPoint*> vec(map_points.begin(), map_points.end());
            map_points_to_pose_array_msg(vec, keyframe_points_and_poses);

            //Publish Keyframe camera pose
            Sophus::SE3f kf_pose = kf->GetPose();
            geometry_msgs::Pose pose;
            geometry_msgs::Transform tf;
            tf2_to_transform_and_pose_msg(orb_slam_pose_to_tf(kf_pose), tf, pose);
            keyframe_points_and_poses.poses.push_back(pose);

            //Publish
            kf_pose_array_pub.publish(keyframe_points_and_poses);
        }

    }

    void publish_latest_keyframe_pose_and_points(ros::Time frame_time)
    {
        ORB_SLAM3::Map* map = orb_system->GetAtlas()->GetCurrentMap();

        std::vector<ORB_SLAM3::KeyFrame*> keyframes = map->GetAllKeyFrames();
        sort(keyframes.begin(), keyframes.end(), ORB_SLAM3::KeyFrame::lId);

        //The LAST pose of this array must be the pose of the keyframe.
        geometry_msgs::PoseArray keyframe_points_and_poses;
        keyframe_points_and_poses.header.frame_id = "map";

        if(map_id_changed)return;
        
        //Since ORB_SLAM cull keyframes, it is possible that there are LESS keyframes this frame than previous frames
        size_t current_size = keyframes.size();
        long difference = current_size - last_kf_size;
        printf("Current/Last kf size: %d/%d\n", current_size,last_kf_size);
        last_kf_size = current_size;

        if(difference<0)return;

        for( size_t i = difference; i!= 0; i--)
        {
            auto last_kf = keyframes.at(keyframes.size()-i);
            orb_slam3_msgs::keyframe_pointcloud kf_pcl;
            kf_pcl.header.frame_id = orb_slam3_map_frame_id;
            kf_pcl.header.stamp = current_frame_time;

            keyframe_to_keyframe_pointcloud_msg(last_kf, kf_pcl);
            kf_pcl_pub.publish(kf_pcl);
        
            // //Publish Keyframe map points
            // auto map_points = last_kf->GetMapPoints();
            // std::vector<ORB_SLAM3::MapPoint*> map_points_vect(map_points.begin(), map_points.end());
            // sensor_msgs::PointCloud2 pcl;
            // pcl.header.stamp = frame_time;
            // map_points_to_point_cloud(map_points_vect,pcl);
            // point_cloud_to_pose_array(pcl, keyframe_points_and_poses);

            // //Publish Keyframe camera pose
            // Sophus::SE3f kf_pose = last_kf->GetPose();
            // geometry_msgs::Pose pose;
            // geometry_msgs::Transform tf;
            // tf2_to_transform_and_pose_msg(orb_slam_pose_to_tf(kf_pose), tf, pose);
            // keyframe_points_and_poses.poses.push_back(pose);

            // //Publish
            // kf_pose_array_pub.publish(keyframe_points_and_poses);
            // lastest_kf_map_points_pub.publish(pcl);
        }
    }
    
    void check_atlas_status()
    {

        //Checks if:
        //Current map has changes since last frame
        //Number of maps in atlas has changed
        //Current map's id has changed

        bool map_changed = orb_system->MapChanged();
        if(map_changed)ROS_INFO("Map changed!");

        ORB_SLAM3::Atlas* atlas = orb_system->GetAtlas();
        size_t current_map_count = atlas->GetAllMaps().size();
        if(current_map_count!=last_map_count)
        {
            ROS_INFO("Number of maps has changed from %zu to %zu", last_map_count, current_map_count);
            last_map_count = current_map_count;
            map_count_changed = true;
        }
        else
        {
            map_count_changed = false;
        }


        size_t current_map_id = 0;
        ORB_SLAM3::Map* current_map = atlas->GetCurrentMap();
        {
            unique_lock<std::mutex> lock(current_map->mMutexMapUpdate);
            current_map_id = current_map->GetId();
        }
        if(current_map_id!=last_map_id)
        {
            ROS_INFO("Map id changed to %zu!",current_map_id);
            map_id_changed = true;
            last_map_id = current_map_id;
        }
        else
        {
            map_id_changed = false;
        }

        if( map_changed || map_id_changed || map_count_changed)
        {
            ROS_INFO("Publishing map data");
            orb_slam3_msgs::orb_slam_map_msg map_msg;
            orb_slam_map_to_msg(current_map,map_msg);
            orb_slam_map_pub.publish(map_msg);
        }
    }
    
    void update_current_position( Sophus::SE3f position)
    {
        int tracking_state = orb_system->GetTrackingState();
        switch(tracking_state)
        {
            case 2: //Only when system is ok
            {
                tf2_orb_slam3_map_to_camera = orb_slam_pose_to_tf(position);
            }break;

            //enums from Tracking.h
            case -1: //System not ready
            case 0: //No images
            case 1: //Not initialized
            case 3: //Recently lost
            case 4: //Lost
            default:
            {
                ROS_INFO("Unable to track current frame !! %d", tracking_state);
                tf2_orb_slam3_map_to_camera.setIdentity();
                tf2_map_to_target.setIdentity();
                return;
            }break;
        }
    }

    void update(Sophus::SE3f position, ros::Time current_frame_time)
    {
            auto atlas = orb_system->GetAtlas();
            auto map = atlas->GetCurrentMap();
            auto map_points= map->GetAllMapPoints();
            auto kfs = map->GetAllKeyFrames();

            check_atlas_status();
            update_current_position(position);
            publish_current_global_map_points(current_frame_time);
            publish_latest_keyframe_pose_and_points(current_frame_time);
            publish_current_transform_and_pose(current_frame_time);
            publish_orb_slam_debug_image(current_frame_time);
    }

    protected:
    ORB_SLAM3::System* orb_system;
    ros::NodeHandle* node_handle;

    private:
    std::string node_name;
    std::string camera_frame_id;
    std::string map_frame_id;
    std::string target_frame_id;
    std::string orb_slam3_map_frame_id;
    ros::Time current_frame_time;

    ros::Publisher lastest_kf_map_points_pub;
    ros::Publisher global_map_points_pub;
    ros::Publisher current_pose_pub;
    ros::Publisher kf_pose_array_pub;
    ros::Publisher initial_kf_pose_pub;
    ros::Publisher orb_slam_goal_pub;
    ros::Publisher orb_slam_map_pub;
    ros::Publisher kf_pcl_pub;
    ros::Subscriber goal_sub;

    image_transport::ImageTransport* image_transport;
    image_transport::Publisher image_pub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    geometry_msgs::PoseStamped received_goal;
    geometry_msgs::Transform   camera_to_goal;
    geometry_msgs::Transform   map_to_camera_msg;

    //Transform from 2D map to target frame
    tf2::Transform tf2_map_to_target;

    //Looked up upon node initialization. Used to determine the tf from 2D map to orb_slam3's map origin
    tf2::Transform tf2_base_link_to_camera;

    //Camera position from orb_slam3. Updates every time an image is processed.
    tf2::Transform tf2_orb_slam3_map_to_camera;

    bool goal_valid;

    //Used to track map and atlas changes
    size_t initial_kf_id;
    size_t last_kf_size;
    size_t last_map_id;
    size_t last_map_count;
    bool map_id_changed, map_count_changed;


    tf2::Transform get_transform_map_to_target(const tf2::Transform& map_to_camera, const std::string& target_frame_id)
    {
        tf2::Transform map_to_target;        
        tf2::Stamped<tf2::Transform> camera_to_target = lookup_transform(camera_frame_id, target_frame_id,ros::Time(0));
        map_to_target = map_to_camera * camera_to_target;
        return map_to_target;
    }
    


    void send_transform(tf2::Transform tf,const std::string& src, const std::string& target, ros::Time time=ros::Time::now())
    {
        geometry_msgs::TransformStamped msg;
        msg.header.frame_id = src;
        msg.child_frame_id = target;
        msg.header.stamp = time;
        msg.transform = tf2::toMsg(tf);
        tf_broadcaster.sendTransform(msg);
    }

    geometry_msgs::Pose transform_to_pose(const tf2::Transform tf)
    {
        geometry_msgs::Pose pose;
        pose.position.x = tf.getOrigin().getX();
        pose.position.y = tf.getOrigin().getY();
        pose.position.z = tf.getOrigin().getZ();
        pose.orientation = tf2::toMsg(tf.getRotation());    
        return pose;
    }

    tf2::Stamped<tf2::Transform> lookup_transform(const string& src, const string& target, ros::Time time=ros::Time())
    {
        tf2::Stamped<tf2::Transform> tf;
        try {
            // Get the transform from camera to target
            geometry_msgs::TransformStamped tf_msg = tfBuffer.lookupTransform( src,target, time);
            // Convert to tf2
            tf2::fromMsg(tf_msg, tf);
        } catch (tf2::TransformException &ex) {
            tf.setIdentity();
        }
        return tf;
    }

    tf2::Stamped<tf2::Transform> lookup_transform_duration(const string& src, const string& target, ros::Time time=ros::Time::now(), ros::Duration duration = ros::Duration(1.0))
    {
        ROS_INFO("Looking up tf from %s to %s",src.c_str(), target.c_str());
        tf2::Stamped<tf2::Transform> tf;
        try {
            // Get the transform from camera to target
            geometry_msgs::TransformStamped tf_msg = tfBuffer.lookupTransform( src,target, time,duration);
            // Convert to tf2
            tf2::fromMsg(tf_msg, tf);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            tf.setIdentity();
        }
        return tf;
    }


};