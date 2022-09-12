#pragma once
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

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

#include <orb_slam3_msgs/keyframe_msg.h>
#include <orb_slam3_msgs/orb_slam_map_msg.h>
#include <orb_slam3_msgs/keyframe_pointcloud.h>

#include <orb_slam3_msgs/MapPoint.h>
#include <orb_slam3_msgs/KeyframeWithPoints.h>
#include <orb_slam3_msgs/KeyframeWithIndices.h>
#include <orb_slam3_msgs/Map.h>




#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include<opencv2/core/core.hpp>

#include "../../include/System.h"

const tf2::Matrix3x3 tf_orb_to_ros(0,  0,  1,
                                -1,  0,  0,
                                0, -1,  0);

bool compare_map_point(ORB_SLAM3::MapPoint* mp1, ORB_SLAM3::MapPoint* mp2)
{
    return mp1->mnId<mp2->mnId;
}

tf2::Transform orb_slam_pose_to_tf(const Sophus::SE3f& position)
{
    tf2::Vector3 camera_translation(position.translation().x(), position.translation().y(), position.translation().z());

    Sophus::Matrix3f mat = position.rotationMatrix();
    tf2::Matrix3x3 camera_rotation( mat(0,0), mat(0,1), mat(0,2), mat(1,0),mat(1,1), mat(1,2), mat(2,0),mat(2,1),mat(2,2));

    camera_translation =  tf_orb_to_ros * camera_translation;
    camera_rotation = tf_orb_to_ros * camera_rotation;

    camera_rotation = camera_rotation.transpose();
    camera_translation = (-(camera_rotation * camera_translation));

    camera_rotation = tf_orb_to_ros * camera_rotation;
    camera_translation = tf_orb_to_ros * camera_translation;

    return tf2::Transform(camera_rotation,camera_translation);
}

tf2::Transform pose_msg_to_tf(const geometry_msgs::Pose& pose)
{
    tf2::Transform tf;
    tf.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
    tf.setRotation(tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
    return tf;
}

void tf2_to_transform_and_pose_msg(const tf2::Transform& tf, geometry_msgs::Transform& transform,  geometry_msgs::Pose& pose  )
{
    tf2::Transform tf_copy = tf;
    tf_copy.setRotation(tf.getRotation().normalize());
    transform = tf2::toMsg(tf_copy);
    pose.position.x =transform.translation.x;
    pose.position.y =transform.translation.y;
    pose.position.z =transform.translation.z;
    pose.orientation = transform.rotation;
}

bool map_points_to_point_cloud(const std::vector<ORB_SLAM3::MapPoint*> map_points, sensor_msgs::PointCloud2& cloud, std::string map_frame_id="map" )
{
    if(map_points.empty()) return false;

    const int num_channels = 3;
    cloud.header.frame_id = "orb_slam3_map";
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {
            tf2::Vector3 point_translation(map_points[i]->GetWorldPos()(0), map_points[i]->GetWorldPos()(1), map_points[i]->GetWorldPos()(2));
            point_translation = tf_orb_to_ros * point_translation;
            float data_array[num_channels] = {point_translation.x(), point_translation.y(), point_translation.z()};
            memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    return true;
}



void map_point_to_pose_msg(ORB_SLAM3::MapPoint* mp, geometry_msgs::Pose& pose)
{
    tf2::Vector3 point_translation(mp->GetWorldPos()(0), mp->GetWorldPos()(1), mp->GetWorldPos()(2));
    point_translation = tf_orb_to_ros * point_translation;
    pose.position.x = point_translation.x();
    pose.position.y = point_translation.y();
    pose.position.z = point_translation.z();
}

tf2::Vector3  map_point_to_tf_vect3(ORB_SLAM3::MapPoint* mp)
{
    tf2::Vector3 point_translation(mp->GetWorldPos()(0), mp->GetWorldPos()(1), mp->GetWorldPos()(2));
    point_translation = tf_orb_to_ros * point_translation;
    return point_translation;
}

void map_points_to_pose_array_msg(std::vector<ORB_SLAM3::MapPoint *> map_points, geometry_msgs::PoseArray& pose_array)
{
    for(auto mp : map_points)
    {
        if(mp)
        {
            geometry_msgs::Pose pose;
            map_point_to_pose_msg(mp,pose);
            pose_array.poses.push_back(pose);
        }
    }
}


void point_cloud_to_pose_array(const sensor_msgs::PointCloud2& pcl,  geometry_msgs::PoseArray& pose_array)
{
    for(sensor_msgs::PointCloud2ConstIterator<float> pcl_iter(pcl, "x"); pcl_iter != pcl_iter.end();++pcl_iter)
    {      
        geometry_msgs::Pose pose;
        pose.position.x = pcl_iter[0];
        pose.position.y = pcl_iter[1];
        pose.position.z = pcl_iter[2];
        pose_array.poses.push_back(pose);      
    }
}

void getMinMax(const geometry_msgs::PoseArray& pts_and_pose,
    geometry_msgs::Point& min_pt, geometry_msgs::Point& max_pt) {

    min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<double>::infinity();
    max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<double>::infinity();
    for (unsigned int i = 0; i < pts_and_pose.poses.size(); ++i){
        const geometry_msgs::Point& curr_pt = pts_and_pose.poses[i].position;
        //ROS_INFO("Point xyz: %f, %f, %f",curr_pt.x,curr_pt.y,curr_pt.z);
        if (curr_pt.x < min_pt.x) { min_pt.x = curr_pt.x; }
        if (curr_pt.y < min_pt.y) { min_pt.y = curr_pt.y; }
        if (curr_pt.z < min_pt.z) { min_pt.z = curr_pt.z; }

        if (curr_pt.x > max_pt.x) { max_pt.x = curr_pt.x; }
        if (curr_pt.y > max_pt.y) { max_pt.y = curr_pt.y; }
        if (curr_pt.z > max_pt.z) { max_pt.z = curr_pt.z; }
    }
}

void bresenham_line_low(int x1 , int y1, int x2, int y2, std::vector<cv::Point2d>& line)
{
    int dx = x2-x1;
    int dy = y2-y1;
    int D = 2*dy-dx;
    int y = y1;

    int yi=1;

    if(dy<0)
    {
        yi = -1;
        dy = -dy;
    }
    assert(dx>=0);

    for(int x = x1; x != x2 ; x++)
    {
        line.push_back(cv::Point2d(x,y));
        if(D>0)
        {
            y+=yi;
            D = D + 2*(dy-dx);
        }
        else
        {
            D = D + 2*dy;
        }
    }
}

void bresenham_line_steep(int x1 , int y1, int x2, int y2, std::vector<cv::Point2d>& line)
{
    int dx = x2-x1;
    int dy = y2-y1;
    int D = 2*dx-dy;
    int x = x1;

    int xi=1;

    if(dx<0)
    {
        xi = -1;
        dx = -dx;
    }
    assert(dy>=0);


    for(int y = y1; y != y2 ; y++)
    {
        line.push_back(cv::Point2d(x,y));
        if(D>0)
        {
            x+=xi;
            D = D + 2*(dx-dy);
        }
        else
        {
            D = D + 2*dx;
        }
    }
}

std::vector<cv::Point2d> bresenham_line(int x1, int y1, int x2, int y2)
{
    std::vector<cv::Point2d> line;
    int dy = y2-y1;
    int dx = x2-x1;
    //True if the line is steep
    if( abs(dy) > abs(dx) )
    {
        if(y1>y2) //check if 
        {
            bresenham_line_steep(x2,y2,x1,y1,line);
            std::reverse(line.begin(),line.end());
        }
        else
        {
            bresenham_line_steep(x1,y1,x2,y2,line);
        }
    }
    else
    {
        //plot shallow
        if(x1>x2) //check if 
        {
            bresenham_line_low(x2,y2,x1,y1,line);
            std::reverse(line.begin(),line.end());
        }
        else
        {
            bresenham_line_low(x1,y1,x2,y2,line);
        }
    }
    return line;
}

void keyframe_to_msg(ORB_SLAM3::KeyFrame* keyframe, orb_slam3_msgs::keyframe_msg& msg, bool mp_id_only=false)
{
    geometry_msgs::Transform tf_msg;
    geometry_msgs::Pose kf_pose;
    geometry_msgs::PoseArray map_points_msg;
    auto map_points = keyframe->GetMapPoints();
    tf2_to_transform_and_pose_msg(orb_slam_pose_to_tf(keyframe->GetPose()),tf_msg,kf_pose);
    std::vector<ORB_SLAM3::MapPoint*> mp_vec(map_points.begin(),map_points.end());

    if(!mp_id_only)
    {
        map_points_to_pose_array_msg(mp_vec,map_points_msg);
    }

    for(auto mp: mp_vec)
    {
        msg.map_points_ids.push_back(mp->mnId);
    }

    msg.map_points = map_points_msg;
    msg.pose = kf_pose;
    msg.id = keyframe->mnId;
}

void keyframe_to_keyframe_pointcloud_msg(ORB_SLAM3::KeyFrame* keyframe,  orb_slam3_msgs::keyframe_pointcloud& kf_pcl)
{
    auto map_points = keyframe->GetMapPoints();
    geometry_msgs::Transform tf;
    kf_pcl.map_id = keyframe->GetMap()->GetId();
    kf_pcl.keyframe_id = keyframe->mnId;
    tf2_to_transform_and_pose_msg(orb_slam_pose_to_tf(keyframe->GetPose()),tf, kf_pcl.pose);
    map_points_to_point_cloud( vector<ORB_SLAM3::MapPoint*>(map_points.begin(), map_points.end()), kf_pcl.pointcloud,kf_pcl.header.frame_id);
}

void orb_slam_map_to_msg(ORB_SLAM3::Map* map, orb_slam3_msgs::orb_slam_map_msg& msg)
{
    geometry_msgs::PoseArray pose_array;
    {
        std::vector<ORB_SLAM3::KeyFrame *> keyframes;
        //Set map id
        msg.id = map->GetId();
        msg.init_keyframe_id = map->GetInitKFid();
        //Set up keyframes
        keyframes = map->GetAllKeyFrames();
        
        sort(keyframes.begin(), keyframes.end(), ORB_SLAM3::KeyFrame::lId);

        for(ORB_SLAM3::KeyFrame* kf : keyframes)
        {
            orb_slam3_msgs::keyframe_msg kf_msg;
            keyframe_to_msg(kf, kf_msg, true);
            msg.keyframes.push_back(kf_msg);
        }

        std::vector<ORB_SLAM3::MapPoint*>mp_vec = map->GetAllMapPoints();
        sort(mp_vec.begin(),mp_vec.end(),compare_map_point);
        map_points_to_pose_array_msg(mp_vec,pose_array);
    }   
    
    msg.map_points = pose_array;

    geometry_msgs::Point min, max;
    getMinMax(msg.map_points,min,max);
    msg.max_point = max;
    msg.min_point = min;
}

void print_tf(geometry_msgs::TransformStamped tf)
{
    double roll, pitch, yaw;
    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation,q);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ROS_INFO("Static transform Map to Target [%s -> %s] at %f",
                    tf.header.frame_id.c_str(), tf.child_frame_id.c_str(), tf.header.stamp.toSec());
    ROS_INFO(" * Translation: {%.3f,%.3f,%.3f}",
                    tf.transform.translation.x, tf.transform.translation.y,tf.transform.translation.z);
    ROS_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
                    RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
}

void print_tf(geometry_msgs::Transform tf)
{
    double roll, pitch, yaw;
    tf2::Quaternion q;
    tf2::fromMsg(tf.rotation,q);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ROS_INFO(" * Translation: {%.3f,%.3f,%.3f}",
                    tf.translation.x, tf.translation.y,tf.translation.z);
    ROS_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
                    RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
}

void map_point_to_msg(ORB_SLAM3::MapPoint* map_point, orb_slam3_msgs::MapPoint& msg)
{
    msg.id = map_point->mnId;
    Eigen::Vector3f v3f = map_point->GetWorldPos();
    tf2::Vector3 point_translation(map_point->GetWorldPos()(0), map_point->GetWorldPos()(1), map_point->GetWorldPos()(2));
    point_translation = tf_orb_to_ros * point_translation;
    msg.position.x = point_translation.x();
    msg.position.y = point_translation.y();
    msg.position.z = point_translation.z();
}
