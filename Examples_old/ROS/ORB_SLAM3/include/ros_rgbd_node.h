#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <signal.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../include/System.h"
#include "../include/ros_common.h"

using namespace std;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;


class RGBD: public ORB_ROS_Node
{
    public:

        RGBD(std::string name,ORB_SLAM3::System::eSensor sensor, ros::NodeHandle* nh ):
        ORB_ROS_Node(name, sensor, nh)
        {
            rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(*nh, "/camera/rgb/image_raw", 10);
            depth_sub = new message_filters::Subscriber<sensor_msgs::Image>(*nh, "/camera/depth_registered/image_raw", 10);
            sync = new  message_filters::Synchronizer<sync_pol>(sync_pol(100), *rgb_sub,*depth_sub);
            sync->registerCallback(boost::bind(&RGBD::GrabRGBD,this,_1,_2));
        };
        ~RGBD()
        {
            delete sync;
            delete depth_sub;
            delete rgb_sub;
        };

    private:
        message_filters::Subscriber<sensor_msgs::Image>* rgb_sub;
        message_filters::Subscriber<sensor_msgs::Image>* depth_sub;
        message_filters::Synchronizer<sync_pol>* sync ;

        void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
        {   
            //auto start = std::chrono::high_resolution_clock::now();
            // Copy the ros image message to cv::Mat.
            cv_bridge::CvImageConstPtr cv_ptrRGB;
            try
            {
                cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            cv_bridge::CvImageConstPtr cv_ptrD;
            try
            {
                cv_ptrD = cv_bridge::toCvShare(msgD);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            Sophus::SE3f position = orb_system->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
            update(position, cv_ptrRGB->header.stamp);
        }
};
