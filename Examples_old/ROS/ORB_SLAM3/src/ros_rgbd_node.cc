/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


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

        RGBD(std::string name, ORB_SLAM3::System* system, ros::NodeHandle* nh ):
        ORB_ROS_Node(name, system, nh)
        {
            rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(*nh, "/camera/rgb/image_raw", 10);
            depth_sub = new message_filters::Subscriber<sensor_msgs::Image>(*nh, "camera/depth_registered/image_raw", 10);
            sync = new  message_filters::Synchronizer<sync_pol>(sync_pol(100), *rgb_sub,*depth_sub);
            sync->registerCallback(boost::bind(&RGBD::GrabRGBD,this,_1,_2));
        };

    private:
        message_filters::Subscriber<sensor_msgs::Image>* rgb_sub;
        message_filters::Subscriber<sensor_msgs::Image>* depth_sub;
        message_filters::Synchronizer<sync_pol>* sync ;

        void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
        {
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

ORB_SLAM3::System* orb_system;
RGBD* rgbd;


void sigint_handler(int sig)
{
    ROS_INFO("SHUTTING DOWN ORB SLAM");
    rgbd->stop();
    orb_system->Shutdown();
    //orb_system->SaveAtlas(ORB_SLAM3::System::FileType::BINARY_FILE);
    orb_system->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    std::cout<<"Finished SHUTTING DOWN ORB SLAM"<<std::endl;
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ROS_INFO("ORB_SLAM NODE!...");
    ros::init(argc, argv, "RGBD");
    ros::start();


    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    ros::NodeHandle nh;
    bool use_viewer = false;
    nh.param<bool>("use_viewer", use_viewer, false);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,use_viewer);
    orb_system = &SLAM;
    signal(SIGINT,sigint_handler);

    rgbd = new RGBD("ORB_SLAM3_RGBD", &SLAM, &nh);

    ros::spin();
    // Stop all threads
    //SLAM.Shutdown();

    return 0;
}

