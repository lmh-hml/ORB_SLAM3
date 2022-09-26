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


#include "../include/ros_rgbd_node.h"

using namespace std;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

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
    //SLAM.Shutdown();

    return 0;
}

