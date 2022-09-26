#include "../include/ros_rgbd_nodelet.h"


RGBDNodelet::RGBDNodelet():Nodelet(),use_viewer(false)
{
}

void RGBDNodelet::onInit()
{
    nh = getMTNodeHandle();
    nh.param<bool>("use_viewer", use_viewer, false);
    nh.param<string>("path_to_vocab",vocabPathStr,"");
    nh.param<string>("path_to_settings",settingsPathStr,"");

    if(vocabPathStr==""||settingsPathStr=="")ROS_WARN("Vocab or settings paths are not set in parameters!");

    orb_slam = make_shared<ORB_SLAM3::System>(vocabPathStr,settingsPathStr,ORB_SLAM3::System::RGBD,use_viewer);
    rgbd = make_unique<RGBD>("RGBD_Nodelet",orb_slam.get(),&nh);
}