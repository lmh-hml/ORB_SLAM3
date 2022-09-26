#include <memory>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include"../../include/System.h"
#include "../include/ros_common.h"

#include "../include/ros_rgbd_node.h"

class RGBDNodelet : public nodelet::Nodelet
{
    public:
        RGBDNodelet();

    protected:

        virtual void onInit(); 

    private:

        std::unique_ptr<RGBD> rgbd;
        std::shared_ptr<ORB_SLAM3::System> orb_slam;
        ros::NodeHandle nh;

        std::string vocabPathStr;
        std::string settingsPathStr;
        bool use_viewer;
};

PLUGINLIB_EXPORT_CLASS(RGBDNodelet, nodelet::Nodelet)

