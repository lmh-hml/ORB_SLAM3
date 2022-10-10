#include "../include/ros_rgbd_node.h"

using namespace std;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

std::unique_ptr<RGBD> rgbd;

void shutdown(int sig)
{
    rgbd->shutdown();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ROS_INFO("ORB_SLAM NODE!...");
    ros::init(argc, argv, "RGBD",ros::init_options::NoSigintHandler);
    ros::start();

    ros::NodeHandle nh;
    bool use_viewer = false;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    rgbd = make_unique<RGBD>("ORB_SLAM3_RGBD", ORB_SLAM3::System::RGBD, &nh);
    signal(SIGINT, shutdown);
    ros::spin();
    printf("Closed ORB_SLAM 3 Node...");

    return 0;
}

