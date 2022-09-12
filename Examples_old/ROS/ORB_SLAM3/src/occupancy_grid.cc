#include "../include/OccupancyGrid.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_maker");
    ros::start();

    ros::NodeHandle nh;
    OccupancyGrid ocg(500,500,0.01);
    ocg.cv_line(0,0,400,400,100);
    ocg.cvMat_to_occupancy_grid();

    ros::Publisher ocg_pub = nh.advertise<nav_msgs::OccupancyGrid>("ocg",5);
    ros::Rate r(5); // 5 hz
    while(!ros::isShuttingDown())
    {
        ocg_pub.publish(ocg.get_grid());
        ros::spinOnce();
        r.sleep();
    }

    ros::shutdown();

    return 0;
}