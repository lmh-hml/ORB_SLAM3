#include "../include/pcl_map_maker.h"

using namespace pcl;



ORB_SLAM3_Mapper::ORB_SLAM3_Mapper():
map_ptr(NULL),occupancy_grid(1000,1000,0.01),grid_pub(NULL),first_map(false),free_thresh(5),height_thresh(10.0),time_thresh(5)
{
};

void ORB_SLAM3_Mapper::set_map(ORB_SLAM3::Map* map)
{
    unique_lock<std::mutex>(map_mutex);
    map_ptr = map;
}

void ORB_SLAM3_Mapper::run()
{
    std::cout << "Map maker started!"<<endl;
    bool finished = false;
    running = true;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr(new pcl::PointCloud<pcl::PointXYZ>), pcl_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setMeanK (20);
    sor.setStddevMulThresh (1.0);

    pcl::PointXYZ last_min, last_max;
    ros::Time last = ros::Time::now();

    while(1)
    {
        if(!check_stop())break;
        map_ptr  = pop_queue();
        if(map_ptr==NULL||map_ptr->IsBad())continue;
        printf("Processing Map: Id %d\n",map_ptr->GetId());

        sensor_msgs::PointCloud2 pcl_msg;
        bool success = map_points_to_point_cloud(map_ptr->GetAllMapPoints(),pcl_msg);
        pcl::fromROSMsg(pcl_msg,*pcl_ptr.get());
        printf("Point cloud size: %d\n",pcl_msg.data.size());

        if(!success)
        {
            std::cout<<"Received no map points"<<std::endl;
        }

        std::vector<ORB_SLAM3::KeyFrame*> keyframes = map_ptr->GetAllKeyFrames();
        if(keyframes.empty())continue;
        sort(keyframes.begin(), keyframes.end(), ORB_SLAM3::KeyFrame::lId);

        ros::Time now = ros::Time::now();
        bool time_thresh_elapsed = ( (now-last).toSec() ) >= time_thresh;
        if(time_thresh_elapsed)last=now;

        ORB_SLAM3::KeyFrame* current_origin_kf = map_ptr->GetOriginKF();
        if(current_origin_kf!=NULL && current_origin_kf->mnId != last_kf_id || time_thresh_elapsed)
        {
            sor.setInputCloud(pcl_ptr);
            sor.filter(*pcl_filtered);
            
            pcl::PointXYZ min, max;
            pcl::getMinMax3D(*pcl_filtered, min, max);
            printf("Poincloud dimensions: max x:%.2f, y:%.2f, min x:%.2f y:%.2f\n",max.x, max.y, min.x, min.y);
        
            occupancy_grid.clear();
            occupancy_grid.resize_image(abs(max.y), abs(min.y),abs(min.x),abs(max.x));
        }

        nav_msgs::OccupancyGrid ocg = occupancy_grid.get_grid();
        size_t halfw = ocg.info.width / 2;
        size_t halfh = ocg.info.height /2 ;

        for(ORB_SLAM3::KeyFrame* kf : keyframes)
        {
            Sophus::SE3f kf_pose = kf->GetPose();
            tf2::Transform camera_tf = orb_slam_pose_to_tf(kf_pose);

            size_t cam_x =  (camera_tf.getOrigin().getX()/ocg.info.resolution) + halfw;
            size_t cam_y =  (camera_tf.getOrigin().getY()/ocg.info.resolution) + halfh;

            std::set<ORB_SLAM3::MapPoint*> map_points = kf->GetMapPoints();
            for(auto mp : map_points)
            {
                geometry_msgs::Pose pose;
                map_point_to_pose_msg(mp,pose);

                if(pose.position.z > height_thresh && pose.position.z<=0.1)continue;

                size_t px =  (pose.position.x/ocg.info.resolution) + halfw;
                size_t py =  (pose.position.y/ocg.info.resolution) + halfh;

                double dist = pow(camera_tf.getOrigin().getX() - pose.position.x, 2) + pow(camera_tf.getOrigin().getY() - pose.position.y,2);
                if(dist > 16.0)continue;
                //printf("On map: %zu, %zu, %zu, %zu\n",px,py,cam_x,cam_y);

                occupancy_grid.cv_mark_occupied(px,py);

                for(auto p:bresenham_line(cam_x,cam_y,px,py))
                {
                    if(p.x == px && p.y == py)continue;
                    occupancy_grid.cv_mark_free(p.x,p.y);
                }
            }
        }

        occupancy_grid.process_free_and_occupied(free_thresh);
        occupancy_grid.cvMat_to_occupancy_grid();
        grid_pub->publish(occupancy_grid.get_grid());
        last_kf_id = current_origin_kf->mnId;

        printf("Ending loop!");
    }
    finished = true;
    std::cout<<"MAPPER EXITING!"<<endl;
}

void ORB_SLAM3_Mapper::stop()
{
    std::unique_lock<std::mutex>(finish_mutex);
    running = false;
}

bool ORB_SLAM3_Mapper::check_stop()
{
    std::unique_lock<std::mutex>(finish_mutex);
    return running;
}

void ORB_SLAM3_Mapper::queue_map(ORB_SLAM3::Map* map)
{
    std::unique_lock<std::mutex>(map_mutex);
    map_queue.push(map);
}

ORB_SLAM3::Map* ORB_SLAM3_Mapper::pop_queue()
{
    std::unique_lock<std::mutex>(map_mutex);
    if(map_queue.empty())return NULL;
    ORB_SLAM3::Map* ret = map_queue.front();
    map_queue.pop();
    return ret;
}