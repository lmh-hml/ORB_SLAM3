#include "../include/pcl_map_maker.h"

using namespace pcl;

ORB_SLAM3_Mapper::ORB_SLAM3_Mapper():
map_ptr(NULL),occupancy_grid(1000,1000,0.01),grid_pub(NULL),first_map(false),free_thresh(10),height_thresh(10.0),time_thresh(5),
filter_radius(1.0),filter_min_neighbors(10),min_z(0.0),max_z(10.0),dist_thresh(16.0),gaussian_ksize(3),free_value(1),occupied_value(1)
{};

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

    pcl::StatisticalOutlierRemoval<pcl::PointXYZL> sor;
    pcl::RadiusOutlierRemoval<pcl::PointXYZL> ror;
    pcl::PassThrough<pcl::PointXYZL> passthrough;

    pcl::PointXYZ last_min, last_max;
    ros::Time last = ros::Time::now();

    while(1)
    {
        if(!check_stop())break;
        map_ptr  = pop_queue();
        if(map_ptr==NULL||map_ptr->IsBad())continue;
        //printf("Processing Map: Id %d\n",map_ptr->GetId());

        std::vector<ORB_SLAM3::KeyFrame*> keyframes = map_ptr->GetAllKeyFrames();
        if(keyframes.empty())continue;

        PointcloudXYZL::Ptr pcl_ptr(new PointcloudXYZL);
        bool success = map_points_labeled_to_point_cloud(map_ptr->GetAllMapPoints(),pcl_ptr);

        if(!success)
        {
            std::cout<<"Received no map points"<<std::endl;
            continue;
        }

        ros::Time now = ros::Time::now();
        bool time_thresh_elapsed = ( (now-last).toSec() ) >= time_thresh;
        if(time_thresh_elapsed)last=now;

        std::set<unsigned long> filtered_point_indexes;
        ORB_SLAM3::KeyFrame* current_origin_kf = map_ptr->GetOriginKF();

        occupancy_grid.clear();
        filter_pointcloud(pcl_ptr);
        for(auto p:pcl_ptr->points)
        {
            filtered_point_indexes.insert(p.label);
        }

        size_t halfw,halfh;
        auto ocg = occupancy_grid.get_grid();
        occupancy_grid.get_map_origin(halfw,halfh);
        //printf("Image origin: %zu,%zu\n",halfw,halfh);

        if( (current_origin_kf!=NULL && current_origin_kf->mnId != last_kf_id) || time_thresh_elapsed)
        {            
            pcl::PointXYZL min_p, max_p;
            pcl::getMinMax3D(*pcl_ptr, min_p, max_p);
            printf("Poincloud dimensions(meters): max x:%.2f, y:%.2f, min x:%.2f y:%.2f\n",max_p.x, max_p.y, min_p.x, min_p.y);

            auto grid_max_x = ocg.info.origin.position.x + (ocg.info.width*ocg.info.resolution);
            auto grid_min_x = ocg.info.origin.position.x;
            auto grid_max_y = ocg.info.origin.position.y + (ocg.info.height*ocg.info.resolution);
            auto grid_min_y = ocg.info.origin.position.y;
            printf("Grid dims(meters): Max X:%.2f,Y:%.2f, Min X:%.2f Min Y:%.2f\n",grid_max_x, grid_max_y, grid_min_x,grid_min_y);

            size_t right_diff = max(0.0,max_p.x - grid_max_x)/ocg.info.resolution;
            size_t left_diff = max(0.0, grid_min_x - min_p.x)/ocg.info.resolution;
            size_t up_diff = max(0.0,max_p.y - grid_max_y)/ocg.info.resolution;
            size_t down_diff = max(0.0, grid_min_y - min_p.y)/ocg.info.resolution;
            printf("Offsets: %zu, %zu, %zu, %zu\n",right_diff,up_diff,left_diff,down_diff);

            printf("Current Image dimensions(cells): %.2f, %.2f\n ",ocg.info.width,ocg.info.height);
            occupancy_grid.resize_image(up_diff,down_diff,left_diff,right_diff);
        }

        auto dist_thresh_sq = (dist_thresh*dist_thresh);
        for(ORB_SLAM3::KeyFrame* kf : keyframes)
        {
            //Get the camera's coordinates on the occupancy grid.
            Sophus::SE3f kf_pose = kf->GetPose();
            tf2::Transform camera_tf = orb_slam_pose_to_tf(kf_pose);
            size_t cam_x =  (camera_tf.getOrigin().getX()/ocg.info.resolution) + halfw;
            size_t cam_y =  (camera_tf.getOrigin().getY()/ocg.info.resolution) + halfh;
            occupancy_grid.cv_mark_free(cam_x,cam_y, free_value * 2);

            for(auto mp : kf->GetMapPoints())
            {
                if(mp==nullptr)continue;
                if(!filtered_point_indexes.count(mp->mnId))continue;

                geometry_msgs::Pose pose;
                map_point_to_pose_msg(mp,pose);

                size_t px =  (pose.position.x/ocg.info.resolution) + halfw;
                size_t py =  (pose.position.y/ocg.info.resolution) + halfh;
                //printf("On map: %zu, %zu, %zu, %zu\n",px,py,cam_x,cam_y);

                auto dist = distance_btw_points(cam_x,cam_y,px,py);
                if(dist>dist_thresh_sq)continue;

                occupancy_grid.cv_mark_occupied(px,py,occupied_value);
                for(auto p:bresenham_line(cam_x,cam_y,px,py))
                {
                    if(p.x == px && p.y == py)continue;
                    occupancy_grid.cv_mark_free(p.x,p.y,free_value);
                }
            }
        }
        printf("Free thresh:%d, ksize: %d\n",free_thresh,gaussian_ksize);
        occupancy_grid.process_free_and_occupied(free_thresh,gaussian_ksize);
        occupancy_grid.cvMat_to_occupancy_grid();
        grid_pub->publish(occupancy_grid.get_grid());

        sensor_msgs::PointCloud2 pcl_msg;
        pcl::toROSMsg<pcl::PointXYZL>(*pcl_ptr,pcl_msg);
        pcl_ros::transformPointCloud( tf2::transformToEigen(tf2::toMsg(point_cloud_offset)).matrix().cast<float>(), pcl_msg, pcl_msg );
        pcl_msg.header.frame_id = "map";
        pcl_pub->publish(pcl_msg);

        last_kf_id = current_origin_kf->mnId;
        printf("Ending loop!\n");
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

bool ORB_SLAM3_Mapper::map_points_labeled_to_point_cloud(const std::vector<ORB_SLAM3::MapPoint *> &map_points, PointcloudXYZL::Ptr pcl)
{
    if(map_points.empty())return false;
    pcl->points.clear();
    for (auto mp: map_points)
    { 
        if (mp!=nullptr || !mp->isBad())
        {
            tf2::Vector3 point(mp->GetWorldPos()(0), mp->GetWorldPos()(1), mp->GetWorldPos()(2));
            point = tf_orb_to_ros * point;
            pcl->points.push_back(pcl::PointXYZL());
            pcl->back().x = point.x();
            pcl->back().y = point.y();
            pcl->back().z = point.z();
            pcl->back().label = mp->mnId;
        }
    }
    return true;
}

void ORB_SLAM3_Mapper::filter_pointcloud(PointcloudXYZL::Ptr pcl_ptr)
{
    //printf("Filtering...\n");
    pcl::StatisticalOutlierRemoval<pcl::PointXYZL> sor;
    pcl::RadiusOutlierRemoval<pcl::PointXYZL> ror;
    pcl::PassThrough<pcl::PointXYZL> passthrough;

    //printf("Filter Rad: %.2f, Min Neigh:%d\n",filter_radius, filter_min_neighbors);
    ror.setRadiusSearch(filter_radius);
    ror.setMinNeighborsInRadius(filter_min_neighbors);
    //printf("Min/Max Z: %.2f %.2f\n",min_z, max_z);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(min_z, max_z);

    passthrough.setInputCloud(pcl_ptr);
    passthrough.filter(*pcl_ptr);       
    ror.setInputCloud(pcl_ptr);
    ror.filter(*pcl_ptr);
}



