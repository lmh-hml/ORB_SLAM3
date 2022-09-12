#include "../include/pcl_map_maker.h"

using namespace pcl;

typedef pcl::PointCloud<pcl::PointXYZ> Pointcloud;


ORB_SLAM3_Mapper::ORB_SLAM3_Mapper():map_ptr(NULL),occupancy_grid(1000,1000,0.01),grid_pub(NULL),first_map(false)
{};

void ORB_SLAM3_Mapper::set_map(ORB_SLAM3::Map* map)
{
    unique_lock<std::mutex>(map_mutex);
    map_ptr = map;
}

void ORB_SLAM3_Mapper::run()
{
    cout << "Map maker started!"<<endl;
    bool finished = false;
    running = true;

    while(1)
    {
        if(!check_stop())break;
        map_ptr  = pop_queue();
        if(map_ptr==NULL)continue;
        printf("Processing Map: Id %d\n",map_ptr->GetId());

        sensor_msgs::PointCloud2 pcl_msg;
        bool success = map_points_to_point_cloud(map_ptr->GetAllMapPoints(),pcl_msg);
        printf("Point cloud size: %d\n",pcl_msg.data.size());

        if(!success)
        {
            cout<<"Received no map points"<<endl;
        }

        std::vector<ORB_SLAM3::KeyFrame*> keyframes = map_ptr->GetAllKeyFrames();
        sort(keyframes.begin(), keyframes.end(), ORB_SLAM3::KeyFrame::lId);

        size_t current_originKF_id = map_ptr->GetOriginKF()->mnId;


        ORB_SLAM3::KeyFrame* current_origin_kf = map_ptr->GetOriginKF();
        if(current_origin_kf->mnId != last_kf_id)
        {
            auto pose = orb_slam_pose_to_tf(current_origin_kf->GetPose());
            printf("New origin KF: Id %zu, px %.2f, %.2f, R %.2f\n",pose.getOrigin().getX(), pose.getOrigin().getY(), pose.getRotation().getAngle()  );
        
            auto it = find_if(keyframes.begin(), keyframes.end(), [this](const ORB_SLAM3::KeyFrame* obj) {return obj->mnId == last_kf_id;});
            if(it != keyframes.end())
            {
                ORB_SLAM3::KeyFrame* kf = *it;
                auto pose = orb_slam_pose_to_tf(kf->GetPose());
                printf("Old origin KF: Id %zu, px %.2f, %.2f, R %.2f\n",pose.getOrigin().getX(), pose.getOrigin().getY(), pose.getRotation().getAngle()  );
            }

            float max_x = -std::numeric_limits<float>::infinity();
            float max_y = -std::numeric_limits<float>::infinity();
            float min_x = std::numeric_limits<float>::infinity();
            float min_y = std::numeric_limits<float>::infinity();

            for(ORB_SLAM3::MapPoint* mp : map_ptr->GetAllMapPoints())
            {
                tf2::Vector3 point_pos(mp->GetWorldPos()(0), mp->GetWorldPos()(1), mp->GetWorldPos()(2));
                point_pos = tf_orb_to_ros * point_pos;

                if(point_pos.getX() > max_x) max_x = point_pos.getX();
                if(point_pos.getY() > max_y) max_y = point_pos.getY();
                if(point_pos.getX() < min_x) min_x = point_pos.getX();
                if(point_pos.getY() < min_y) min_y = point_pos.getY();
            }

            size_t new_w = ceil(max_x - min_x);
            size_t new_h = ceil(max_y - min_y);

            printf("The map is now x: %zu, y: %zu big \n ", new_w, new_h);
        
            occupancy_grid.clear();
            occupancy_grid.resize_image(max_x-min_x, max_y-min_y);
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

                if(pose.position.z > 10.0)continue;

                size_t px =  (pose.position.x/ocg.info.resolution) + halfw;
                size_t py =  (pose.position.y/ocg.info.resolution) + halfh;

                double dist = pow(camera_tf.getOrigin().getX() - pose.position.x, 2) + pow(camera_tf.getOrigin().getY() - pose.position.y,2);
               // printf("Distance:%.1f\n",dist);
                if(dist > 16.0)continue;
                //printf("On map: %zu, %zu, %zu, %zu\n",px,py,cam_x,cam_y);

                occupancy_grid.cv_mark_occupied(px,py);

                for(auto p:bresenham_line(cam_x,cam_y,px,py))
                {
                    if(p.x == px && p.y == py)continue;
                    occupancy_grid.cv_mark_free(p.x,p.y);
                }
                //occupancy_grid.cv_line(px, py, cam_x, cam_y, 0);
            }
        }

        occupancy_grid.process_free_and_occupied();
        occupancy_grid.cvMat_to_occupancy_grid();
        grid_pub->publish(occupancy_grid.get_grid());


        last_kf_id = current_origin_kf->mnId;

        printf("Ending loop!");

    }
    finished = true;
    cout<<"MAPPER EXITING!"<<endl;
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