#include "../include/ros_common.h"


ORB_ROS_Node::ORB_ROS_Node(std::string name, ORB_SLAM3::System* system, ros::NodeHandle* nh ):
map_changed(false),map_id_changed(false),map_count_changed(false),use_mapper(false),publish_pointcloud(false),
publish_image(false)
{
    node_handle = nh;
    image_transport = new image_transport::ImageTransport(*node_handle);
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    node_name = name;
    initial_kf_id = -1;
    last_kf_size = 0;
    init_ros();
    init_ORB_SLAM(system);

};

void ORB_ROS_Node::init_ros()
{
    ROS_INFO("INIT ROS!!");
    global_map_points_pub = node_handle->advertise<sensor_msgs::PointCloud2>(node_name+"/global_map_points",5);
    current_pose_pub = node_handle->advertise<geometry_msgs::PoseStamped>(node_name+"/pose",5);
    image_pub = image_transport->advertise(node_name+"/debug_image",10);
    tracking_state_pub = node_handle->advertise<std_msgs::Int16>(node_name+"/tracking_state",5);
    grid_pub = node_handle->advertise<nav_msgs::OccupancyGrid>(node_name+"/occupancy_grid",5);

    save_map_sub = node_handle->subscribe<std_msgs::Bool>(node_name+"/save_map",1, &ORB_ROS_Node::save_map_cb, this);

    node_handle->param<std::string>("map_frame_id", map_frame_id, "map");
    node_handle->param<std::string>("orb_slam3_frame_id", orb_slam3_map_frame_id, "orb_slam3_map");
    node_handle->param<std::string>("camera_frame_id", camera_frame_id, "camera_link");
    node_handle->param<std::string>("target_frame_id", target_frame_id, "orb_pose");
    node_handle->param<std::string>("base_link_id", base_link_id, "base_footprint");
    node_handle->param<std::string>("odom_frame_id", odom_frame_id, "odom");
    node_handle->param<bool>("use_mapper", use_mapper, false);
    node_handle->param<bool>("publish_pointcloud", publish_pointcloud,false);
    node_handle->param<bool>("publish_image", publish_image,false);

    reconfig_callback = boost::bind(&ORB_ROS_Node::config_cb, this, _1, _2);
    server.setCallback(reconfig_callback);

    tf2_base_link_to_camera_origin = lookup_transform_duration(base_link_id,camera_frame_id, ros::Time(), ros::Duration(3.0));
}

void ORB_ROS_Node::init_ORB_SLAM(ORB_SLAM3::System* system)
{
        orb_system =  system;
        if(use_mapper)
        {
            cout<<"RUNNING MAPPER THREAD!!!!!!!!!!!!!!!!!!!!!"<<endl;
            mapper.set_publisher(&grid_pub);
            mapper_thread = new std::thread(&ORB_SLAM3_Mapper::run,&mapper);
        }
}

void ORB_ROS_Node::stop()
{
    printf("Stopping!!!!!!!!!!!!!!!!!!!!!!");
    mapper.stop();
}

void ORB_ROS_Node::publish_current_transform(ros::Time frame_time)
{
    tf2::Transform base_to_odom = lookup_transform(base_link_id, odom_frame_id,frame_time);
    tf2::Transform map_to_odom = tf2_orb_slam3_map_to_camera * base_to_odom;
    send_transform(map_to_odom,"map","odom",ros::Time::now());
}

void ORB_ROS_Node::publish_current_global_map_points(ros::Time frame_time, const std::vector<ORB_SLAM3::MapPoint*>& map_points)
{
    sensor_msgs::PointCloud2 cloud, cloud_out;
    cloud.header.stamp = frame_time;
    cloud.header.frame_id = orb_slam3_map_frame_id;
    map_points_to_point_cloud(map_points, cloud);

    //Transform the map points from orb_slam3's frame to the map frame
    auto eigen = tf2::transformToEigen(tf2::toMsg(tf2_base_link_to_camera_origin));
    pcl_ros::transformPointCloud( eigen.matrix().cast<float>(), cloud, cloud_out );
    cloud_out.header.stamp = frame_time;
    cloud_out.header.frame_id = "map";
    global_map_points_pub.publish(cloud_out);

}

void ORB_ROS_Node::publish_orb_slam_debug_image(ros::Time frame_time)
{
    cv::Mat img = orb_system->GetDebugImg()->DrawFrame();
    std_msgs::Header header;
    header.stamp = frame_time;
    header.frame_id = camera_frame_id;
    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
    image_pub.publish(rendered_image_msg);
}

void ORB_ROS_Node::publish_tracking_state(ros::Time frame_time, int tracking_state)
{
    std_msgs::Int16 msg;
    msg.data = tracking_state;
    tracking_state_pub.publish(msg);
}

void ORB_ROS_Node::check_atlas_status(ORB_SLAM3::Atlas* atlas, ORB_SLAM3::Map* current_map)
{
    static int last_map_id = -1;
    static size_t last_map_count = 0;

    map_changed = orb_system->MapChanged();
    if(map_changed)ROS_INFO("Map changed!");

    size_t current_map_count = atlas->CountMaps();
    map_count_changed = (current_map_count!=last_map_count);
    if(map_count_changed)ROS_INFO("Map count changed from %zu to %zu", last_map_count, current_map_count);
    last_map_count = current_map_count;

    size_t current_map_id = 0;
    current_map_id = current_map->GetId();
    map_id_changed = (current_map_id!=last_map_id);
    if(map_id_changed)ROS_INFO("Map id changed from %d to %zu", last_map_id, current_map_id);
    last_map_id = current_map_id;

}

int ORB_ROS_Node::update_current_position( Sophus::SE3f position)
{
    int tracking_state = orb_system->GetTrackingState();

    switch(tracking_state)
    {
        case 2: //Only when system is ok
        {
            tf2_orb_slam3_map_to_camera = orb_slam_pose_to_tf(position);
        }break;

        //enums from Tracking.h
        case -1: //System not ready
        case 0: //No images
        case 1: //Not initialized
        case 3: //Recently lost
        case 4: //Lost
        default:
        {
            ROS_INFO("Unable to track current frame !! %d", tracking_state);
        }break;
    }
    return tracking_state;
}

void ORB_ROS_Node::update(Sophus::SE3f position, ros::Time current_frame_time)
{
        ORB_SLAM3::Atlas* atlas = orb_system->GetAtlas();
        ORB_SLAM3::Map* map = atlas->GetCurrentMap();

        check_atlas_status(atlas,map);
        int tracking_state = update_current_position(position);

        if( use_mapper && map!=NULL && !map->IsBad())
            mapper.queue_map(map);
        
        if(tracking_state==2)
        {
            publish_current_transform(current_frame_time);
        }

        if(publish_pointcloud)
        {
            std::vector<ORB_SLAM3::MapPoint*> map_points= map->GetAllMapPoints();
            publish_current_global_map_points(current_frame_time,map_points);
        }
        if(publish_image)publish_orb_slam_debug_image(current_frame_time);
        publish_tracking_state(current_frame_time,tracking_state);
}

tf2::Transform ORB_ROS_Node::get_transform_map_to_target(const tf2::Transform& map_to_camera, const std::string& target_frame_id)
{
    tf2::Transform map_to_target;        
    tf2::Stamped<tf2::Transform> camera_to_target = lookup_transform(camera_frame_id, target_frame_id,ros::Time(0));
    map_to_target = map_to_camera * camera_to_target;
    return map_to_target;
}

void ORB_ROS_Node::send_transform(tf2::Transform tf,const std::string& src, const std::string& target, ros::Time time)
{
    geometry_msgs::TransformStamped msg;
    msg.header.frame_id = src;
    msg.child_frame_id = target;
    msg.header.stamp = time + ros::Duration(0.1);
    msg.transform = tf2::toMsg(tf);
    tf_broadcaster.sendTransform(msg);
}

geometry_msgs::Pose ORB_ROS_Node::transform_to_pose(const tf2::Transform tf)
{
    geometry_msgs::Pose pose;
    pose.position.x = tf.getOrigin().getX();
    pose.position.y = tf.getOrigin().getY();
    pose.position.z = tf.getOrigin().getZ();
    pose.orientation = tf2::toMsg(tf.getRotation());    
    return pose;
}

tf2::Stamped<tf2::Transform> ORB_ROS_Node::lookup_transform(const string& src, const string& target, ros::Time time)
{
    tf2::Stamped<tf2::Transform> tf;
    try {
        // Get the transform from camera to target
        geometry_msgs::TransformStamped tf_msg = tfBuffer.lookupTransform( src,target, time);
        // Convert to tf2
        tf2::fromMsg(tf_msg, tf);
    } catch (tf2::TransformException &ex) {
        tf.setIdentity();
    }
    return tf;
}

tf2::Stamped<tf2::Transform> ORB_ROS_Node::lookup_transform_duration(const string& src, const string& target, ros::Time time, ros::Duration duration)
{
    ROS_INFO("Looking up tf from %s to %s",src.c_str(), target.c_str());
    tf2::Stamped<tf2::Transform> tf;
    try {
        // Get the transform from camera to target
        geometry_msgs::TransformStamped tf_msg = tfBuffer.lookupTransform( src,target, time,duration);
        // Convert to tf2
        tf2::fromMsg(tf_msg, tf);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        tf.setIdentity();
    }
    return tf;
}

void ORB_ROS_Node::save_map_cb(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data == 1)
    {
        if(!orb_system->mStrSaveAtlasToFile.empty())
        {
            cout<<"Saving atlas to file "+orb_system->mStrSaveAtlasToFile<<endl;
            ORB_SLAM3::Verbose::PrintMess("Atlas saving to file " + orb_system->mStrSaveAtlasToFile, ORB_SLAM3::Verbose::VERBOSITY_NORMAL);
            orb_system->SaveAtlas(ORB_SLAM3::System::FileType::BINARY_FILE);
        }
        cout << "Saved Atlas Finished" << endl;
    }
}

void ORB_ROS_Node::config_cb(const ORB_SLAM3::orb_slam3Config& config, uint32_t level)
{
    if(config.Localization_mode) orb_system->ActivateLocalizationMode();
    else orb_system->DeactivateLocalizationMode();
}
