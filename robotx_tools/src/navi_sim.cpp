#include <navi_sim.h>

navi_sim::navi_sim() : tf_listener_(tf_buffer_)
{
    nh_ = ros::NodeHandle();
    pnh_ = ros::NodeHandle("~");
    pnh_.param<int>("utm_zone", utm_zone_, 0);
    pnh_.param<double>("update_rate", update_rate_, 10);
    pnh_.param<double>("gps_update_rate", gps_update_rate_, 1);
    pnh_.param<double>("velodyne_rate", velodyne_rate_, 10.0);
    pnh_.param<double>("detection_range", detection_range_, 10.0);
    pnh_.param<double>("buoy_bbox_size", buoy_bbox_size_, 1.0);
    pnh_.param<bool>("southhemi", southhemi_, false);
    pnh_.param<std::string>("gps_frame", gps_frame_, "gps");
    pnh_.param<std::string>("world_frame", world_frame_, "world");
    pnh_.param<std::string>("robot_frame", robot_frame_, "base_link");
    pnh_.param<std::string>("velodyne_frame", velodyne_frame_, "velodyne");
    pnh_.param<std::string>("fix_topic", fix_topic_, "/fix");
    pnh_.param<std::string>("gps_twist_topic", gps_twist_topic_, "/fix/twist");
    pnh_.param<std::string>("true_course_topic", true_course_topic_, "/true_course");
    fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(fix_topic_,1);
    true_course_pub_ = nh_.advertise<geometry_msgs::QuaternionStamped>(true_course_topic_,1);
    gps_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(gps_twist_topic_,1);
    true_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot_pose/truth",1);
    navigation_trigger_event_pub_ = nh_.advertise<robotx_msgs::Event>("/robotx_state_machine_node/navigation_state_machine/trigger_event",1);
    obstacles_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/euclidean_clustering_node/bbox",1);
    twist_cmd_sub_ = nh_.subscribe("/cmd_vel",1,&navi_sim::cmd_vel_callback,this);
    init_pose_sub_ = nh_.subscribe("/initialpose",1,&navi_sim::init_pose_callback_,this);
    field_map_sub_ = nh_.subscribe("/field_map",1,&navi_sim::field_map_callback_,this);
}

navi_sim::~navi_sim()
{

}

void navi_sim::field_map_callback_(const robotx_msgs::FieldMap::ConstPtr msg)
{
    field_map_ = *msg;
    return;
}

void navi_sim::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr msg)
{
    mtx_.lock();
    twist_cmd_ = *msg;
    mtx_.unlock();
    return;
}

void navi_sim::init_pose_callback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
    mtx_.lock();
    current_twist_ = geometry_msgs::Twist();
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.pose = msg->pose.pose;
    geometry_msgs::PoseStamped transformed_pose;
    geometry_msgs::TransformStamped transform_stamped;
    if(msg->header.frame_id != world_frame_)
    {
        try
        {
            transform_stamped = tf_buffer_.lookupTransform(world_frame_, msg->header.frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            return;
        }
        tf2::doTransform(pose_msg,transformed_pose,transform_stamped);
    }
    else
    {
        transformed_pose = pose_msg;
    }
    tf::Quaternion quat(transformed_pose.pose.orientation.x,transformed_pose.pose.orientation.y,
        transformed_pose.pose.orientation.z,transformed_pose.pose.orientation.w);
    double r,p,y;
    tf::Matrix3x3(quat).getRPY(r, p, y);
    geometry_msgs::Pose2D pose;
    pose.theta = y;
    pose.x = transformed_pose.pose.position.x;
    pose.y = transformed_pose.pose.position.y;
    current_pose_ = pose;
    robotx_msgs::Event event_msg;
    event_msg.trigger_event_name = "navigation_start";
    navigation_trigger_event_pub_.publish(event_msg);
    mtx_.unlock();
    return;
}

double navi_sim::get_diff_angle_(double from,double to)
{
    double ans = 0;
    double inner_prod = std::cos(from)*std::cos(to)+std::sin(from)*std::sin(to);
    double theta = std::acos(inner_prod);
    double outer_prod = std::cos(from)*std::sin(to)-std::sin(from)*std::cos(to);
    if(outer_prod > 0)
    {
        ans = theta;
    }
    else
    {
        ans = -1 * theta;
    }
    return ans;
}

void navi_sim::update_gps_()
{
    ros::Rate rate(gps_update_rate_);
    while(ros::ok())
    {
        mtx_.lock();
        double lat,lon;
        if(current_pose_)
        {
            ros::Time now = ros::Time::now();
            double x = current_pose_->x;
            double y = current_pose_->y;
            geodesy::UTMPoint utm_point;
            utm_point.northing = current_pose_->y;
            utm_point.easting = current_pose_->x;
            utm_point.altitude = 0;
            utm_point.zone = utm_zone_;
            utm_point.band = 'Q';
            geographic_msgs::GeoPoint geopoint = geodesy::toMsg(utm_point);
            lat = geopoint.latitude;
            lon = geopoint.longitude;
            sensor_msgs::NavSatFix fix_msg;
            fix_msg.header.frame_id = gps_frame_;
            fix_msg.header.stamp = now;
            fix_msg.latitude = lat;
            fix_msg.longitude = lon;
            fix_pub_.publish(fix_msg);
            geometry_msgs::QuaternionStamped quat_msg;
            quat_msg.header.frame_id = gps_frame_;
            quat_msg.header.stamp = now;
            tf::Quaternion quat = tf::createQuaternionFromRPY(0,0,current_pose_->theta);
            true_course_buf_.push_back(current_pose_->theta);
            quaternionTFToMsg(quat, quat_msg.quaternion);
            true_course_pub_.publish(quat_msg);
            geometry_msgs::TwistStamped gps_twist;
            gps_twist.header.frame_id = gps_frame_;
            gps_twist.header.stamp = now;
            gps_twist.twist.linear.x = current_twist_.linear.x;
            if(true_course_buf_.size() == 2)
            {
                ROS_ERROR_STREAM(true_course_buf_[1] << "," << true_course_buf_[0]);
                gps_twist.twist.angular.z = get_diff_angle_(true_course_buf_[1],true_course_buf_[0])/gps_update_rate_;
            }
            gps_twist_pub_.publish(gps_twist);
        }
        mtx_.unlock();
        rate.sleep();
    }
    return;
}

void navi_sim::update_obstacle_()
{
    ros::Rate rate(velodyne_rate_);
    while(ros::ok())
    {
        mtx_.lock();
        if(field_map_ && current_pose_)
        {
            boost::optional<jsk_recognition_msgs::BoundingBoxArray> obstacles = get_obstacles_();
            if(obstacles)
            {
                obstacles_pub_.publish(*obstacles);
            }
            else
            {
                jsk_recognition_msgs::BoundingBoxArray empty_bbox;
                empty_bbox.header.frame_id = velodyne_frame_;
                empty_bbox.header.stamp = ros::Time::now();
                obstacles_pub_.publish(empty_bbox);
            }
        }
        mtx_.unlock();
        rate.sleep();
    }
    return;
}

void navi_sim::update_pose_()
{
    ros::Rate rate(update_rate_);
    while(ros::ok())
    {
        mtx_.lock();
        if(current_pose_)
        {
            geometry_msgs::PoseStamped pose3d;
            geometry_msgs::Pose2D new_pose;
            double d_theta = twist_cmd_.angular.z / update_rate_;
            new_pose.theta = current_pose_->theta + d_theta;
            new_pose.x = current_pose_->x + twist_cmd_.linear.x * std::cos(current_pose_->theta + d_theta/2) / update_rate_;
            new_pose.y = current_pose_->y + twist_cmd_.linear.x * std::sin(current_pose_->theta + d_theta/2) / update_rate_;
            pose3d.pose.position.x = new_pose.x;
            pose3d.pose.position.y = new_pose.y;
            pose3d.header.frame_id = world_frame_;
            pose3d.header.stamp = ros::Time::now();
            tf::Quaternion quat=tf::createQuaternionFromRPY(0,0,current_pose_->theta);
            tf::quaternionTFToMsg(quat,pose3d.pose.orientation);
            true_pose_pub_.publish(pose3d);
            current_pose_ = new_pose;
        }
        mtx_.unlock();
        rate.sleep();
    }
    return;
}

void navi_sim::run()
{
    boost::thread update_pose_thread_(&navi_sim::update_pose_,this);
    boost::thread update_gps_thread_(&navi_sim::update_gps_,this);
    boost::thread update_velodyne_thread_(&navi_sim::update_obstacle_,this);
    return;
}

boost::optional<jsk_recognition_msgs::BoundingBoxArray> navi_sim::get_obstacles_()
{
    jsk_recognition_msgs::BoundingBoxArray obstacles;
    if(!current_pose_)
    {
        return boost::none;
    }
    obstacles.header.frame_id = velodyne_frame_;
    obstacles.header.stamp = ros::Time::now();

    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer_.lookupTransform(velodyne_frame_, field_map_->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return boost::none;
    }
    ros::Time now = ros::Time::now();
    for(auto buoy_itr = field_map_->red_buoys.begin(); buoy_itr != field_map_->red_buoys.end(); buoy_itr++)
    {
        geometry_msgs::PointStamped map_point;
        map_point.header = field_map_->header;
        map_point.point = *buoy_itr;
        geometry_msgs::PointStamped transformed_point;
        tf2::doTransform(map_point,transformed_point,transform_stamped);
        double distance = std::sqrt(std::pow(transformed_point.point.x,2)+std::pow(transformed_point.point.y,2));
        if(detection_range_ > distance)
        {
            jsk_recognition_msgs::BoundingBox bbox;
            bbox.header.frame_id = velodyne_frame_;
            bbox.header.stamp = now;
            bbox.pose.position = transformed_point.point;
            bbox.pose.orientation.x = 0;
            bbox.pose.orientation.y = 0;
            bbox.pose.orientation.z = 0;
            bbox.pose.orientation.w = 1;
            bbox.dimensions.x = buoy_bbox_size_;
            bbox.dimensions.y = buoy_bbox_size_;
            bbox.dimensions.z = buoy_bbox_size_;
            obstacles.boxes.push_back(bbox);
        }
    }
    for(auto buoy_itr = field_map_->green_buoys.begin(); buoy_itr != field_map_->green_buoys.end(); buoy_itr++)
    {
        geometry_msgs::PointStamped map_point;
        map_point.header = field_map_->header;
        map_point.point = *buoy_itr;
        geometry_msgs::PointStamped transformed_point;
        tf2::doTransform(map_point,transformed_point,transform_stamped);
        double distance = std::sqrt(std::pow(transformed_point.point.x,2)+std::pow(transformed_point.point.y,2));
        if(detection_range_ > distance)
        {
            jsk_recognition_msgs::BoundingBox bbox;
            bbox.header.frame_id = velodyne_frame_;
            bbox.header.stamp = now;
            bbox.pose.position = transformed_point.point;
            bbox.pose.orientation.x = 0;
            bbox.pose.orientation.y = 0;
            bbox.pose.orientation.z = 0;
            bbox.pose.orientation.w = 1;
            bbox.dimensions.x = buoy_bbox_size_;
            bbox.dimensions.y = buoy_bbox_size_;
            bbox.dimensions.z = buoy_bbox_size_;
            obstacles.boxes.push_back(bbox);
        }
    }
    for(auto buoy_itr = field_map_->white_buoys.begin(); buoy_itr != field_map_->white_buoys.end(); buoy_itr++)
    {
        geometry_msgs::PointStamped map_point;
        map_point.header = field_map_->header;
        map_point.point = *buoy_itr;
        geometry_msgs::PointStamped transformed_point;
        tf2::doTransform(map_point,transformed_point,transform_stamped);
        double distance = std::sqrt(std::pow(transformed_point.point.x,2)+std::pow(transformed_point.point.y,2));
        if(detection_range_ > distance)
        {
            jsk_recognition_msgs::BoundingBox bbox;
            bbox.header.frame_id = velodyne_frame_;
            bbox.header.stamp = now;
            bbox.pose.position = transformed_point.point;
            bbox.pose.orientation.x = 0;
            bbox.pose.orientation.y = 0;
            bbox.pose.orientation.z = 0;
            bbox.pose.orientation.w = 1;
            bbox.dimensions.x = buoy_bbox_size_;
            bbox.dimensions.y = buoy_bbox_size_;
            bbox.dimensions.z = buoy_bbox_size_;
            obstacles.boxes.push_back(bbox);
        }
    }
    return obstacles;
}