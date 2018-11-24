#include <navi_sim.h>

navi_sim::navi_sim() : tf_listener_(tf_buffer_)
{
    nh_ = ros::NodeHandle();
    pnh_ = ros::NodeHandle("~");
    pnh_.param<int>("utm_zone", utm_zone_, 0);
    pnh_.param<double>("update_rate", update_rate_, 10);
    pnh_.param<bool>("southhemi", southhemi_, false);
    pnh_.param<std::string>("gps_frame", gps_frame_, "gps");
    pnh_.param<std::string>("world_frame", world_frame_, "world");
    pnh_.param<std::string>("fix_topic", fix_topic_, "/fix");
    pnh_.param<std::string>("true_course_topic", true_course_topic_, "/true_course");
    fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(fix_topic_,1);
    true_course_pub_ = nh_.advertise<geometry_msgs::QuaternionStamped>(true_course_topic_,1);
    twist_cmd_sub_ = nh_.subscribe("/cmd_vel",1,&navi_sim::cmd_vel_callback,this);
    init_pose_sub_ = nh_.subscribe("/initialpose",1,&navi_sim::init_pose_callback_,this);
}

navi_sim::~navi_sim()
{
    
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
    mtx_.unlock();
    return;
}

void navi_sim::update_()
{
    ros::Rate rate(update_rate_);
    while(ros::ok())
    {
        mtx_.lock();
        double lat,lon;
        if(current_pose_)
        {
            UTMXYToLatLon(current_pose_->x, current_pose_->y, utm_zone_, southhemi_, lat, lon);
            sensor_msgs::NavSatFix fix_msg;
            fix_msg.header.frame_id = gps_frame_;
            fix_msg.header.stamp = ros::Time::now();
            fix_msg.latitude = lat;
            fix_msg.longitude = lon;
            fix_pub_.publish(fix_msg);
            geometry_msgs::QuaternionStamped quat_msg;
            quat_msg.header.frame_id = gps_frame_;
            quat_msg.header.stamp = ros::Time::now();
            tf::Quaternion quat = tf::createQuaternionFromRPY(0,0,current_pose_->theta);
            quaternionTFToMsg(quat, quat_msg.quaternion);
            true_course_pub_.publish(quat_msg);
        }
        mtx_.unlock();
        rate.sleep();
    }
    return;
}

void navi_sim::run()
{
    boost::thread update_thread_(&navi_sim::update_,this);
    return;
}