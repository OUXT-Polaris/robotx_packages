#include <waypoint_clicker.h>

waypoint_clicker::waypoint_clicker(ros::NodeHandle nh, ros::NodeHandle pnh) : tf_listener_(tf_buffer_)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("waypoint_frame", waypoint_frame_, "map");
}

waypoint_clicker::~waypoint_clicker()
{

}

void waypoint_clicker::run()
{
    marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("marker",1);
    goal_pose_sub_ = nh_.subscribe("/move_base_simple/goal",1,&waypoint_clicker::goal_pose_callback_,this);
    boost::thread marker_thread_(&waypoint_clicker::publish_marker_,this);
    return;
}

void waypoint_clicker::publish_marker_()
{
    ros::Rate rate(1);
    while(ros::ok())
    {
        visualization_msgs::MarkerArray marker_msg;
        int i = 0;
        ros::Time now = ros::Time::now();
        for(auto pose_itr = target_poses_.begin(); pose_itr != target_poses_.end(); pose_itr++)
        {
            visualization_msgs::Marker single_marker;
            single_marker.type = single_marker.ARROW;
            single_marker.id = i;
            single_marker.action = single_marker.ADD;
            single_marker.pose = pose_itr->pose;
            single_marker.header.frame_id = waypoint_frame_;
            single_marker.header.stamp = now;
            single_marker.scale.x = 3;
            single_marker.scale.y = 3;
            single_marker.scale.z = 30;
            single_marker.frame_locked = true;
            single_marker.color.r = 0;
            single_marker.color.g = 1;
            single_marker.color.b = 0;
            single_marker.color.a = 1;
            marker_msg.markers.push_back(single_marker);
            i++;
        }
        marker_pub_.publish(marker_msg);
        rate.sleep();
    }
    return;
}

void waypoint_clicker::goal_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    geometry_msgs::PoseStamped transformed_pose;
    if(msg->header.frame_id != waypoint_frame_)
    {
        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_.lookupTransform(waypoint_frame_, msg->header.frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            return;
        }
        tf2::doTransform(*msg,transformed_pose,transform_stamped);
    }
    else
    {
        transformed_pose = *msg;
    }
    geometry_msgs::Pose2D pose2d;
    target_poses_.push_back(transformed_pose);
    return;
}