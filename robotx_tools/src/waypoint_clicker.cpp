#include <waypoint_clicker.h>

waypoint_clicker::waypoint_clicker(ros::NodeHandle nh, ros::NodeHandle pnh) : tf_listener_(tf_buffer_)
{
    initial_pose_received_ = false;
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("waypoint_frame", waypoint_frame_, "map");
    marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("marker",1);
    goal_pose_sub_ = nh_.subscribe("/move_base_simple/goal",1,&waypoint_clicker::goal_pose_callback_,this);
}

waypoint_clicker::~waypoint_clicker()
{
    if(initial_pose_received_)
    {
        add_goal_pose_(initial_pose_);
    }
}

void waypoint_clicker::run()
{
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
            single_marker.ns = "waypoint";
            marker_msg.markers.push_back(single_marker);
            visualization_msgs::Marker id_marker;
            id_marker.type = id_marker.TEXT_VIEW_FACING;
            id_marker.id = i;
            id_marker.action = id_marker.ADD;
            id_marker.pose = pose_itr->pose;
            id_marker.header.frame_id = waypoint_frame_;
            id_marker.header.stamp = now;
            id_marker.scale.x = 1;
            id_marker.scale.y = 1;
            id_marker.scale.z = 10;
            id_marker.frame_locked = true;
            id_marker.color.r = 1;
            id_marker.color.g = 1;
            id_marker.color.b = 1;
            id_marker.color.a = 1;
            id_marker.ns = "id";
            id_marker.text = std::to_string(i);
            marker_msg.markers.push_back(id_marker);
            i++;
        }
        marker_pub_.publish(marker_msg);
        rate.sleep();
    }
    return;
}

void waypoint_clicker::add_goal_pose_(const geometry_msgs::PoseStamped::ConstPtr msg)
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
    pose2d.x = transformed_pose.pose.position.x;
    pose2d.y = transformed_pose.pose.position.y;
    double r,p,y;
    tf::Quaternion quat(transformed_pose.pose.orientation.x, transformed_pose.pose.orientation.y,
        transformed_pose.pose.orientation.z,transformed_pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(r, p, y);
    pose2d.theta = y;
    target_poses_2d_.push_back(pose2d);
    std::ofstream csv_file;
    std::string filename = ros::package::getPath("robotx_navigation") +"/data/clicked_waypoints.csv";
    csv_file.open(filename, std::ios::trunc);
    int i = 0;
    for(auto pose_2d_itr = target_poses_2d_.begin(); pose_2d_itr != target_poses_2d_.end(); pose_2d_itr++)
    {
        std::string line_str = std::to_string(i) + "," + waypoint_frame_ + "," + std::to_string(pose_2d_itr->x) + "," 
            + std::to_string(pose_2d_itr->y) + "," + std::to_string(pose_2d_itr->theta) + ",None";
        csv_file << line_str << std::endl;
        i++;
    }
    csv_file.close();
    return;
}

void waypoint_clicker::goal_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    if(!initial_pose_received_)
    {
        initial_pose_received_ = true;
        initial_pose_ = msg;
    }
    add_goal_pose_(msg);
    return;
}