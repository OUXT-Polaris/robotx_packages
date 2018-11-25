#include <waypoint_clicker.h>

waypoint_clicker::waypoint_clicker(ros::NodeHandle nh, ros::NodeHandle pnh) : tf_listener_(tf_buffer_)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("world_frame", world_frame_, "world");
    goal_pose_sub_ = nh_.subscribe("/move_base_simple/goal",1,&waypoint_clicker::goal_pose_callback_,this);
}

waypoint_clicker::~waypoint_clicker()
{

}

void waypoint_clicker::goal_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    geometry_msgs::PoseStamped transformed_pose;
    if(msg->header.frame_id != world_frame_)
    {
        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_.lookupTransform(world_frame_, msg->header.frame_id, ros::Time(0));
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
    return;
}