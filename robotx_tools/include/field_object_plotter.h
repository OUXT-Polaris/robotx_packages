#ifndef FIELD_OBJECT_PLOTTER_H_INCLUDED
#define FIELD_OBJECT_PLOTTER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>

#include <robotx_msgs/ObjectRegionOfInterestArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <map>
#include <fstream>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class field_object_plotter
{
public:
    field_object_plotter();
    ~field_object_plotter();
private:
    void roi_callback_(const robotx_msgs::ObjectRegionOfInterestArray::ConstPtr msg);
    //void pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg);
    ros::Subscriber roi_sub_;
    //ros::Subscriber pose_sub_;
    std::vector<std::pair<std::string,geometry_msgs::PoseStamped> > objects_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};
#endif  //FIELD_OBJECT_PLOTTER_H_INCLUDED