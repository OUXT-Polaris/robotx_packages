#ifndef GEOGRAPHIC_MAP_SERVER_H_INCLUDED
#define GEOGRAPHIC_MAP_SERVER_H_INCLUDED

//headers in this package
#include <robotx_msgs/GeographicMap.h>

//headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <yaml-cpp/yaml.h>

//headers in stl
#include <vector>

class geographic_map_server
{
public:
  struct parameters
  {
    std::string yaml_filepath;
    std::string origin_fix_topic;
    parameters()
    {
      ros::param::param<std::string>(ros::this_node::getName()+"/yaml_filepath", yaml_filepath, ros::this_node::getName()+"/yaml_filepath");
      ros::param::param<std::string>(ros::this_node::getName()+"/origin_fix_topic", origin_fix_topic, "/origin/fix");
    }
  };
  geographic_map_server();
  ~geographic_map_server();
private:
  const parameters params_;
  ros::NodeHandle nh_;
  ros::Subscriber origin_fix_sub_;
  void origin_fix_callback_(sensor_msgs::NavSatFix msg);
  robotx_msgs::GeographicMap read_yaml_file();
};
#endif //GEOGRAPHIC_MAP_SERVER_H_INCLUDED
