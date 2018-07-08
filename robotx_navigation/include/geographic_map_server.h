#ifndef GEOGRAPHIC_MAP_SERVER_H_INCLUDED
#define GEOGRAPHIC_MAP_SERVER_H_INCLUDED

//headers in this package
#include <robotx_msgs/GeographicMap.h>

//headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <yaml-cpp/yaml.h>

//headers in stl
#include <vector>

class geographic_map_server
{
public:
  struct parameters
  {
    std::string yaml_filepath;
    parameters()
    {
      ros::param::param<std::string>(ros::this_node::getName()+"/yaml_filepath", yaml_filepath, ros::this_node::getName()+"/yaml_filepath");
    }
  };
  geographic_map_server();
  ~geographic_map_server();
private:
};
#endif //GEOGRAPHIC_MAP_SERVER_H_INCLUDED
