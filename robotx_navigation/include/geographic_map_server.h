#ifndef GEOGRAPHIC_MAP_SERVER_H_INCLUDED
#define GEOGRAPHIC_MAP_SERVER_H_INCLUDED

//headers in this package
#include <robotx_msgs/GeographicMap.h>

//headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>

//headers in stl
#include <vector>

class geographic_map_server
{
public:
  geographic_map_server();
  ~geographic_map_server();
private:
};
#endif //GEOGRAPHIC_MAP_SERVER_H_INCLUDED
