#ifndef GEOGRAPHIC_MAP_SERVER_H_INCLUDED
#define GEOGRAPHIC_MAP_SERVER_H_INCLUDED

// headers in this package
#include <robotx_msgs/GeographicMap.h>

// headers in ROS
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <yaml-cpp/yaml.h>

// headers in stl
#include <vector>

class geographic_map_server {
 public:
  struct parameters {
    std::string yaml_filepath;
    std::string frame_id;
    double frequency;
    parameters() {
      ros::param::param<std::string>(ros::this_node::getName() + "/yaml_filepath", yaml_filepath,
                                     ros::package::getPath("robotx_navigation") + "/data/coastline.yaml");
      ros::param::param<std::string>(ros::this_node::getName() + "/frame_id", frame_id, "world");
      ros::param::param<double>(ros::this_node::getName() + "/frequency", frequency, 1);
    }
  };
  geographic_map_server();
  ~geographic_map_server();
  void run();

 private:
  const parameters params_;
  ros::NodeHandle nh_;
  ros::Publisher map_pub_;
  void origin_fix_callback_(sensor_msgs::NavSatFix msg);
  robotx_msgs::GeographicMap generate_geographic_map_();
  robotx_msgs::GeographicMap geographic_map_;
};
#endif  // GEOGRAPHIC_MAP_SERVER_H_INCLUDED
