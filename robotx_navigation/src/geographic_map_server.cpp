//headers in this package
#include <geographic_map_server.h>

geographic_map_server::geographic_map_server() : params_()
{
  read_yaml_file();
  origin_fix_sub_ = nh_.subscribe(params_.origin_fix_topic, 1, &geographic_map_server::origin_fix_callback_, this);
}

geographic_map_server::~geographic_map_server()
{

}

void geographic_map_server::origin_fix_callback_(sensor_msgs::NavSatFix msg)
{

}

robotx_msgs::GeographicMap geographic_map_server::read_yaml_file()
{
  robotx_msgs::GeographicMap map;
  try
  {
    YAML::Node map_yaml_data = YAML::LoadFile(params_.yaml_filepath);
  }
  catch(YAML::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    std::exit(0);
    return map;
  }
  return map;
}