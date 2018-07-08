//headers in this package
#include <geographic_map_server.h>

geographic_map_server::geographic_map_server() : params_()
{
  generate_geographic_map();
  origin_fix_sub_ = nh_.subscribe(params_.origin_fix_topic, 1, &geographic_map_server::origin_fix_callback_, this);
}

geographic_map_server::~geographic_map_server()
{

}

void geographic_map_server::origin_fix_callback_(sensor_msgs::NavSatFix msg)
{

}

robotx_msgs::GeographicMap geographic_map_server::generate_geographic_map()
{
  robotx_msgs::GeographicMap map;
  return map;
}