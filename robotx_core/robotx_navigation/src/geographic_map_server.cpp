// headers in this package
#include <geographic_map_server.h>

geographic_map_server::geographic_map_server() : params_() {
  map_pub_ = nh_.advertise<robotx_msgs::GeographicMap>("/geographic_map", 1);
  geographic_map_ = generate_geographic_map_();
}

geographic_map_server::~geographic_map_server() {}

void geographic_map_server::run() {
  ros::Rate rate(params_.frequency);
  while (ros::ok) {
    geographic_map_.header.stamp = ros::Time::now();
    map_pub_.publish(geographic_map_);
    rate.sleep();
  }
}

robotx_msgs::GeographicMap geographic_map_server::generate_geographic_map_() {
  robotx_msgs::GeographicMap map;
  map.header.frame_id = params_.frame_id;
  try {
    YAML::Node map_yaml_data = YAML::LoadFile(params_.yaml_filepath);
    YAML::Node way_data = map_yaml_data["way"];
    YAML::Node node_data = map_yaml_data["node"];
    for (auto node_data_itr = node_data.begin(); node_data_itr != node_data.end(); ++node_data_itr) {
      robotx_msgs::GeographicPoint geopoint;
      geopoint.node_id = (uint64_t)std::stoull(node_data_itr->first.as<std::string>());
      YAML::Node geopos_data = node_data[node_data_itr->first.as<std::string>()];
      geopoint.geographic_point.latitude = geopos_data["lat"].as<double>();
      geopoint.geographic_point.longitude = geopos_data["lon"].as<double>();
      map.points.push_back(geopoint);
    }
    for (auto way_data_itr = way_data.begin(); way_data_itr != way_data.end(); ++way_data_itr) {
      robotx_msgs::GeographicLine geoline;
      geoline.id = (uint64_t)std::stoull(way_data_itr->first.as<std::string>());
      YAML::Node geoline_data = way_data[way_data_itr->first.as<std::string>()];
      geoline.start_node_id = (uint64_t)std::stoull(geoline_data["start_node_id"].as<std::string>());
      geoline.end_node_id = (uint64_t)std::stoull(geoline_data["end_node_id"].as<std::string>());
      map.lines.push_back(geoline);
    }
  } catch (YAML::Exception &e) {
    ROS_ERROR_STREAM(e.what());
  }
  return map;
}