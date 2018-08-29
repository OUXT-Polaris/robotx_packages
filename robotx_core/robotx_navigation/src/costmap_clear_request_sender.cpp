#include <costmap_clear_request_sender.h>

costmap_clear_request_sender::costmap_clear_request_sender() {
  client_ = nh_.serviceClient<std_srvs::Empty>(params_.target_service_name);
}

costmap_clear_request_sender::~costmap_clear_request_sender() {}

void costmap_clear_request_sender::run() {
  ros::Rate rate(params_.frequency);
  std_srvs::Empty srv_msg;
  while (ros::ok) {
    client_.call(srv_msg);
    rate.sleep();
  }
}