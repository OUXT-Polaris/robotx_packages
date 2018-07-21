// headers in this package
#include <ball_launcher_plugin.hh>
#include <load_params.hh>

// headers in ROS
//#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SpawnModel.h>
#include <ros/ros.h>

// headers in STL
#include <fstream>

using namespace gazebo;

ball_launcher_plugin::ball_launcher_plugin() {}
ball_launcher_plugin::~ball_launcher_plugin() {}
void ball_launcher_plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr sdf) {
  // ball_exist_ = false;
  count_ = 0;
  // Read ball sdf file path
  ball_sdf_path_ = ros::package::getPath("robotx_gazebo") + "/models/ball/ball.urdf";
  std::string default_ball_launcher_link_name = "ball_launcher_pitch_link";
  std::string ball_launcher_link_name;
  LoadParams(sdf, "target_link", ball_launcher_link_name, default_ball_launcher_link_name);
  LoadParams(sdf, "num_balls", num_balls_, 5);
  ball_remains_ = num_balls_;
  model_ptr_ = _parent;
  ball_launcher_link_ptr_ = model_ptr_->GetLink(ball_launcher_link_name);

  // Read lifetime
  double lifetime;
  LoadParams(sdf, "lifetime", lifetime, 5.0);
  ball_lifetime_ = ros::Duration(lifetime);

  ball_launcher_status_pub_ = nh_.advertise<robotx_msgs::BallLauncherStatus>("/ball_launcher/status", 10);

  load_ball_urdf();
  spawn_client_ = nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  // delete_client_ =
  // nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

  this->update_connection_ =
      event::Events::ConnectWorldUpdateBegin(boost::bind(&ball_launcher_plugin::OnUpdate, this, _1));
  launch_sub_ =
      nh_.subscribe("/ball_launcher/launch_ball", 1, &ball_launcher_plugin::ball_launcher_callback, this);
  return;
}

void ball_launcher_plugin::ball_launcher_callback(std_msgs::Empty /*msg*/) {
  if (ball_remains_ > 0) {
    spawn_ball(num_balls_ - ball_remains_);
    ball_remains_ = ball_remains_ - 1;
  }
}

void ball_launcher_plugin::OnUpdate(const common::UpdateInfo& /*_info*/) {
  ball_launcher_link_pose_ = ball_launcher_link_ptr_->GetWorldPose();
  count_ = count_ + 1;
  if (count_ % 100 == 0) {
    robotx_msgs::BallLauncherStatus msg;
    msg.ball_remains = ball_remains_;
    msg.ball_launched = num_balls_ - ball_remains_;
    ball_launcher_status_pub_.publish(msg);
    count_ = 0;
  }
}

void ball_launcher_plugin::load_ball_urdf() {
  ball_urdf_str_ = "";
  std::ifstream ifs(ball_sdf_path_);
  if (ifs.fail()) {
    ROS_ERROR_STREAM("failed to find " << ball_sdf_path_);
    std::exit(0);
  }
  std::string str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  ball_urdf_str_ = str;
  return;
}

/*
void ball_launcher_plugin::delete_ball()
{
  gazebo_msgs::DeleteModel msg;
  msg.request.model_name = "ball";
  if (delete_client_.call(msg)) {
    ROS_INFO_STREAM(msg.response.status_message);
    ball_exist_ = false;
  }
  else {
    ROS_ERROR_STREAM("failed to delete ball");
  }
  return;
}
*/

void ball_launcher_plugin::spawn_ball(int ball_id) {
  // see also
  // http://docs.ros.org/kinetic/api/gazebo_msgs/html/srv/SpawnModel.html
  gazebo_msgs::SpawnModel msg;
  msg.request.model_name = "ball_" + std::to_string(ball_id);
  msg.request.model_xml = ball_urdf_str_;
  msg.request.reference_frame = "";
  msg.request.initial_pose.position.x = 0;
  msg.request.initial_pose.position.y = 0;
  msg.request.initial_pose.position.z = 0;
  msg.request.initial_pose.orientation.x = 0;
  msg.request.initial_pose.orientation.y = 0;
  msg.request.initial_pose.orientation.z = 0;
  msg.request.initial_pose.orientation.w = 1;
  msg.request.robot_namespace = "ball_" + std::to_string(ball_id);
  if (spawn_client_.call(msg)) {
    last_spawm_time_ = ros::Time::now();
    // ball_exist_ = true;
  } else {
    ROS_ERROR_STREAM("failed to spawn ball");
  }
  return;
}
GZ_REGISTER_MODEL_PLUGIN(ball_launcher_plugin)
