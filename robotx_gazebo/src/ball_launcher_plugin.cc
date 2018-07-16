
// headers in this package
#include <ball_launcher_plugin.hh>
#include <load_params.hh>

// headers in ROS
#include <gazebo_msgs/SpawnModel.h>
#include <ros/ros.h>

using namespace gazebo;

ball_launcher_plugin::ball_launcher_plugin() {}
ball_launcher_plugin::~ball_launcher_plugin() {}
void ball_launcher_plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
{
  // Read ball sdf file path
  ball_sdf_path_ =
      ros::package::getPath("robotx_gazebo") + "/modeles/ball/ball.urdf";
  std::string default_ball_launcher_link_name = "ball_launcher_pitch_link";
  std::string ball_launcher_link_name;
  LoadParams(sdf, "target_link", ball_launcher_link_name,
             default_ball_launcher_link_name);
  model_ptr_ = _parent;

  this->model_ptr_->GetLink(ball_launcher_link_name);
  //  this->ball_launcher_link_ptr_ =

  // Read lifetime
  double lifetime;
  LoadParams(sdf, "lifetime", lifetime, 5.0);
  ball_lifetime_ = ros::Duration(lifetime);

  // world_ptr_->InsertModelFile(ball_sdf_path_);
  client_ =
      nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ball_launcher_plugin::OnUpdate, this, _1));
}

void ball_launcher_plugin::OnUpdate(const common::UpdateInfo& _info)
{
  // ROS_ERROR_STREAM(ball_sdf_path_);
  // ROS_ERROR_STREAM("TEST");
}

void ball_launcher_plugin::spawn_ball()
{
  gazebo_msgs::SpawnModel msg;
  msg.request.model_name = "ball";
  msg.request.reference_frame = "ball_launcher_base_link";
}
GZ_REGISTER_MODEL_PLUGIN(ball_launcher_plugin)
