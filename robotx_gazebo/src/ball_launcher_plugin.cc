
// headers in this package
#include <load_params.hh>
#include <ball_launcher_plugin.hh>

using namespace gazebo;

ball_launcher_plugin::ball_launcher_plugin() {}
ball_launcher_plugin::~ball_launcher_plugin() {}

void ball_launcher_plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
{
  // Read ball sdf file path
  ball_sdf_path_ = ros::package::getPath("robotx_gazebo") + "/modeles/ball/ball.sdf";
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
  //
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ball_launcher_plugin::OnUpdate, this, _1));
}


void ball_launcher_plugin::OnUpdate(const common::UpdateInfo& _info)
{
  // ROS_ERROR_STREAM(ball_sdf_path_);
  ROS_ERROR_STREAM("TEST");
}

GZ_REGISTER_MODEL_PLUGIN(ball_launcher_plugin)
