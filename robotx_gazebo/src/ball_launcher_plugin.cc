// headers in gazebo
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>

// headers in ROS
#include <ros/package.h>
#include <ros/ros.h>

// headers in this package
#include <load_params.hh>

using namespace gazebo;
class ball_launcher_plugin : public ModelPlugin {
 public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
  {
    // Read ball sdf file path
    ball_sdf_path_ =
        ros::package::getPath("robotx_gazebo") + "/modeles/ball/ball.sdf";
    std::string default_ball_launcher_link_name = "ball_launcher_pitch_link";
    std::string ball_launcher_link_name;
    LoadParams(sdf, "target_link", ball_launcher_link_name,
               default_ball_launcher_link_name);
    model_ptr_ = _parent;
    this->ball_launcher_link_ptr_ =
        this->model_ptr_->GetLink(ball_launcher_link_name);

    // Read lifetime
    double lifetime;
    LoadParams(sdf, "lifetime", lifetime, 5.0);
    ball_lifetime_ = ros::Duration(lifetime);

    // world_ptr_->InsertModelFile(ball_sdf_path_);
    //
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ball_launcher_plugin::OnUpdate, this, _1));
  }

  void OnUpdate(const common::UpdateInfo& /*_info*/)
  {
    // ROS_ERROR_STREAM(ball_sdf_path_);
    ROS_ERROR_STREAM("TEST");
  }

 private:
  std::string ball_sdf_path_;
  physics::LinkPtr ball_launcher_link_ptr_;
  physics::ModelPtr model_ptr_;
  physics::ModelPtr ball_model_ptr_;
  physics::WorldPtr world_ptr_;
  volatile bool ball_exist_;
  ros::Duration ball_lifetime_;
  event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(ball_launcher_plugin)