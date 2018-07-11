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
class ball_launcher : public ModelPlugin {
 public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
  {
    ball_urdf_path_ =
        ros::package::getPath("robotx_gazebo") + "/modeles/ball/ball.urdf";
    std::string default_ball_launcher_link_name = "ball_launcher_pitch_link";
    std::string ball_launcher_link_name;
    LoadParams(sdf, "target_link", ball_launcher_link_name,
               default_ball_launcher_link_name);
    model_ptr_ = _parent;
    this->ball_launcher_link_ptr_ =
        this->model_ptr_->GetLink(ball_launcher_link_name);
  }
  void OnUpdate(const common::UpdateInfo& /*_info*/) {}
 private:
  std::string ball_urdf_path_;
  physics::LinkPtr ball_launcher_link_ptr_;
  physics::ModelPtr model_ptr_;
};

GZ_REGISTER_MODEL_PLUGIN(ball_launcher)