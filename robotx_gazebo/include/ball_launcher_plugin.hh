#ifndef BALL_LAUNCHER_PLUGIN_H
#define BALL_LAUNCHER_PLUGIN_H

// headers in gazebo
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// #include <gazebo/math/gzmath.hh>
// #include <gazebo/physics/Link.hh>
// #include <gazebo/physics/Model.hh>

// headers in ROS
#include <ros/package.h>
#include <ros/ros.h>

namespace gazebo {
  class ball_launcher_plugin : public ModelPlugin {
  public:
    ball_launcher_plugin();
    virtual ~ball_launcher_plugin();
    
    /*! Loads the model in gets dynamic parameters from SDF. */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf);

    void OnUpdate(const common::UpdateInfo& _info);

  private:
    std::string ball_sdf_path_;
    physics::LinkPtr ball_launcher_link_ptr_;
    physics::ModelPtr model_ptr_;
    physics::ModelPtr ball_model_ptr_;
    physics::WorldPtr world_ptr_;
    volatile bool ball_exist_;
    ros::Duration ball_lifetime_;
    event::ConnectionPtr update_connection_;
  }; // class ball_launcher_plugin
} // namespace gazebo

#endif // BALL_LAUNCHER_PLUGIN_H
