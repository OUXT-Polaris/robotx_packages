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
#include <geometry_msgs/Quaternion.h>
#include <robotx_msgs/BallLauncherStatus.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

namespace gazebo {
class ball_launcher_plugin : public ModelPlugin {
 public:
  ball_launcher_plugin();
  virtual ~ball_launcher_plugin();
  /*! Loads the model in gets dynamic parameters from SDF. */
  void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf);
  void OnUpdate(const common::UpdateInfo &_info);

 private:
  void spawn_ball(int ball_id);
  void delete_ball();
  void load_ball_urdf();
  // launch command subscriber
  void ball_launcher_callback(std_msgs::Empty msg);
  void get_rpy(geometry_msgs::Quaternion &q, double &roll, double &pitch, double &yaw);
  ros::Subscriber launch_sub_;

  // model parameters
  std::string ball_sdf_path_;
  physics::LinkPtr ball_launcher_link_ptr_;
  gazebo::math::Pose ball_launcher_link_pose_;
  physics::ModelPtr model_ptr_;
  physics::ModelPtr ball_model_ptr_;
  physics::WorldPtr world_ptr_;
  ros::Duration ball_lifetime_;
  event::ConnectionPtr update_connection_;
  int num_balls_;
  int ball_remains_;
  int count_;
  // initial speed of the ball
  double ball_v_init_;
  // volatile bool ball_exist_;

  // ROS nodehandle
  ros::NodeHandle nh_;
  // ROS Publisher
  ros::Publisher ball_launcher_status_pub_;
  // spanm model service client
  ros::ServiceClient spawn_client_;
  ros::Time last_spawm_time_;
  std::string ball_urdf_str_;
};  // class ball_launcher_plugin
}  // namespace gazebo

#endif  // BALL_LAUNCHER_PLUGIN_H
