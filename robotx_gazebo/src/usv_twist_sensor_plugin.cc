/**
 * @mainpage gazebo plugin for twist sensor simulation
 * @image html images/vmrc.jpg
 * @author Masaya Kataoka
 * @date 2018-06-09
 */

//headers for stl
#include <sstream>
#include <random>
//headers for gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
//headers for sdf
#include <sdf/Param.hh>
#include <sdf/sdf.hh>
//headers fot boost
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
//headers for ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace gazebo;

/**
 * @brief gazebo twist sensor plugin class
 * 
 */
class twist_sensor_plugin : public ModelPlugin
{
  /**
   * @brief get target link(default: base_link) pointer amd connect to gazebo
   * 
   */
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
  {
    this->model = _parent;
    this->LoadParams(sdf,"targetLink",this->target_link,std::string("base_link"));
    this->LoadParams(sdf,"sensorNoiseVariance",this->sensor_noise_variance,0.1);
    this->LoadParams(sdf,"publishRate",this->publish_rate,50.0);
    this->link = this->model->GetLink(target_link);
    twist_pub = nh.advertise<geometry_msgs::Twist>("/vel", 1);
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&twist_sensor_plugin::OnUpdate, this, _1));
    boost::thread publisher_thread(boost::bind(&twist_sensor_plugin::publish_twist,this));
  }

  /**
   * @brief callback function which was called when the simulation updates.
   * 
   */
  public: void OnUpdate(const common::UpdateInfo & /*_info*/)
  {
    angular_vel = this->link->GetRelativeAngularVel();
    linear_vel = this->link->GetRelativeLinearVel();
  }

  /**
   * @brief publish geometry_msgs/Twist type to /vel topic
   * 
   */
  public: void publish_twist()
  {
    ros::Rate loop_rate(this->publish_rate);
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0,this->sensor_noise_variance);
    while(ros::ok())
    {
      geometry_msgs::Twist twist_msg;
      twist_msg.linear.x = linear_vel.x + distribution(generator);
      twist_msg.linear.y = linear_vel.y + distribution(generator);
      twist_msg.angular.z = angular_vel.z + distribution(generator);
      twist_pub.publish(twist_msg);
      loop_rate.sleep();
    }
  }

  /** 
   * @brief Load parameters from sdf element.
   * 
   * @tparam T 
   * @param sdf sdf model.
   * @param key query key.
   * @param param loaded parameters.
   * @return true : success to find target.
   * @return false : fail to find target.
   */
  template <typename T>
  bool LoadParams(sdf::ElementPtr sdf,std::string key,T& param)
  {
    std::string param_str;
    if(!sdf->HasElement(key))
    {
      ROS_WARN_STREAM("failed to get " << key);
      return false;
    }
    else
    {
      param_str = sdf->GetElement(key)->GetValue()->GetAsString();
    }
    try
    {
      param = boost::lexical_cast<T>(param_str);
    }
    catch(boost::bad_lexical_cast &)
    {
      ROS_WARN_STREAM("failed to casting " << key);
    }
    return true;
  }

  /**
   * @brief Load parameters from sdf element.
   * 
   * @tparam T 
   * @param sdf sdf model.
   * @param key query key.
   * @param param loaded parameters.
   * @param default_param default parameters.
   * @return true : success to find target.
   * @return false : failed to find target. (param = default_param)
   */
  template <typename T>
  bool LoadParams(sdf::ElementPtr sdf,std::string key,T& param, T default_param)
  {
    std::string param_str;
    if(!sdf->HasElement(key))
    {
      param = default_param;
      ROS_INFO_STREAM("unable to get " << key << " ,set default_value = " << default_param);
      return false;
    }
    else
    {
      param_str = sdf->GetElement(key)->GetValue()->GetAsString();
    }
    try
    {
      param = boost::lexical_cast<T>(param_str);
    }
    catch(boost::bad_lexical_cast &)
    {
      ROS_WARN_STREAM("failed to casting " << key);
      param = default_param;
    }
    return true;
  }
  /**
   * @brief name of target link
   * 
   */
  private: std::string target_link;
  /**
   * @brief pointer of target link
   * 
   */
  private: physics::LinkPtr link;
  /**
   * @brief pointer of target model
   * 
   */
  private: physics::ModelPtr model;
  /**
   * @brief parameter for noise variance.
   * sensor noise modeled as a normal distribution.
   * 
   */
  private: double sensor_noise_variance;
  /**
   * @brief parameter for sensor publish rate.
   * 
   */
  private: double publish_rate;
  /**
   * @brief ROS nodehandle.
   * 
   */
  private: ros::NodeHandle nh;
  /**
   * @brief ROS publisher for /vel topic (geometry_msgs/Twist)
   * 
   */
  private: ros::Publisher twist_pub;
  /**
   * @brief parameter for angular velocity
   * 
   */
  private: math::Vector3 angular_vel;
  /**
   * @brief parameter for linear velocity
   * 
   */
  private: math::Vector3 linear_vel;
  /**
   * @brief update_connection with gazebo
   * 
   */
  private: event::ConnectionPtr update_connection;
};

GZ_REGISTER_MODEL_PLUGIN(twist_sensor_plugin)
