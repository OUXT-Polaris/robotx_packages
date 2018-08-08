#ifndef LOAD_PARAMS_HH_INCLUDED
#define LOAD_PARAMS_HH_INCLUDED

// Gazebo
#include <ros/ros.h>
#include <gazebo/common/common.hh>

template <typename T>
bool LoadParams(sdf::ElementPtr sdf, std::string key, T &param) {
  std::string param_str;
  if (!sdf->HasElement(key)) {
    ROS_WARN_STREAM("failed to get " << key);
    return false;
  } else {
    param_str = sdf->GetElement(key)->GetValue()->GetAsString();
  }
  try {
    param = boost::lexical_cast<T>(param_str);
  } catch (boost::bad_lexical_cast &) {
    ROS_WARN_STREAM("failed to casting " << key);
  }
  return true;
}

template <typename T>
bool LoadParams(sdf::ElementPtr sdf, std::string key, T &param, T default_param) {
  std::string param_str;
  if (!sdf->HasElement(key)) {
    param = default_param;
    ROS_INFO_STREAM("unable to get " << key << " ,set default_value = " << default_param);
    return false;
  } else {
    param_str = sdf->GetElement(key)->GetValue()->GetAsString();
  }
  try {
    param = boost::lexical_cast<T>(param_str);
  } catch (boost::bad_lexical_cast &) {
    ROS_WARN_STREAM("failed to casting " << key);
    param = default_param;
  }
  return true;
}

#endif  // LOAD_PARAMS_HH_INCLUDED