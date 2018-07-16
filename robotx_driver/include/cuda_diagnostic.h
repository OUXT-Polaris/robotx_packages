#ifndef CUDA_DIAGNOSTIC_H_INCLUDED
#define CUDA_DIAGNOSTIC_H_INCLUDED

// headers in ROS
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

// headers in CUDA
#include <cuda_runtime.h>
#include <nvml.h>

// headers in STL
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <iostream>
#include <memory>

/**
 * @brief cuda diagnostic class
 * it publish status of cuda devices
 */
class cuda_diagnostic {
 public:
  /**
   * @brief parameters for cuda_diagnostic class
   *
   */
  struct parameters {
    /**
     * @brief device id (use in diagnostic_updater)
     *
     */
    std::string device_id;
    /**
     * @brief update frequency of diagnostic
     *
     */
    double update_frequency;
    /**
     * @brief threshold of low or enought memory usage
     *
     */
    double low_memory_usage_threshold;
    /**
     * @brief threashold of temperature error
     *
     */
    double temperature_error_threshold;
    /**
     * @brief threashold of temperature warning
     *
     */
    double temperature_warn_threshold;
    /**
     * @brief name of GPU device in
     * /sys/devices/virtual/thermal/thermal_zone(asterisk)/temp command
     *
     */
    std::string gpu_device_name;
    parameters() {
      ros::param::param<std::string>(ros::this_node::getName() + "/device_id", device_id, "cuda_device");
      ros::param::param<double>(ros::this_node::getName() + "/update_frequency", update_frequency, 1);
      ros::param::param<double>(ros::this_node::getName() + "/low_memory_usage_threshold",
                                low_memory_usage_threshold, 0.2);
      ros::param::param<double>(ros::this_node::getName() + "/temperature_error_threshold",
                                temperature_error_threshold, 100);
      ros::param::param<double>(ros::this_node::getName() + "/temperature_warn_threshold",
                                temperature_warn_threshold, 80);
      ros::param::param<std::string>(ros::this_node::getName() + "/gpu_device_name", gpu_device_name,
                                     "gpu_device");
    }
  };
  cuda_diagnostic();
  ~cuda_diagnostic();
  void run();

 private:
  ros::NodeHandle nh_;
  diagnostic_updater::Updater updater_;
  const parameters params_;
  /**
   * @brief callback function for GPU memory usage.
   *
   * @param stat
   */
  void update_memory_usage_(diagnostic_updater::DiagnosticStatusWrapper &stat);
  /**
   * @brief callback function for GPU temperature
   *
   * @param stat
   */
  void update_temperature_(diagnostic_updater::DiagnosticStatusWrapper &stat);
  /**
   * @brief execute shell command
   *
   * @param cmd shell command
   * @param stdOut execution result
   * @param exitCode exit code
   * @return true success on executing comand.
   * @return false failed on executing comand.
   */
  bool exec_shell_cmd(const char *cmd, std::string &stdOut, int &exitCode);
};
#endif  // CUDA_DIAGNOSTIC_H_INCLUDED