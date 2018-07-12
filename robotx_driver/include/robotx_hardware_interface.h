#ifndef ROBOTX_HARDWARE_INTERFACE_H_INCLUDEDE
#define ROBOTX_HARDWARE_INTERFACE_H_INCLUDEDE

// headers in this package
#include <remote_operated_interface.h>
#include <tcp_client.h>

// headers in ROS
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8.h>

// headers in boost
#include <boost/thread.hpp>

// headers in STL
#include <mutex>

class robotx_hardware_interface {
 public:
  struct parameters {
    /**
     * @brief enum parameters describe driving mode.
     *
     */
    enum modes_ { REMOTE_OPERATED = 0, AUTONOMOUS = 1, EMERGENCY = 2 };
    /**
     * @brief enum parameters describe target.
     *
     */
    enum targets_ { ALL = 0, SIMULATION = 1, HARDWARE = 2 };
    /**
     * @brief hardware target
     * @sa robotx_hardware_interface::targets_
     */
    int target;
    /**
     * @brief action mode
     * @sa robotx_hardware_interface::modes_
     */
    int init_mode;
    /**
     * @brief connection timeout [sec] with motor controller
     *
     */
    int timeout;
    /**
     * @brief controller type
     * @sa robotx_hardware_interface::controllers_
     */
    int controller;
    /**
     * @brief frequency for publishing TCP/IP command.
     *
     */
    int frequency;
    /**
     * @brief right motor driver IP address
     *
     */
    std::string right_motor_ip;
    /**
     * @brief left motor driver IP address
     *
     */
    std::string left_motor_ip;
    /**
     * @brief right motor driver TCP/IP port
     *
     */
    int right_motor_port;
    /**
     * @brief left motor driver TCP/IP port
     *
     */
    int left_motor_port;
    /**
     * @brief parameter for team ID (default: OUXT Polaris)
     *
     */
    std::string team_id;
    /**
     * @brief set parameters
     *
     */
    parameters() {
      ros::param::param<int>(ros::this_node::getName() + "/target", target, ALL);
      ros::param::param<int>(ros::this_node::getName() + "/init_mode", init_mode, REMOTE_OPERATED);
      ros::param::param<int>(ros::this_node::getName() + "/timeout", timeout, 30);
      ros::param::param<int>(ros::this_node::getName() + "/frequency", frequency, 30);
      ros::param::param<std::string>(ros::this_node::getName() + "/left_motor_ip", left_motor_ip,
                                     "127.0.0.1");
      ros::param::param<std::string>(ros::this_node::getName() + "/right_motor_ip", right_motor_ip,
                                     "127.0.0.1");
      ros::param::param<int>(ros::this_node::getName() + "/left_motor_port", left_motor_port, 50000);
      ros::param::param<int>(ros::this_node::getName() + "/right_motor_port", right_motor_port, 50001);
      ros::param::param<std::string>(ros::this_node::getName() + "/team_id", team_id, "OUXT Polaris");
    };
  };
  robotx_hardware_interface();
  ~robotx_hardware_interface();
  /**
   * @brief function for setting action mode.
   *
   * @param mode target action mode
   */
  void set_action_mode(int mode);
  /**
   * @brief callback function for
   * remote_operated_interface::send_motor_command
   * function
   * @sa remote_operated_interface::send_motor_command_
   * @param msg
   */
  void recieve_remote_oprated_motor_command(std_msgs::Float64MultiArray msg);

 private:
  /**
   * @brief ROS callback function for motor command
   *
   * @param msg [left_thruster_cmd left_thruster_joint_angle
   * right_thruster_cmd
   * right_thruster_joint_angle]
   *
   * -1 < left_thruster_cmd < 1
   *
   *  -1 < right_thruster_cmd < 1
   */
  void motor_command_callback_(std_msgs::Float64MultiArray msg);
  /**
   * @brief ROS callback function for /fix (message type :
   * sensor_msgs/NavSatFix) topic.
   *
   * @param msg ROS message
   */
  void fix_callback_(sensor_msgs::NavSatFix msg);
  /**
   * @brief ROS callback function for /current_task_number topic.
   *
   * @param msg ROS message
   */
  void current_task_number_callback_(std_msgs::UInt8 msg);
  /**
   * @brief send command to motor_driver or simulation.
   *
   */
  void send_command_();
  /**
   * @brief publish heartbeat message to /heartbeat ROS topic
   *
   */
  void publish_heartbeat_();
  /**
   * @brief ROS nodehandle.
   *
   */
  ros::NodeHandle nh_;
  /**
   * @brief ROS subscriber for /wam_v/motor_command topic.(message type :
   * std_msgs/Float64MultiArray)
   *
   */
  ros::Subscriber motor_command_sub_;
  /**
   * @brief ROS subscriber for /fix topic.(message type :
   * sensot_msgs/NavSatFix)
   *
   */
  ros::Subscriber fix_sub_;
  /**
   * @brief ROS subscriber fot /current_task_number topic.(message type :
   * std_msgs/uint8)
   *
   */
  ros::Subscriber current_task_number_sub_;
  /**
   * @brief ROS publisher for /cmd_drive topic.(message type :
   * robotx_msgs/UsvDrive)
   *
   */
  ros::Publisher usv_drive_cmd_pub_;
  /**
   * @brief ROS publisher for /left_thruster_position_controller/command
   * topic.(message type : std_msgs/Float64)
   *
   */
  ros::Publisher left_thrust_joint_pub_;
  /**
   * @brief ROS publisher for /right_thruster_position_controller/command
   * topic.(message type : std_msgs/Float64)
   *
   */
  ros::Publisher right_thrust_joint_pub_;
  /**
   * @brief ROS publisher for ~/heartbeat topic.(message type :
   * robotx_msgs/Heartbeat)
   *
   */
  ros::Publisher heartbeat_pub_;
  /**
   * @brief ROS message for last motor command from robotx_control
   *
   */
  std_msgs::Float64MultiArray last_motor_cmd_msg_;
  /**
   * @brief message for last manual motor command.
   *
   */
  std_msgs::Float64MultiArray last_manual_motor_cmd_msg_;
  /**
   * @brief ROS message for last /fix (message type : sensor_msgs/NavSatFix)
   * message
   *
   */
  sensor_msgs::NavSatFix last_fix_msg_;
  /**
   * @brief TCP/IP client for the left motor.
   *
   */
  tcp_client *left_motor_cmd_client_ptr_;
  /**
   * @brief TCP/IP client for the right motor.
   *
   */
  tcp_client *right_motor_cmd_client_ptr_;
  /**
   * @brief io service fot TCP/IP client
   *
   */
  boost::asio::io_service io_service_;
  /**
   * @brief parameters for robotx_hardware_interface class
   *
   */
  const parameters params_;
  /**
   * @brief parameter for driving mode.
   *
   */
  volatile int driving_mode_;
  /**
   * @brief parameter for current task number.
   *
   */
  volatile int current_task_number_;
  /**
   * @brief mutex
   *
   */
  std::mutex mtx_;
  /**
   * @brief remote_operated_interface class
   *
   */
  remote_operated_interface remote_operated_if;
  /**
   * @brief send_command_thread
   *
   */
  boost::thread send_command_thread_;
  /**
   * @brief publish_heartbeat thread
   *
   */
  boost::thread publish_heartbeat_thread_;
  /**
   * @brief diagnostic updater for thruster
   *
   */
  diagnostic_updater::Updater thruster_diagnostic_updater_;
  /**
   * @brief diagnostic update function for left thruster
   * checke TCP/IP connection status
   *
   * @param stat
   */
  void update_left_thruster_connection_status_(diagnostic_updater::DiagnosticStatusWrapper &stat);
  /**
   * @brief diagnostic update function for right thruster
   * checke TCP/IP connection status
   *
   * @param stat
   */
  void update_right_thruster_connection_status_(diagnostic_updater::DiagnosticStatusWrapper &stat);
};

#endif  // ROBOTX_HARDWARE_INTERFACE_H_INCLUDEDE