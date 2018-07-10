#include <robotx_localization.h>

robotx_localization::robotx_localization() : params_() {
  Eigen::VectorXd init_value = Eigen::VectorXd::Ones(3);
  init_value = init_value * 0.5;
  std::vector<bool> is_circular(3);
  is_circular[0] = false;
  is_circular[1] = false;
  is_circular[2] = true;
  pfilter_ptr_ = new particle_filter(3, params_.num_particles, init_value, is_circular);
  fix_recieved_ = false;
  twist_received_ = false;
  robot_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot_pose", 1);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
  init_fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/origin/fix", 1);
  fix_sub_ = nh_.subscribe(params_.fix_topic, 1, &robotx_localization::fix_callback_, this);
  twist_sub_ = nh_.subscribe(params_.twist_topic, 1, &robotx_localization::twist_callback_, this);
  thread_update_frame_ = boost::thread(boost::bind(&robotx_localization::update_frame_, this));
}

robotx_localization::~robotx_localization() { thread_update_frame_.join(); }

void robotx_localization::update_frame_() {
  ros::Rate rate(params_.publish_rate);
  while (fix_recieved_ == false) {
    rate.sleep();
  }
  while (ros::ok()) {
    std::lock(fix_mutex_, twist_mutex_);
    // critical section start
    pfilter_ptr_->resample(params_.ess_threshold);
    Eigen::VectorXd control_input(3);
    Eigen::VectorXd state = pfilter_ptr_->get_state();
    Eigen::VectorXd position(3);
    position(0) = params_.min_x + state(0) * (params_.max_x - params_.min_x);
    position(1) = params_.min_y + state(1) * (params_.max_y - params_.min_y);
    position(2) = 2 * M_PI * state(2);
    control_input(0) = (std::cos(position(2)) * last_twist_message_.linear.x -
                        std::sin(position(2)) * last_twist_message_.linear.y) /
                       params_.publish_rate;
    control_input(1) = (std::sin(position(2)) * last_twist_message_.linear.x +
                        std::cos(position(2)) * last_twist_message_.linear.y) /
                       params_.publish_rate;
    control_input(2) = last_twist_message_.angular.z / params_.publish_rate;
    control_input(0) = control_input(0) / (params_.max_x - params_.min_x);
    control_input(1) = control_input(1) / (params_.max_y - params_.min_y);
    control_input(2) = control_input(2) / (2 * M_PI);
    pfilter_ptr_->add_system_noise(control_input, 0);
    Eigen::MatrixXd states = pfilter_ptr_->get_states();
    double measurement_x = (last_fix_message_.longitude - init_measurement_.longitude) * 111263.283 /
                           (params_.max_x - params_.min_x);
    double measurement_y = (last_fix_message_.latitude - init_measurement_.latitude) * 6378150 *
                           std::cos(last_fix_message_.longitude / 180 * M_PI) * 2 * M_PI / (360 * 60 * 60) /
                           (params_.max_y - params_.min_y);
    Eigen::VectorXd weights(params_.num_particles);
    for (int i = 0; i < params_.num_particles; i++) {
      double error =
          std::sqrt(std::pow(states(0, i) - measurement_x, 2) + std::pow(states(1, i) - measurement_y, 2));
      double threashold = 0.01;
      // avoid zero division
      if (std::fabs(error) < threashold) error = threashold;
      weights(i) = 1 / error;
    }
    pfilter_ptr_->set_weights(weights);
    Eigen::VectorXd predicted_pos = pfilter_ptr_->get_state();
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = params_.publish_frame;
    transform_stamped.child_frame_id = params_.robot_frame;
    transform_stamped.transform.translation.x =
        predicted_pos(0) * (params_.max_x - params_.min_x) + params_.min_x;
    transform_stamped.transform.translation.y =
        predicted_pos(1) * (params_.max_y - params_.min_y) + params_.min_y;
    transform_stamped.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, predicted_pos(2) * 2 * M_PI);
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();
    broadcaster_.sendTransform(transform_stamped);
    geometry_msgs::PoseStamped robot_pose_msg;
    robot_pose_msg.header = transform_stamped.header;
    robot_pose_msg.pose.position.x = predicted_pos(0) * (params_.max_x - params_.min_x) + params_.min_x;
    robot_pose_msg.pose.position.y = predicted_pos(1) * (params_.max_y - params_.min_y) + params_.min_y;
    robot_pose_msg.pose.position.z = 0;
    robot_pose_msg.pose.orientation.x = q.x();
    robot_pose_msg.pose.orientation.y = q.y();
    robot_pose_msg.pose.orientation.z = q.z();
    robot_pose_msg.pose.orientation.w = q.w();
    robot_pose_pub_.publish(robot_pose_msg);
    nav_msgs::Odometry odom_msg;
    odom_msg.header = robot_pose_msg.header;
    odom_msg.child_frame_id = params_.robot_frame;
    odom_msg.pose.pose = robot_pose_msg.pose;
    odom_msg.twist.twist = last_twist_message_;
    odom_pub_.publish(odom_msg);
    init_fix_pub_.publish(init_measurement_);
    // critical section end
    fix_mutex_.unlock();
    twist_mutex_.unlock();
    rate.sleep();
  }
}

void robotx_localization::fix_callback_(sensor_msgs::NavSatFix msg) {
  std::lock_guard<std::mutex> lock(fix_mutex_);
  if (fix_recieved_ == false) {
    init_measurement_ = msg;
  }
  last_fix_message_ = msg;
  fix_recieved_ = true;
}

void robotx_localization::twist_callback_(geometry_msgs::Twist msg) {
  std::lock_guard<std::mutex> lock(twist_mutex_);
  last_twist_message_ = msg;
  twist_received_ = true;
}