#ifndef PARTICLE_FILTER_H_INCLUDED
#define PARTICLE_FILTER_H_INCLUDED

#include <Eigen/Core>
#include <Eigen/Geometry>

class particle_filter {
 public:
  particle_filter(int dimensions, int num_particles, Eigen::VectorXd init_value);
  particle_filter(int dimensions, int num_particles);
  ~particle_filter();
  Eigen::VectorXd get_state();
  Eigen::MatrixXd get_states() { return states_; };
  void set_weights(Eigen::VectorXd weights);
  void add_system_noise(Eigen::VectorXd& control_input, double variance);
  void add_system_noise(double variance);
  void resample(double threshold);

 private:
  int dimensions_;
  int num_partcles_;
  Eigen::MatrixXd states_;
  Eigen::VectorXd weights_;
  double get_ess() { return weights_.cwiseAbs2().sum(); };
  void clamp(Eigen::MatrixXd& target, double max, double min);
  void clamp(Eigen::VectorXd& target, double max, double min);
  void get_normal_distribution_random_numbers(Eigen::MatrixXd& target, double average, double variance);
  void get_normal_distribution_random_numbers(Eigen::VectorXd& target, double average, double variance);
  void get_uniform_distribution(Eigen::MatrixXd& target, double max, double min);
  void get_uniform_distribution(Eigen::VectorXd& target, double max, double min);
};
#endif  // PARTICLE_FILTER_H_INCLUDED