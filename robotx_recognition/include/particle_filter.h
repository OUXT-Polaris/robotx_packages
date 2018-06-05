#ifndef PARTICLE_FILTER_H_INCLUDED
#define PARTICLE_FILTER_H_INCLUDED

#include <Eigen/Core>
#include <Eigen/Geometry>

class particle_filter
{
public:
  particle_filter(int dimensions, int num_partcles, Eigen::VectorXd init_value);
  particle_filter(int dimensions, int num_partcles);
  ~particle_filter();
  Eigen::VectorXd get_normalized_state();
  Eigen::MatrixXd get_states();
  void set_weights(Eigen::VectorXd weights);
  void add_system_noise(Eigen::VectorXd& control_input, double variance);
  void add_system_noise(double variance);
  void resample();
private:
  int dimensions_;
  int num_partcles_;
  Eigen::MatrixXd states_;
  Eigen::VectorXd weights_;
  void clamp(Eigen::MatrixXd& target, double max, double min);
  void clamp(Eigen::VectorXd& target, double max, double min);
  void get_normal_distribution_random_numbers(Eigen::MatrixXd& target, double average, double variance);
  void get_normal_distribution_random_numbers(Eigen::VectorXd& target, double average, double variance);
  void get_uniform_distribution(Eigen::MatrixXd& target, double max, double min);
  void get_uniform_distribution(Eigen::VectorXd& target, double max, double min);
};
#endif //PARTICLE_FILTER_H_INCLUDED