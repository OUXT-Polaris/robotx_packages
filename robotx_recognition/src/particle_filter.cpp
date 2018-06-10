#include <ros/ros.h>
#include <particle_filter.h>
#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

particle_filter::particle_filter(int dimensions, int num_particles,Eigen::VectorXd init_value)
{
  using namespace Eigen;
  std::srand((unsigned int) time(0));
  dimensions_ = dimensions;
  num_partcles_ = num_particles;
  Eigen::MatrixXd random_values = (MatrixXd::Random(dimensions_,num_partcles_).cwiseAbs()-MatrixXd::Ones(dimensions_,num_partcles_)*0.5)*0.1;
  states_ = MatrixXd::Ones(dimensions_,num_partcles_);
  for(int i = 0; i < num_partcles_ ;i++)
  {
    states_.block(0,i,dimensions_,1)= init_value;
  }
  states_ = states_ + random_values;
  weights_ = VectorXd::Ones(num_particles)/num_particles;
}

particle_filter::particle_filter(int dimensions, int num_particles)
{
  using namespace Eigen;
  std::srand((unsigned int) time(0));
  dimensions_ = dimensions;
  num_partcles_ = num_particles;
  states_ = MatrixXd::Random(dimensions_,num_partcles_).cwiseAbs();
  weights_ = VectorXd::Ones(num_particles)/num_particles;
}

particle_filter::~particle_filter()
{

}

void particle_filter::set_weights(Eigen::VectorXd weights)
{
  weights_ = weights/weights.sum();
}

Eigen::VectorXd particle_filter::get_state()
{
  using namespace Eigen;
  VectorXd normalized_state = VectorXd::Zero(dimensions_);
  for(int i = 0; i < num_partcles_ ;i++)
  {
    VectorXd state = states_.block(0,i,dimensions_,1);
    normalized_state = normalized_state + state * weights_(i);
  }
  clamp(states_,1,0);
  return normalized_state;
}

void particle_filter::clamp(Eigen::VectorXd& target, double max, double min)
{
  for(int i = 0; i < target.size(); i++)
  {
    target(i) = std::min(max, std::max(min, target(i)));
  }
}

void particle_filter::clamp(Eigen::MatrixXd& target, double max, double min)
{
  for(int i = 0; i < target.rows(); i ++)
  {
    for(int j = 0; j < target.cols(); j ++)
    {
      target(i, j) = std::min(max, std::max(min, target(i, j)));
    }
  }
}

void particle_filter::add_system_noise(double variance)
{
  using namespace Eigen;
  MatrixXd system_noise = MatrixXd::Zero(dimensions_,num_partcles_);
  get_normal_distribution_random_numbers(system_noise,0,variance);
  states_ = states_ + system_noise;
  clamp(states_,1,0);
}

void particle_filter::add_system_noise(Eigen::VectorXd& control_input, double variance)
{
  using namespace Eigen;
  MatrixXd system_noise = MatrixXd::Zero(dimensions_,num_partcles_);
  get_normal_distribution_random_numbers(system_noise,0,variance);
  MatrixXd control_input_matrix = MatrixXd::Zero(dimensions_,num_partcles_);
  for(int i = 0; i < num_partcles_ ;i++)
  {
    control_input_matrix.block(0,i,dimensions_,1) = control_input;
  }
  states_ = states_ + system_noise + control_input_matrix;
  clamp(states_,1,0);
}

void particle_filter::resample(double threshold)
{
  if(get_ess() <= threshold)
  {
    using namespace Eigen;
    MatrixXd updated_state = MatrixXd::Zero(dimensions_,num_partcles_);
    VectorXd random_values = VectorXd::Zero(num_partcles_);
    get_normal_distribution_random_numbers(random_values,0,1);
    std::vector<int> indexes(num_partcles_);
    for(int i = 0; i<num_partcles_; i++)
    {
        double total_weights = 0;
        for(int j = 0; j<weights_.size(); j++)
        {
        total_weights += weights_(j);
        if(total_weights >= random_values[i])
        {
            indexes[i] = i;
        }
        }
    }
    for(int i = 0; i<num_partcles_; i++)
    {
        updated_state.block(0,i,dimensions_,1) = states_.block(0,indexes[i],dimensions_,1);
    }
    states_ = updated_state;
  }
}

void particle_filter::get_normal_distribution_random_numbers(Eigen::MatrixXd& target, double average, double variance)
{
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::normal_distribution<> norm(average, variance);
  for(int i = 0; i < target.rows(); i ++)
  {
    for(int j = 0; j < target.cols(); j ++)
    {
      target(i, j) = norm(mt);
    }
  }
}

void particle_filter::get_normal_distribution_random_numbers(Eigen::VectorXd& target, double average, double variance)
{
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::normal_distribution<> norm(average, variance);
  for(int i = 0; i < target.size(); i++)
  {
    target(i) = norm(mt);
  }
}

void particle_filter::get_uniform_distribution(Eigen::VectorXd& target, double max, double min)
{
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::uniform_int_distribution<> random_values(0,10000);
  for(int i = 0; i < target.size(); i++)
  {
    target(i) = ((double)random_values(mt)/10000)*(max-min)+min;
  }
}

void particle_filter::get_uniform_distribution(Eigen::MatrixXd& target, double max, double min)
{
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::uniform_int_distribution<> random_values(0,10000);
  for(int i = 0; i < target.rows(); i ++)
  {
    for(int j = 0; j < target.cols(); j ++)
    {
      target(i, j) = ((double)random_values(mt)/10000)*(max-min)+min;
    }
  }
}
