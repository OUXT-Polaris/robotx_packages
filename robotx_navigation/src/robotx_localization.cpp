#include <robotx_localization.h>

robotx_localization::robotx_localization() : params_()
{
    transition_matrix_ = Eigen::MatrixXd::Identity(3,3);
    input_matrix_ = Eigen::MatrixXd::Zero(3,3);
    measurement_matrix_ = Eigen::MatrixXd::Identity(2,3);
    Eigen::VectorXd system_noise_vec = Eigen::VectorXd(3);
    system_noise_vec[0] = params_.system_noise_x;
    system_noise_vec[1] = params_.system_noise_y;
    system_noise_vec[2] = params_.system_noise_theta;
    system_noise_matrix_ = system_noise_vec.asDiagonal();
    system_noise_matrix_ = system_noise_matrix_ * system_noise_matrix_;
    Eigen::VectorXd measurement_noise_vec = Eigen::VectorXd(2);
    measurement_noise_vec[0] = params_.measurement_noise_x;
    measurement_noise_vec[1] = params_.measurement_noise_y;
    measurement_noise_matrix_ = measurement_noise_vec.asDiagonal();
    measurement_noise_matrix_ = measurement_noise_matrix_ * measurement_noise_matrix_;
    fix_recieved = false;
    twist_received = false;
    fix_sub_ = nh_.subscribe(params_.fix_topic, 1, &robotx_localization::fix_callback_, this);
    twist_sub_ = nh_.subscribe(params_.twist_topic, 1, &robotx_localization::twist_callback_, this);
}

robotx_localization::~robotx_localization()
{

}

void robotx_localization::fix_callback_(sensor_msgs::NavSatFix msg)
{
    if(fix_recieved == false)
    {
        init_measurement_ = msg;
    }
    fix_recieved = true;
}

void robotx_localization::twist_callback_(geometry_msgs::Twist msg)
{
    twist_received = true;
}