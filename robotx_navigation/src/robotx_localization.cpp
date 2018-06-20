#include <robotx_localization.h>

robotx_localization::robotx_localization() : params_()
{
    Eigen::VectorXd init_value = Eigen::VectorXd::Ones(3);
    init_value = init_value * 0.5;
    std::vector<bool> is_circular(3);
    is_circular[0] = false;
    is_circular[1] = false;
    is_circular[2] = true;
    pfilter_ptr_ = new particle_filter(3,params_.num_particles,init_value,is_circular);
    fix_recieved_ = false;
    twist_received_ = false;
    fix_sub_ = nh_.subscribe(params_.fix_topic, 1, &robotx_localization::fix_callback_, this);
    twist_sub_ = nh_.subscribe(params_.twist_topic, 1, &robotx_localization::twist_callback_, this);
}

robotx_localization::~robotx_localization()
{

}

void robotx_localization::fix_callback_(sensor_msgs::NavSatFix msg)
{
    if(fix_recieved_ == false)
    {
        init_measurement_ = msg;
    }
    last_fix_message_ = msg;
    fix_recieved_ = true;
}

void robotx_localization::twist_callback_(geometry_msgs::Twist msg)
{
    last_twist_message_ = msg;
    twist_received_ = true;
}