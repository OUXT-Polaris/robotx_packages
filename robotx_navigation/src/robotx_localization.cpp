#include <robotx_localization.h>

robotx_localization::robotx_localization() : params_()
{
    //pfilter_ptr_ = new particle_filter(3,params_.num_particles_,);
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