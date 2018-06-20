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
    thread_update_frame_ = boost::thread(boost::bind(&robotx_localization::update_frame_, this));
}

robotx_localization::~robotx_localization()
{
    thread_update_frame_.join();
}

void robotx_localization::update_frame_()
{
    ros::Rate rate(params_.publish_rate);
    while(ros::ok())
    {
        std::lock(fix_mutex_,twist_mutex_);
        //critical section start
        
        //critical section end
        fix_mutex_.unlock();
        twist_mutex_.unlock();
        rate.sleep();
    }
}

void robotx_localization::fix_callback_(sensor_msgs::NavSatFix msg)
{
    std::lock_guard<std::mutex> lock(fix_mutex_);
    if(fix_recieved_ == false)
    {
        init_measurement_ = msg;
    }
    last_fix_message_ = msg;
    fix_recieved_ = true;
}

void robotx_localization::twist_callback_(geometry_msgs::Twist msg)
{
    std::lock_guard<std::mutex> lock(twist_mutex_);
    last_twist_message_ = msg;
    twist_received_ = true;
}