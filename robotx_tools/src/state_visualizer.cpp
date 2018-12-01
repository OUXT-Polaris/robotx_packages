#include <state_visualizer.h>

state_visualizer::state_visualizer(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    text_pub_ = pnh_.advertise<jsk_rviz_plugins::OverlayText>("text",1);
    control_state_sub_ptr_ = std::make_shared<message_filters::Subscriber<robotx_msgs::State> >
        (nh_,"/robotx_state_machine_node/control_state_machine/current_state",10);
    navigation_state_sub_ptr_ = std::make_shared<message_filters::Subscriber<robotx_msgs::State> >
        (nh_,"/robotx_state_machine_node/navigation_state_machine/current_state",10);
    mission_state_sub_ptr_ = std::make_shared<message_filters::Subscriber<robotx_msgs::State> >
        (nh_,"/robotx_state_machine_node/mission_state_machine/current_state",10);
    sync_ptr_ = std::make_shared<message_filters::Synchronizer<policy> >(policy(10),
        *control_state_sub_ptr_,*mission_state_sub_ptr_,*navigation_state_sub_ptr_);
    sync_ptr_->registerCallback(boost::bind(&state_visualizer::callback, this, _1, _2, _3));
}

state_visualizer::~state_visualizer()
{

}

void state_visualizer::callback(const robotx_msgs::StateConstPtr& control_state,
    const robotx_msgs::StateConstPtr& mission_state, const robotx_msgs::StateConstPtr& navigation_state)
{
    jsk_rviz_plugins::OverlayText text_msg;
    text_msg.text = "control_state : " + control_state->current_state +"\n"+
        "mission_state : " + mission_state->current_state + "\n"+
        "navigation_state : " + navigation_state->current_state + "\n";
    text_msg.action = text_msg.ADD;
    text_msg.height = 320;
    text_msg.width = 640;
    text_msg.fg_color.r = 0.3;
    text_msg.fg_color.g = 0.3;
    text_msg.fg_color.b = 1;
    text_msg.fg_color.a = 1;
    text_msg.bg_color.r = 0;
    text_msg.bg_color.g = 0;
    text_msg.bg_color.b = 0;
    text_msg.bg_color.a = 0.5;
    text_pub_.publish(text_msg);
    return;
}