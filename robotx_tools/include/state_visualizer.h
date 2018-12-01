#ifndef STATE_VISUALIZER_H_INCLUDED
#define STATE_VISUALIZER_H_INCLUDED

#include <robotx_msgs/State.h>

//headers in ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <jsk_rviz_plugins/OverlayText.h>

//headers in STL
#include <memory>

typedef message_filters::sync_policies::ApproximateTime<robotx_msgs::State,robotx_msgs::State,robotx_msgs::State> policy;

class state_visualizer
{
public:
    state_visualizer(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~state_visualizer();
private:
    ros::Publisher text_pub_;
    void navigation_state_callback_();
    std::shared_ptr<message_filters::Subscriber<robotx_msgs::State> > navigation_state_sub_ptr_;
    void control_state_callback_();
    std::shared_ptr<message_filters::Subscriber<robotx_msgs::State> > control_state_sub_ptr_;
    void mission_state_callback_();
    std::shared_ptr<message_filters::Subscriber<robotx_msgs::State> > mission_state_sub_ptr_;
    std::shared_ptr<message_filters::Synchronizer<policy> > sync_ptr_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    void callback(const robotx_msgs::StateConstPtr& control_state,
        const robotx_msgs::StateConstPtr& mission_state, const robotx_msgs::StateConstPtr& navigation_state);
};

#endif  //STATE_VISUALIZER_H_INCLUDED