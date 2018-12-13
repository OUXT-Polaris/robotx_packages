#ifndef FIELD_FRAME_PUBLISHER_H_INCLUDED
#define FIELD_FRAME_PUBLISHER_H_INCLUDED

#include <ros/ros.h>
#include <robotx_tools/field_frame_publisherConfig.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/tf.h>

class field_frame_publisher
{
public:
    field_frame_publisher();
    ~field_frame_publisher();
private:
    void configure_callback_(robotx_tools::field_frame_publisherConfig &config, uint32_t level);
    dynamic_reconfigure::Server<robotx_tools::field_frame_publisherConfig> server_;
    dynamic_reconfigure::Server<robotx_tools::field_frame_publisherConfig>::CallbackType callback_func_type_;
    tf2_ros::StaticTransformBroadcaster broadcaster_;
    geometry_msgs::Quaternion convert_(double r,double p, double y);
};
#endif  //FIELD_FRAME_PUBLISHER_H_INCLUDED