#include <field_frame_publisher.h>

field_frame_publisher::field_frame_publisher()
{
    callback_func_type_ = boost::bind(&field_frame_publisher::configure_callback_, this, _1, _2);
    server_.setCallback(callback_func_type_);
}

field_frame_publisher::~field_frame_publisher()
{

}

void field_frame_publisher::configure_callback_(robotx_tools::field_frame_publisherConfig &config, uint32_t level)
{
    ros::Time now = ros::Time::now();
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = "map";
    transform_stamped.header.stamp = now;
    transform_stamped.child_frame_id = "field";
    transform_stamped.transform.translation.x = config.map_to_field_x;
    transform_stamped.transform.translation.y = config.map_to_field_y;
    transform_stamped.transform.translation.z = 0;
    transform_stamped.transform.rotation = convert_(0,0,config.map_to_field_theta*M_PI);
    broadcaster_.sendTransform(transform_stamped);
    return;
}

geometry_msgs::Quaternion field_frame_publisher::convert_(double r,double p, double y)
{
    geometry_msgs::Quaternion ret;
    tf::Quaternion quat = tf::createQuaternionFromRPY(r,p,y);
    quaternionTFToMsg(quat,ret);
    return ret;
}