#ifndef NAVI_SIM_H_INCLUDED
#define NAVI_SIM_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <geodesy/utm.h>

//headers in this package
//#include <UTM.h>

//headers in STL
#include <mutex>

//headers in boost
#include <boost/optional.hpp>

class navi_sim
{
public:
    navi_sim();
    ~navi_sim();
    void run();
private:
    std::mutex mtx_;
    geometry_msgs::Twist twist_cmd_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber twist_cmd_sub_;
    ros::Subscriber init_pose_sub_;
    ros::Publisher fix_pub_;
    ros::Publisher true_course_pub_;
    ros::Publisher gps_twist_pub_;
    boost::optional<geometry_msgs::Pose2D> current_pose_;
    geometry_msgs::Twist current_twist_;
    std::string gps_frame_;
    std::string world_frame_;
    std::string fix_topic_;
    std::string gps_twist_topic_;
    std::string true_course_topic_;
    bool southhemi_;
    int utm_zone_;
    double update_rate_;
    void update_gps_();
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr msg);
    void init_pose_callback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};
#endif  //NAVI_SIM_H_INCLUDED