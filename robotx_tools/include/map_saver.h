#ifndef MAP_SAVER_H_INCLUDED
#define MAP_SAVER_H_INCLUDED

#include <tf2_ros/transform_listener.h>

class map_saver
{
public:
    map_saver(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~map_saver();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
};

#endif  //MAP_SAVER_H_INCLUDED