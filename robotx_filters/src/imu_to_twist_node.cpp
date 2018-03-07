//headers for ros
#include <ros/ros.h>

#include <KalmanFilter/Kalman.h>

int main(int argc, char *argv[])
{
  Kalman kalmanX;
  Kalman kalmanY;
  ros::init(argc, argv, "imu_to_twist_node");
  //pcl_object_recognition object_recognition;
  ros::spin();
  return 0;
}
