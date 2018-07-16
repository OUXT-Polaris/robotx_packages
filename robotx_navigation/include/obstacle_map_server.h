#ifndef OBSTACLE_MAP_SERVER_H_INCLUDED
#define OBSTACLE_MAP_SERVER_H_INCLUDED

// headers in ROS
#include <geometry_msgs/TransformStamped.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// headers in Boost
#include <boost/circular_buffer.hpp>

/**
 * @brief obstacle_map_server class
 *
 * this class recoeve /euclidian_clustering_node/marker topic and publish
 * /obstacle_map topic
 */
class obstacle_map_server {
  struct parameters {
    /**
     * @brief margin of objects [m]
     *
     */
    double margin;
    /**
     * @brief map resolution [m/cell]
     *
     */
    double resolution;
    /**
     * @brief height offset of the map[m]
     *
     */
    double height_offset;
    /**
     * @brief map width [cells]
     *
     */
    int map_width;
    /**
     * @brief map height [cells]
     *
     */
    int map_height;
    /**
     * @brief ROS topic name for object bounding box
     *
     */
    std::string object_bbox_topic;
    /**
     * @brief buffer length of measurement data
     *
     */
    int buffer_length;
    /**
     * @brief name of world frame
     *
     */
    std::string world_frame;
    /**
     * @brief name of robot frame
     *
     */
    std::string robot_frame;
    /**
     * @brief Construct a new parameters object
     *
     */
    parameters() {
      ros::param::param<double>(ros::this_node::getName() + "/margin", margin, 0.2);
      ros::param::param<double>(ros::this_node::getName() + "/resolution", resolution, 0.05);
      ros::param::param<double>(ros::this_node::getName() + "/height_offset", height_offset, 2.5);
      ros::param::param<int>(ros::this_node::getName() + "/map_height", map_height, 400);
      ros::param::param<int>(ros::this_node::getName() + "/map_width", map_width, 400);
      ros::param::param<int>(ros::this_node::getName() + "/buffer_length", buffer_length, 10);
      ros::param::param<std::string>(ros::this_node::getName() + "/object_bbox_topic", object_bbox_topic,
                                     ros::this_node::getName() + "/object_bbox");
      ros::param::param<std::string>(ros::this_node::getName() + "/world_frame", world_frame,
                                     ros::this_node::getName() + "/world_frame");
      ros::param::param<std::string>(ros::this_node::getName() + "/robot_frame", robot_frame,
                                     ros::this_node::getName() + "/robot_frame");
    }
  };

 public:
  /**
   * @brief Construct a new obstacle map server object
   *
   */
  obstacle_map_server();
  /**
   * @brief Destroy the obstacle map server object
   *
   */
  ~obstacle_map_server();

 private:
  /**
   * @brief parameters for obstacle_map_server class
   *
   */
  const parameters params_;
  /**
   * @brief ROS NodeHandle
   *
   */
  ros::NodeHandle nh_;
  /**
   * @brief ROS subscriber for (object_bbox_topic) topic (message type :
   * jsk_recognition_msgs/BoundingBoxArray)
   *
   */
  ros::Subscriber objects_bbox_sub_;
  /**
   * @brief ROS publisher for /obstacle_map topic (message type :
   * nav_msgs/OccupancyGrid)
   *
   */
  ros::Publisher map_pub_;
  /**
   * @brief ROS callback function for (object_bbox_topic) topic (message type :
   * jsk_recognition_msgs/BoundingBoxArray)
   *
   * @param msg ROS message (message type :
   * jsk_recognition_msgs/BoundingBoxArray)
   * @sa obstacle_map_server::objects_bbox_sub_
   */
  void objects_bbox_callback_(jsk_recognition_msgs::BoundingBoxArray msg);
  /**
   * @brief function for generating occupancy grid map
   *
   * @return nav_msgs::OccupancyGrid
   */
  nav_msgs::OccupancyGrid generate_occupancy_grid_map_();
  /**
   * @brief transform buffer
   *
   */
  tf2_ros::Buffer tf_buffer_;
  /**
   * @brief transform listener
   *
   */
  tf2_ros::TransformListener tf_listener_;
  /**
   * @brief buffer of measurement data
   *
   */
  boost::circular_buffer<jsk_recognition_msgs::BoundingBoxArray> measurements_;
};

#endif  // OBSTACLE_MAP_SERVER_H_INCLUDED