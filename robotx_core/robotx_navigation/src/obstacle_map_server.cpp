#include <obstacle_map_server.h>
#include <tf/transform_datatypes.h>

obstacle_map_server::obstacle_map_server() : params_(), tf_listener_(tf_buffer_) {
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/obstacle_map", 1);
  measurements_ = boost::circular_buffer<jsk_recognition_msgs::BoundingBoxArray>(params_.buffer_length);
  objects_bbox_sub_ =
      nh_.subscribe(params_.object_bbox_topic, 1, &obstacle_map_server::objects_bbox_callback_, this);
}

obstacle_map_server::~obstacle_map_server() {}

void obstacle_map_server::objects_bbox_callback_(jsk_recognition_msgs::BoundingBoxArray msg) {
  measurements_.push_back(msg);
  generate_occupancy_grid_map_();
}

nav_msgs::OccupancyGrid obstacle_map_server::generate_occupancy_grid_map_() {
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = params_.robot_frame;
  map.header.stamp = measurements_[measurements_.size() - 1].header.stamp;
  map.info.map_load_time = ros::Time::now();
  map.info.height = params_.map_height;
  map.info.width = params_.map_width;
  map.info.resolution = params_.resolution;
  map.info.origin.position.x = -params_.map_height * params_.resolution / 2;
  map.info.origin.position.y = -params_.map_width * params_.resolution / 2;
  map.info.origin.position.z = params_.height_offset;
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, 0);
  map.info.origin.orientation.x = 0;
  map.info.origin.orientation.y = 0;
  map.info.origin.orientation.z = 0;
  map.info.origin.orientation.w = 1;
  std::vector<int8_t> map_data(params_.map_height * params_.map_width);
  std::vector<jsk_recognition_msgs::BoundingBoxArray> transformed_measurements;
  for (int i = 0; i < measurements_.size(); i++) {
    if (i == measurements_.size() - 1) {
      try {
        geometry_msgs::TransformStamped transform_stemped;
        transform_stemped =
            tf_buffer_.lookupTransform(measurements_[i].header.frame_id, params_.world_frame, ros::Time(0));
        jsk_recognition_msgs::BoundingBoxArray transformed_measurement;
        transformed_measurement.header.frame_id = params_.world_frame;
        transformed_measurement.header.stamp = measurements_[i].header.stamp;
        for (int m = 0; m < measurements_[i].boxes.size(); m++) {
          jsk_recognition_msgs::BoundingBox transformed_bbox;
          geometry_msgs::PoseStamped pose_stamped_in, pose_stamped_out;
          pose_stamped_in.header = measurements_[i].header;
          pose_stamped_in.pose = measurements_[i].boxes[m].pose;
          pose_stamped_out.header.frame_id = params_.world_frame;
          pose_stamped_out.header.stamp = measurements_[i].header.stamp;
          tf2::doTransform(pose_stamped_in, pose_stamped_out, transform_stemped);
          geometry_msgs::Vector3Stamped vector3_stamped_in, vector3_stamped_out;
          vector3_stamped_in.header = measurements_[i].header;
          vector3_stamped_in.vector = measurements_[i].boxes[m].dimensions;
          vector3_stamped_out.header.frame_id = params_.world_frame;
          vector3_stamped_out.header.stamp = measurements_[i].header.stamp;
          // tf2::doTransform(vector3_stamped_in, vector3_stamped_out,
          // transform_stemped);
          transformed_bbox.pose = pose_stamped_out.pose;
          // transformed_bbox.dimensions = vector3_stamped_out.vector;
          transformed_bbox.dimensions = vector3_stamped_in.vector;
          transformed_bbox.header.frame_id = measurements_[i].header.frame_id;
          transformed_bbox.header.stamp = measurements_[i].header.stamp;
          transformed_measurement.boxes.push_back(transformed_bbox);
        }
        transformed_measurements.push_back(transformed_measurement);
      } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT transform  %s", ex.what());
      }
    } else {
      try {
        geometry_msgs::TransformStamped transform_stemped;
        transform_stemped = tf_buffer_.lookupTransform(measurements_[i].header.frame_id, params_.world_frame,
                                                       measurements_[i].header.stamp);
        jsk_recognition_msgs::BoundingBoxArray transformed_measurement;
        transformed_measurement.header.frame_id = params_.world_frame;
        transformed_measurement.header.stamp = measurements_[i].header.stamp;
        for (int m = 0; m < measurements_[i].boxes.size(); m++) {
          jsk_recognition_msgs::BoundingBox transformed_bbox;
          geometry_msgs::PoseStamped pose_stamped_in, pose_stamped_out;
          pose_stamped_in.header = measurements_[i].header;
          pose_stamped_in.pose = measurements_[i].boxes[m].pose;
          pose_stamped_out.header.frame_id = params_.world_frame;
          pose_stamped_out.header.stamp = measurements_[i].header.stamp;
          tf2::doTransform(pose_stamped_in, pose_stamped_out, transform_stemped);
          geometry_msgs::Vector3Stamped vector3_stamped_in, vector3_stamped_out;
          vector3_stamped_in.header = measurements_[i].header;
          vector3_stamped_in.vector = measurements_[i].boxes[m].dimensions;
          vector3_stamped_out.header.frame_id = params_.world_frame;
          vector3_stamped_out.header.stamp = measurements_[i].header.stamp;
          // tf2::doTransform(vector3_stamped_in, vector3_stamped_out,
          // transform_stemped);
          transformed_bbox.pose = pose_stamped_out.pose;
          // transformed_bbox.dimensions = vector3_stamped_out.vector;
          transformed_bbox.dimensions = vector3_stamped_in.vector;
          transformed_bbox.header.frame_id = measurements_[i].header.frame_id;
          transformed_bbox.header.stamp = measurements_[i].header.stamp;
          transformed_measurement.boxes.push_back(transformed_bbox);
        }
        transformed_measurements.push_back(transformed_measurement);
      } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT transform  %s", ex.what());
      }
    }
  }
  geometry_msgs::TransformStamped transform_stemped;
  try {
    transform_stemped = tf_buffer_.lookupTransform(params_.world_frame, params_.robot_frame, ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Could NOT transform  %s", ex.what());
  }
  std::vector<std::array<double, 3>> objects_data;
  for (int i = 0; i < transformed_measurements.size(); i++) {
    for (int m = 0; m < transformed_measurements[i].boxes.size(); m++) {
      std::array<double, 3> object_data;
      double radius = std::max(transformed_measurements[i].boxes[m].dimensions.x,
                               transformed_measurements[i].boxes[m].dimensions.y) +
                      params_.margin;
      geometry_msgs::PoseStamped object_pose;
      object_pose.pose = transformed_measurements[i].boxes[m].pose;
      object_pose.header.frame_id = params_.robot_frame;
      object_pose.header.stamp = transform_stemped.header.stamp;
      tf2::doTransform(object_pose, object_pose, transform_stemped);
      double center_x = object_pose.pose.position.x;
      double center_y = object_pose.pose.position.y;
      object_data[0] = radius;
      object_data[1] = center_x;
      object_data[2] = center_y;
      objects_data.push_back(object_data);
    }
  }
  for (int i = 0; i < params_.map_height; i++) {
    for (int m = 0; m < params_.map_width; m++) {
      double x = (double)m * params_.resolution - 0.5 * params_.resolution * params_.map_width;
      double y = (double)i * params_.resolution - 0.5 * params_.resolution * params_.map_height;
      for (int n = 0; n < objects_data.size(); n++) {
        double distance =
            std::sqrt(std::pow(objects_data[n][1] - x, 2) + std::pow(objects_data[n][2] - y, 2));
        if (distance < objects_data[n][0]) map_data[i * params_.map_height + m] = 100;
      }
    }
  }
  map.data = map_data;
  map_pub_.publish(map);
  return map;
}