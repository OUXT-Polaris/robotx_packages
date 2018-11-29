#include <field_map_clicker.h>

field_map_clicker::field_map_clicker() : tf_listener_(tf_buffer_)
{
    nh_ = ros::NodeHandle();
    pnh_ = ros::NodeHandle("~");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    field_map_.header.frame_id = map_frame_;
    field_map_pub_ = nh_.advertise<robotx_msgs::FieldMap>("/field_map",10);
    green_buoy_sub_ = nh_.subscribe("/green_buoy/plant/point",10,&field_map_clicker::green_buoy_callback_,this);
    red_buoy_sub_ = nh_.subscribe("/red_buoy/plant/point",10,&field_map_clicker::red_buoy_callback_,this);
    white_buoy_sub_ = nh_.subscribe("/white_buoy/plant/point",10,&field_map_clicker::white_buoy_callback_,this);
}

field_map_clicker::~field_map_clicker()
{

}

void field_map_clicker::green_buoy_callback_(const geometry_msgs::PointStamped::ConstPtr msg)
{
    field_map_.header.stamp = msg->header.stamp;
    geometry_msgs::PointStamped transformed_point;
    geometry_msgs::TransformStamped transform_stamped;
    if(msg->header.frame_id != map_frame_)
    {
        try
        {
            transform_stamped = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            return;
        }
        tf2::doTransform(*msg,transformed_point,transform_stamped);
    }
    else
    {
        transformed_point = *msg;
    }
    field_map_.green_buoys.push_back(transformed_point.point);
    save_and_publish_();
    return;
}

void field_map_clicker::red_buoy_callback_(const geometry_msgs::PointStamped::ConstPtr msg)
{
    field_map_.header.stamp = msg->header.stamp;
    geometry_msgs::PointStamped transformed_point;
    geometry_msgs::TransformStamped transform_stamped;
    if(msg->header.frame_id != map_frame_)
    {
        try
        {
            transform_stamped = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            return;
        }
        tf2::doTransform(*msg,transformed_point,transform_stamped);
    }
    else
    {
        transformed_point = *msg;
    }
    field_map_.red_buoys.push_back(transformed_point.point);
    save_and_publish_();
    return;
}

void field_map_clicker::white_buoy_callback_(const geometry_msgs::PointStamped::ConstPtr msg)
{
    field_map_.header.stamp = msg->header.stamp;
    geometry_msgs::PointStamped transformed_point;
    geometry_msgs::TransformStamped transform_stamped;
    if(msg->header.frame_id != map_frame_)
    {
        try
        {
            transform_stamped = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            return;
        }
        tf2::doTransform(*msg,transformed_point,transform_stamped);
    }
    else
    {
        transformed_point = *msg;
    }
    field_map_.white_buoys.push_back(transformed_point.point);
    save_and_publish_();
    return;
}

void field_map_clicker::save_and_publish_()
{
    std::ofstream csv_file;
    std::string green_buoy_csv = ros::package::getPath("robotx_navigation") +"/data/green_buoys.csv";
    csv_file.open(green_buoy_csv, std::ios::trunc);
    for(auto buoy_itr = field_map_.green_buoys.begin(); buoy_itr != field_map_.green_buoys.end(); buoy_itr++)
    {
        std::string line_str = std::to_string(buoy_itr->x) + "," + std::to_string(buoy_itr->y) + "," + std::to_string(buoy_itr->z);
        csv_file << line_str << std::endl;
    }
    csv_file.close();
    std::string red_buoy_csv = ros::package::getPath("robotx_navigation") +"/data/red_buoys.csv";
    csv_file.open(red_buoy_csv, std::ios::trunc);
    for(auto buoy_itr = field_map_.red_buoys.begin(); buoy_itr != field_map_.red_buoys.end(); buoy_itr++)
    {
        std::string line_str = std::to_string(buoy_itr->x) + "," + std::to_string(buoy_itr->y) + "," + std::to_string(buoy_itr->z);
        csv_file << line_str << std::endl;
    }
    csv_file.close();
    std::string white_buoy_csv = ros::package::getPath("robotx_navigation") +"/data/white_buoys.csv";
    csv_file.open(white_buoy_csv, std::ios::trunc);
    for(auto buoy_itr = field_map_.white_buoys.begin(); buoy_itr != field_map_.white_buoys.end(); buoy_itr++)
    {
        std::string line_str = std::to_string(buoy_itr->x) + "," + std::to_string(buoy_itr->y) + "," + std::to_string(buoy_itr->z);
        csv_file << line_str << std::endl;
    }
    csv_file.close();
    field_map_pub_.publish(field_map_);
    return;
}