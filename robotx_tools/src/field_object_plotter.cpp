#include <field_object_plotter.h>

field_object_plotter::field_object_plotter() : tf_listener_(tf_buffer_)
{

}

field_object_plotter::~field_object_plotter()
{
    std::ofstream csv_file;
    std::string path = ros::package::getPath("robotx_tools") +"/data/tmp/field_objects.csv";
    csv_file.open(path, std::ios::trunc);
    for(auto object_itr = objects_.begin(); object_itr != objects_.end(); object_itr++)
    {
        std::string line_str = object_itr->first + ",map," + std::to_string(object_itr->second.pose.position.x) + ","
            + std::to_string(object_itr->second.pose.position.y);
        csv_file << line_str << std::endl;
    }
    csv_file.close();
}

void field_object_plotter::roi_callback_(const robotx_msgs::ObjectRegionOfInterestArray::ConstPtr msg)
{
    for(auto roi_itr = msg->object_rois.begin(); roi_itr != msg->object_rois.end(); roi_itr++)
    {
        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_.lookupTransform("map", roi_itr->roi_3d.header.frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            return;
        }
        geometry_msgs::PoseStamped transformed_pose;
        geometry_msgs::PoseStamped roi_pose;
        roi_pose.header = roi_itr->roi_3d.header;
        roi_pose.pose = roi_itr->roi_3d.pose;
        tf2::doTransform(roi_pose,transformed_pose,transform_stamped);
        if(roi_itr->object_type.ID == robotx_msgs::ObjectType::OTHER)
        {
            continue;
        }
        if(roi_itr->object_type.ID == robotx_msgs::ObjectType::RED_BUOY)
        {
            std::pair<std::string,geometry_msgs::PoseStamped> data;
            data.first = "red_buoy";
            data.second = transformed_pose;
            objects_.push_back(data);
        }
        if(roi_itr->object_type.ID == robotx_msgs::ObjectType::GREEN_BUOY)
        {
            std::pair<std::string,geometry_msgs::PoseStamped> data;
            data.first = "green_buoy";
            data.second = transformed_pose;
            objects_.push_back(data);
        }
        if(roi_itr->object_type.ID == robotx_msgs::ObjectType::WHITE_BUOY)
        {
            std::pair<std::string,geometry_msgs::PoseStamped> data;
            data.first = "white_buoy";
            data.second = transformed_pose;
            objects_.push_back(data);
        }
    }
    return;
}