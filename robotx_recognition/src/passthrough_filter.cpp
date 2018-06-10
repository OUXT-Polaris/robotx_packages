#include <passthrough_filter.h>

//headers in PCL
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

passthough_filter::passthough_filter() : param_(passthough_filter::parameters())
{
    pointcloud_sub_ = nh_.subscribe(param_.input_cloud, 1, &passthough_filter::pointcloud_callback_, this);
}

void passthough_filter::pointcloud_callback_(sensor_msgs::PointCloud2 msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);
    if(param_.min_x == 0 && param_.max_x == 0){}
    else
    {
        pass.setFilterFieldName("x");
        pass.setFilterLimits(param_.min_x, param_.max_x);
    }
    if(param_.min_y == 0 && param_.max_y == 0){}
    else
    {
        pass.setFilterFieldName("y");
        pass.setFilterLimits(param_.min_y, param_.max_y);
    }
    if(param_.min_z == 0 && param_.max_z == 0){}
    else
    {
        pass.setFilterFieldName("z");
        pass.setFilterLimits(param_.min_x, param_.max_z);
    }
    pass.filter(*cloud_filtered);
}