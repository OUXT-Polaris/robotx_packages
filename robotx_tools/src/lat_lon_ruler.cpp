#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include "lat_lon_ruler.h"

namespace robotx_tools
{
  LatLonRulerPanel::LatLonRulerPanel( QWidget* parent )
    : rviz::Panel( parent )
  {
    ruler_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/lat_lon_ruler/marker" ,1);
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(new QLabel("Lat"));
    lat_box_ = new QLineEdit("21.311");
    connect(lat_box_, SIGNAL( editingFinished() ), this, SLOT( update_lat_() ));
    layout->addWidget(lat_box_);
    layout->addWidget(new QLabel("Lon"));
    lon_box_ = new QLineEdit("-157.89");
    layout->addWidget(lon_box_);
    connect(lon_box_, SIGNAL( editingFinished() ), this, SLOT( update_lon_() ));
    setLayout(layout);
  }

  void LatLonRulerPanel::save( rviz::Config config ) const
  {
    rviz::Panel::save(config);
    return;
  }

  void LatLonRulerPanel::load( const rviz::Config& config )
  {
    rviz::Panel::load(config);
    return;
  }

  void LatLonRulerPanel::draw_line_()
  {
    if(lat_ && lon_)
    {
      double x,y;
      LatLonToUTMXY(*lat_, *lon_, 0, x, y);
      visualization_msgs::MarkerArray msg;
      visualization_msgs::Marker lat_marker;
      lat_marker.header.frame_id = "world";
      lat_marker.header.stamp = ros::Time::now();
      lat_marker.frame_locked = true;
      lat_marker.type = lat_marker.CYLINDER;
      lat_marker.action = lat_marker.ADD;
      lat_marker.id = 0;
      lat_marker.pose.position.x = x;
      lat_marker.pose.position.y = y;
      lat_marker.pose.position.z = 0;
      tf::Quaternion quaternion_lat = tf::createQuaternionFromRPY(0,M_PI/2,0);
      tf::quaternionTFToMsg(quaternion_lat, lat_marker.pose.orientation);
      lat_marker.scale.x = 1.0;
      lat_marker.scale.y = 1.0;
      lat_marker.scale.z = 1000.0;
      lat_marker.color.r = 0;
      lat_marker.color.g = 1.0;
      lat_marker.color.b = 0.0;
      lat_marker.color.a = 1.0;
      lat_marker.ns = "latitude";
      msg.markers.push_back(lat_marker);
      visualization_msgs::Marker lon_marker;
      lon_marker.header.frame_id = "world";
      lon_marker.header.stamp = ros::Time::now();
      lon_marker.frame_locked = true;
      lon_marker.type = lon_marker.CYLINDER;
      lon_marker.action = lon_marker.ADD;
      lon_marker.id = 1;
      lon_marker.pose.position.x = x;
      lon_marker.pose.position.y = y;
      lon_marker.pose.position.z = 0;
      tf::Quaternion quaternion_lon = tf::createQuaternionFromRPY(M_PI/2,0,0);
      tf::quaternionTFToMsg(quaternion_lon, lon_marker.pose.orientation);
      lon_marker.scale.x = 1.0;
      lon_marker.scale.y = 1.0;
      lon_marker.scale.z = 1000.0;
      lon_marker.color.r = 0;
      lon_marker.color.g = 1.0;
      lon_marker.color.b = 0.0;
      lon_marker.color.a = 1.0;
      lon_marker.ns = "longitude";
      msg.markers.push_back(lon_marker);
      ruler_pub_.publish(msg);
    }
    else
    {
      visualization_msgs::MarkerArray msg;
      ruler_pub_.publish(msg);
    }
    return;
  }

  void LatLonRulerPanel::update_lat_()
  {
    std::string lat_str = lat_box_->text().toStdString();
    try
    {
      lat_ = std::stod(lat_str);
    }
    catch(...)
    {
      lat_ = boost::none;
    }
    draw_line_();
    return;
  }

  void LatLonRulerPanel::update_lon_()
  {
    std::string lon_str = lon_box_->text().toStdString();
    try
    {
      lon_ = std::stod(lon_str);
    }
    catch(...)
    {
      lon_ = boost::none;
    }
    draw_line_();
    return;
  }
} // end namespace robotx_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robotx_tools::LatLonRulerPanel,rviz::Panel )