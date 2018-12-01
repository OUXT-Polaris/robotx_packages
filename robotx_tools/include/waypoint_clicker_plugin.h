#ifndef WAYPOINT_CLICKER_PLUGIN_H_INCLUDED
#define WAYPOINT_CLICKER_PLUGIN_H_INCLUDED

#include <memory>

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#include <QLabel>
#endif

#include <std_msgs/String.h>

namespace robotx_tools
{
class WaypointClickerPanel : public rviz::Panel
{
  Q_OBJECT
public:
  WaypointClickerPanel(QWidget *parent = 0);
private:
  // END_TUTORIAL
};

}  // robotx_tools

#endif // WAYPOINT_CLICKER_PLUGIN_H_INCLUDED