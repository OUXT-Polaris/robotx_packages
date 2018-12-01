
#include <QPainter>
#include <QVBoxLayout>

#include <waypoint_clicker_plugin.h>

namespace robotx_tools
{
WaypointClickerPanel::WaypointClickerPanel(QWidget *parent) : rviz::Panel(parent)
{
}

}  // robotx_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robotx_tools::WaypointClickerPanel, rviz::Panel)