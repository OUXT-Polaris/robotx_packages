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
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(new QLabel("Lat"));
    layout->addWidget(new QLineEdit());
    layout->addWidget(new QLabel("Lon"));
    layout->addWidget(new QLineEdit());
    setLayout( layout );
  }

  void LatLonRulerPanel::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
  }

  void LatLonRulerPanel::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
  }
} // end namespace robotx_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robotx_tools::LatLonRulerPanel,rviz::Panel )