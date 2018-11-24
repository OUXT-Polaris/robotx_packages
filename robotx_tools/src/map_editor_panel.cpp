#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include "map_editor_panel.h"

namespace robotx_tools
{
  MapEditorPanel::MapEditorPanel( QWidget* parent )
    : rviz::Panel( parent )
  {
    QVBoxLayout *layout = new QVBoxLayout;
    QPushButton *load_button = new QPushButton("Load Map Data");
    layout->addWidget(load_button);
    setLayout( layout );
  }

  void MapEditorPanel::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
  }

  void MapEditorPanel::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
  }
} // end namespace robotx_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robotx_tools::MapEditorPanel,rviz::Panel )