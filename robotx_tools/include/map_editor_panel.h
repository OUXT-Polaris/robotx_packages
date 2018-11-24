#ifndef MAP_EFITOR_PANEL_H_INCLUDED
#define MAP_EFITOR_PANEL_H_INCLUDED

#ifndef Q_MOC_RUN
    #include <ros/ros.h>

    #include <rviz/panel.h>
    #include <QLabel>
    #include <QPushButton>
#endif

#include <geometry_msgs/PointStamped.h>

namespace robotx_tools
{
    class MapEditorWidget;

    class MapEditorPanel: public rviz::Panel
    {
    Q_OBJECT
    public:
        MapEditorPanel( QWidget* parent = 0 );
        virtual void load( const rviz::Config& config );
        virtual void save( rviz::Config config ) const;
    private:
        ros::NodeHandle nh_;
    };
}

#endif  //MAP_EFITOR_PANEL_H_INCLUDED