#ifndef LAT_LON_RULER_PANEL_H_INCLUDED
#define LAT_LON_RULER_PANEL_H_INCLUDED

#ifndef Q_MOC_RUN
    #include <ros/ros.h>

    #include <rviz/panel.h>
    #include <QLabel>
    #include <QLineEdit>
#endif

#include <UTM.h>
#include <geometry_msgs/PointStamped.h>

namespace robotx_tools
{
    class MapEditorWidget;

    class LatLonRulerPanel: public rviz::Panel
    {
    Q_OBJECT
    public:
        LatLonRulerPanel( QWidget* parent = 0 );
        virtual void load( const rviz::Config& config );
        virtual void save( rviz::Config config ) const;
    private:
        ros::NodeHandle nh_;
    };
}

#endif  //LAT_LON_RULER_PANEL_H_INCLUDED