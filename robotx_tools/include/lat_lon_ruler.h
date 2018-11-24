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
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

//headers in Boost
#include <boost/optional.hpp>

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
        void draw_line_();
        QLineEdit* lat_box_;
        QLineEdit* lon_box_;
        boost::optional<double> lat_;
        boost::optional<double> lon_;
        ros::Publisher ruler_pub_;
    protected Q_SLOTS:
        void update_lat_();
        void update_lon_();
    };
}

#endif  //LAT_LON_RULER_PANEL_H_INCLUDED