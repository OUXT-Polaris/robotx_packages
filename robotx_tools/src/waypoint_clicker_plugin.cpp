#include <QComboBox>
#include <QPainter>
#include <QVBoxLayout>

#include <waypoint_clicker_plugin.h>

namespace robotx_tools
{
    WaypointClickerPanel::WaypointClickerPanel(QWidget *parent) : rviz::Panel(parent)
    {
        QStringList commands = 
            { "mission_start", "demonstrate_navigation_and_control","go_to_start_position", "entrance_gates", 
            "scan_the_code", "exit_gates", "avoid_obstacles","find_totems", "identify_symbols_and_dock",
            "return_to_home","mission_complete"};
        QVBoxLayout *layout = new QVBoxLayout(this);
        layout->addWidget(new QLabel("Task"));
        QComboBox* combo = new QComboBox();
        combo->addItems(commands);
        layout->addWidget(combo);
        setLayout(layout);
        //connect( combo, &QComboBox::currentTextChanged, this, &MainWindow::commandChanged);
    }

}  // robotx_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robotx_tools::WaypointClickerPanel, rviz::Panel)