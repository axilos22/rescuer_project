#include "rescuer_project/main_window.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace rescuer_project {

MainWindow::MainWindow():rqt_gui_cpp::Plugin(),_centralWidget(0) {
    setObjectName("MainWindow0");
}

void MainWindow::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    _centralWidget = new QWidget;
    _ui.setupUi(_centralWidget);
    _console = _ui.consoleTextEdit;
    _console->setText("Console");
    // add widget to the user interface
    context.addWidget(_centralWidget);
    //TODO check if rosnode is set
    //init rosnode and send nodeHandle object
    ros::NodeHandle nh;
    _nh = &nh;
    //connect GUI
    connect(_ui.droneTakeOffButton,SIGNAL(released()),this,SLOT(droneTakeOff()));
    connect(_ui.droneLandButton,SIGNAL(released()),this,SLOT(droneLand()));
}

void MainWindow::shutdownPlugin()
{
    // TODO unregister all publishers here
}

void MainWindow::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    // TODO save intrinsic configuration, usually using:
    // instance_settings.setValue(k, v)
}

void MainWindow::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    // TODO restore intrinsic configuration, usually using:
    // v = instance_settings.value(k)
}

void MainWindow::log(QString msg)
{
    QString oldText = _console->toPlainText();
    _console->setText(oldText+"\n>>"+msg);
}

void MainWindow::droneTakeOff()
{
    ROS_INFO("Drone has to take off now.");
    log("Drone has to take off now.");
    std_msgs::Empty emptyMsg;
    ros::Publisher droneTakeOffPub = (*_nh).advertise<std_msgs::Empty>("/ardrone/takeoff",1);
    droneTakeOffPub.publish(emptyMsg);
}

void MainWindow::droneLand()
{
    ROS_INFO("Drone has to land now.");
    log("Drone has to land now.");
    std_msgs::Empty emptyMsg;
    ros::Publisher droneLandPub = (*_nh).advertise<std_msgs::Empty>("/ardrone/land",1);
    droneLandPub.publish(emptyMsg);
}
} // namespace
PLUGINLIB_EXPORT_CLASS(rescuer_project::MainWindow,rqt_gui_cpp::Plugin)
