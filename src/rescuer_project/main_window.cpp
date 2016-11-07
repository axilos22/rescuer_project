#include "rescuer_project/main_window.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
namespace rescuer_project {

MainWindow::MainWindow():rqt_gui_cpp::Plugin(),_centralWidget(0) {
    setObjectName("rescuer_gui");
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
    //init rosnode and map nodeHandle object
    //~ _nh = new ros::NodeHandle;
    _nh = &(getNodeHandle());
    _rate = new ros::Rate(30); //30ms rate
    //connect GUI
    connect(_ui.droneTakeOffButton,SIGNAL(pressed()),this,SLOT(droneTakeOff()));
    connect(_ui.droneLandButton,SIGNAL(pressed()),this,SLOT(droneLand()));
    connect(_ui.connectButton,SIGNAL(pressed()),this,SLOT(connectWithDrone()));
    connect(this,SIGNAL(batteryChanged(int)),_ui.batteryProgressBar,SLOT(setValue(int)));
}

void MainWindow::testCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("I heard: [%s]", msg->data.c_str());
	log("new msg");
}

void MainWindow::shutdownPlugin()
{
    // TODO unregister all publishers here
    for (int i = 0; i < _subs.size(); ++i) {
		_subs[i].shutdown();
	}
	for (int i = 0; i < _pubs.size(); ++i) {
		_pubs[i].shutdown();
	}
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
    ROS_DEBUG("Drone has to take off now.");
    log("Drone taking off.");
    sendEmptyCommand("/ardrone/takeoff");
}

void MainWindow::droneLand()
{
    ROS_DEBUG("Drone has to land now.");
    log("Drone landing");
    sendEmptyCommand("/ardrone/land");
}

void MainWindow::connectWithDrone()
{
    _droneNavDataSub = (*_nh).subscribe("/ardrone/navdata",1,&MainWindow::navDataCallback,this);
    //~ _subs.append(droneNavDataSub);
    ros::Subscriber testSub = (*_nh).subscribe("/test",1,&MainWindow::testCallback,this);
    //~ _subs.append(testSub);
    log("subbed into drone");
}

int MainWindow::sendEmptyCommand(QString commandTopic)
{
    std_msgs::Empty emptyMsg;
    ros::Publisher droneTakeOffPub = (*_nh).advertise<std_msgs::Empty>(commandTopic.toStdString(),1);
    for(int loop=0;loop<10;loop++) {
        if(!ros::ok())
            break;
        droneTakeOffPub.publish(emptyMsg);
        log("loop into: "+commandTopic);
        ros::spinOnce();
        _rate->sleep();
    }
}

void MainWindow::navDataCallback(const ardrone_autonomy::Navdata &navData)
{
	int bat =(int) navData.batteryPercent;
	emit batteryChanged(bat);
	log("Updated battery");
    //~ QVector<float> rot;
    //~ rot<<navData.rotX<<navData.rotY <<navData.rotZ;
    //~ QString displayed = "["+QString::number(rot.at(1))+";"+QString::number(rot.at(2))+";"+QString::number(rot.at(3))+"]";
    //~ QLineEdit* rotLe = _ui.rotXYZLineEdit;
    //~ rotLe->clear();
    //~ rotLe->insert(displayed);
    //~ _ui.batteryProgressBar->setValue(navData.batteryPercent);
    //~ log("update Navdata");
}
} // namespace
PLUGINLIB_EXPORT_CLASS(rescuer_project::MainWindow,rqt_gui_cpp::Plugin)
