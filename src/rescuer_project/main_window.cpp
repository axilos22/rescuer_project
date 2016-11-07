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
    //Adding custom types for ROS data transmission
    qRegisterMetaType<QVector<float> >("QVector<float>");
    //connect GUI
    connect(_ui.droneTakeOffButton,SIGNAL(pressed()),this,SLOT(droneTakeOff()));
    connect(_ui.droneLandButton,SIGNAL(pressed()),this,SLOT(droneLand()));
    connect(_ui.connectButton,SIGNAL(pressed()),this,SLOT(connectWithDrone()));
    connect(this,SIGNAL(batteryUpdated(int)),_ui.batteryProgressBar,SLOT(setValue(int)));
    connect(this,SIGNAL(rotDataUpdated(QVector<float>)),this,SLOT(updateRotValues(QVector<float>)));
    //~ connect(this,SIGNAL(camImgUpdated(QPixmap)),_ui.droneCamLabel,SLOT(setPixmap(QPixmap)));
}

void MainWindow::testCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void MainWindow::cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
	ROS_INFO("Camera callback");
	//~ try {
		//~ cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
	//~ }
	//~ catch(cv_bridge::Exception e) {
        //~ ROS_ERROR("Could not convert from %s to bgr8.",msg->encoding.c_str());
    //~ }
    //~ try {
		//~ cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
		//~ _conversionMat = cv_ptr->image;
		//~ ROS_INFO("Converted to cv:Mat");
        //~ QImage img(_conversionMat.data,_conversionMat.cols,_conversionMat.rows,_conversionMat.step[0],QImage::Format_RGB888);
        //~ QPixmap pix;
        //~ ROS_INFO("Now put QImage into QPixmap");
        //~ if(pix.convertFromImage(img)) {
            //~ emit camImgUpdated(pix);
        //~ } else {
            //~ ROS_ERROR("Failed converting QImage into QPixmap");
        //~ }
    //~ }

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

void MainWindow::connectWithDrone()
{
    ros::NodeHandle nh = getNodeHandle();
    ROS_DEBUG("@connect with drone");
    ros::Subscriber droneNavDataSub = nh.subscribe("/ardrone/navdata",1,&MainWindow::navDataCallback,this);
    ROS_DEBUG("Subbed to navdata");
    ros::Subscriber testSub = nh.subscribe("/test",1,&MainWindow::testCallback,this);
    ROS_DEBUG("Subbed to test sub");
    _subs.append(droneNavDataSub);
    _subs.append(testSub);
//    _it = new image_transport::ImageTransport(nh);
//    _itSub = new image_transport::Subscriber(_it->subscribe("/ardrone/image_raw",1,&MainWindow::cameraCallback,this));
//    image_transport::ImageTransport it = image_transport::ImageTransport(nh);
//    image_transport::Subscriber sub = it.subscribe("/ardrone/image_raw",1,&MainWindow::cameraCallback,this);
//    _it = &it;
//    _itSub = &sub;
//    ROS_INFO("Subbed to the camera");
    log("Drone connected.");
    ROS_INFO("-- all subs Ok");
}

void MainWindow::updateRotValues(QVector<float> rotV)
{
    QString displayed = "["+QString::number(rotV.at(0))+","+QString::number(rotV.at(1))+","+QString::number(rotV.at(2))+"]";
    QLineEdit* rotLe = _ui.rotXYZLineEdit;
    rotLe->clear();
    rotLe->setText(displayed);
}
/**
 * @brief MainWindow::navDataCallback
 * @param navData
 * Note: this is a ROS callbacl: not possible to access Qt objects.
 */
void MainWindow::navDataCallback(const ardrone_autonomy::Navdata &navData)
{
    emit batteryUpdated((int)navData.batteryPercent);
    QVector<float> rot;
    rot<<navData.rotX<<navData.rotY <<navData.rotZ;
    emit rotDataUpdated(rot);
}
} // namespace
PLUGINLIB_EXPORT_CLASS(rescuer_project::MainWindow,rqt_gui_cpp::Plugin)
