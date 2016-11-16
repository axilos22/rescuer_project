#include "rescuer_project/main_window.h"
namespace rescuer_project {

MainWindow::MainWindow():rqt_gui_cpp::Plugin(),
    _centralWidget(0),
    _defaultSpeed(.2),
    m_isConnected(false),
    m_droneState(0)
{
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
    _console->setText("Console\n");
    // add widget to the user interface
    context.addWidget(_centralWidget);
    //init rosnode and map nodeHandle object
    _nh = &(getNodeHandle());
    _rate = new ros::Rate(30); //30ms rate
    //Adding custom types for ROS data transmission
    qRegisterMetaType<QVector<float> >("QVector<float>");
    //connect GUI
    connect(_ui.droneTakeOffButton,SIGNAL(pressed()),this,SLOT(droneTakeOff()));
    connect(_ui.droneLandButton,SIGNAL(pressed()),this,SLOT(droneLand()));
    connect(_ui.connectButton,SIGNAL(pressed()),this,SLOT(connectWithDrone()));
    connect(_ui.switchCamPushButton,SIGNAL(pressed()),this,SLOT(swapCamera()));
    connect(_ui.flatTrimButton,SIGNAL(pressed()),this,SLOT(flatTrim()));
    connect(_ui.autoHoverButton,SIGNAL(pressed()),this,SLOT(activateAutoHoverMode()));
    connect(_ui.resetButton,SIGNAL(pressed()),this,SLOT(droneReset()));
    connect(this,SIGNAL(batteryUpdated(int)),_ui.batteryProgressBar,SLOT(setValue(int)));
    connect(this,SIGNAL(rotDataUpdated(QVector<float>)),this,SLOT(updateRotValues(QVector<float>)));
    connect(this,SIGNAL(camImgUpdated(QPixmap)),_ui.droneCamLabel,SLOT(setPixmap(QPixmap)));
    connect(this,SIGNAL(camRescuerImgUpdated(QPixmap)),_ui.rescuerCamLabel,SLOT(setPixmap(QPixmap)));
    connect(this,SIGNAL(droneStateChanged(int)),_ui.stateSpinBox,SLOT(setValue(int)));
    connect(this,SIGNAL(velDataUpdated(QVector<float>)),this,SLOT(updateVValues(QVector<float>)));
    connect(this,SIGNAL(altUpdated(int)),_ui.altSpinBox,SLOT(setValue(int)));
    connect(this,SIGNAL(tagCountUpdated(int)),_ui.tagCountSpinBox,SLOT(setValue(int)));
    //Adding teleoperator
    _teleop = new LineEditTeleop;
    _teleop->setEnabled(false);
    _ui.formLayout->addRow(tr("&Teleop:"),_teleop);    
    //Drone teleoperation
    connect(this,SIGNAL(isConnectedChanged(bool)),_teleop,SLOT(setEnabled(bool)));
    connect(_teleop,SIGNAL(pageUpPressed()),this,SLOT(droneUp()));
    connect(_teleop,SIGNAL(pageDownPressed()),this,SLOT(droneDown()));
    connect(_teleop,SIGNAL(upPressed()),this,SLOT(droneForward()));
    connect(_teleop,SIGNAL(downPressed()),this,SLOT(droneBackward()));
    connect(_teleop,SIGNAL(leftPressed()),this,SLOT(droneLeft()));
    connect(_teleop,SIGNAL(rightPressed()),this,SLOT(droneRight()));
    connect(_teleop,SIGNAL(homePressed()),this,SLOT(droneTakeOff()));
    connect(_teleop,SIGNAL(endPressed()),this,SLOT(droneLand()));
    connect(_teleop,SIGNAL(keyReleased()),this,SLOT(activateAutoHoverMode()));
}

void MainWindow::testCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/**
 * @brief MainWindow::cameraRescuerCallback
 * @param msg
 */
void MainWindow::cameraRescuerCallback(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_DEBUG("Camera rescuer callback");
    try {
        cv_bridge::CvImageConstPtr cvPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        _conversionMat=cvPtr->image;
        ROS_DEBUG("Converted image");
        QImage img(_conversionMat.data,_conversionMat.cols,_conversionMat.rows,_conversionMat.step[0],QImage::Format_RGB888);
        if(_cameraPixmap.convertFromImage(img)) {
            emit camRescuerImgUpdated(_cameraPixmap);
        } else {
            ROS_ERROR("Failed putting image into pixmap");
        }
    } catch(cv_bridge::Exception e) {
        ROS_ERROR("Could not convert from %s to bgr8.",msg->encoding.c_str());
    }
}

/**
 * @brief MainWindow::cameraCallback
 * @param msg
 * Drone cam info :Drone 2: 640x360@20fps H264 codec with no record stream
 */
void MainWindow::cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_DEBUG("Camera callback");
    try {
        cv_bridge::CvImageConstPtr cvPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        _conversionMat=cvPtr->image;
        ROS_DEBUG("Converted image");
        QImage img(_conversionMat.data,_conversionMat.cols,_conversionMat.rows,_conversionMat.step[0],QImage::Format_RGB888);
        if(_cameraPixmap.convertFromImage(img)) {
            emit camImgUpdated(_cameraPixmap);
        } else {
            ROS_ERROR("Failed putting image into pixmap");
        }
    } catch(cv_bridge::Exception e) {
        ROS_ERROR("Could not convert from %s to bgr8.",msg->encoding.c_str());
    }
}

void MainWindow::shutdownPlugin()
{
    //Unregister all publishers here
    for (int i = 0; i < _subs.size(); ++i) {
        _subs[i].shutdown();
    }
    for (int i = 0; i < _pubs.size(); ++i) {
        _pubs[i].shutdown();
    }
    if(_itSub)
        _itSub->shutdown();
    if(_itSubRescuer)
        _itSubRescuer->shutdown();
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
    _console->setText(msg+"\n"+oldText);
}

int MainWindow::droneState() const
{
    return m_droneState;
}

QString MainWindow::format3Data(const QVector<float> tab)
{
    return "["+QString::number(tab.at(0))+","+QString::number(tab.at(1))+","+QString::number(tab.at(2))+"]";
}

bool MainWindow::isConnected() const
{
    return m_isConnected;
}

void MainWindow::droneTakeOff()
{
    ROS_DEBUG("Drone has to take off now.");
    log("Drone taking off.");
    std_msgs::Empty emptyMsg;
    ros::Publisher droneTakeOffPub = getNodeHandle().advertise<std_msgs::Empty>("/ardrone/takeoff",1);
    int loop=0;
    while(ros::ok() && droneState()!=6 && loop<100) {
        droneTakeOffPub.publish(emptyMsg);
        loop++;
        ros::spinOnce();
        _rate->sleep();
    }
}

void MainWindow::droneLand()
{
    ROS_DEBUG("Drone has to land now.");
    log("Drone landing");
    std_msgs::Empty emptyMsg;
    ros::Publisher droneTakeOffPub = getNodeHandle().advertise<std_msgs::Empty>("/ardrone/land",1);
    int loop=0;
    while(ros::ok() && droneState()!=8 && loop<100) {
        droneTakeOffPub.publish(emptyMsg);
        loop++;
        ros::spinOnce();
        _rate->sleep();
    }
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
    /*Subscribers*/
    ros::Subscriber droneNavDataSub = nh.subscribe("/ardrone/navdata",1,&MainWindow::navDataCallback,this);
    ROS_DEBUG("Subbed to navdata");
    ros::Subscriber testSub = nh.subscribe("/test",1,&MainWindow::testCallback,this);
    ROS_DEBUG("Subbed to test sub");
    _subs.append(droneNavDataSub);
    _subs.append(testSub);
    image_transport::ImageTransport it(nh);
    _itSub = new image_transport::Subscriber(it.subscribe("/ardrone/image_raw",1,&MainWindow::cameraCallback,this));
    ROS_DEBUG("Subbed to the camera");    
    _itSubRescuer = new image_transport::Subscriber(it.subscribe("/kinect/rgb/image_raw",1,&MainWindow::cameraRescuerCallback,this));
    /*Publishers*/
    _cmdVelPub = new ros::Publisher(getNodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel",1));
    _pubs.append(*_cmdVelPub);
    setIsConnected(true);
    log("Drone connected.");
}

void MainWindow::updateRotValues(QVector<float> rotV)
{
    QString displayed = "["+QString::number(rotV.at(0))+","+QString::number(rotV.at(1))+","+QString::number(rotV.at(2))+"]";
    QLineEdit* rotLe = _ui.rotXYZLineEdit;
    rotLe->clear();
    rotLe->setText(displayed);
}

void MainWindow::setDroneState(int arg)
{
    if (m_droneState != arg) {
        m_droneState = arg;
        emit droneStateChanged(arg);
    }
    QPalette myPal;
    if(arg==0) {
        myPal.setColor(QPalette::Base,Qt::red);
    }
    _ui.stateSpinBox->setPalette(myPal);
}

void MainWindow::updateVValues(const QVector<float> vel)
{
    QString displayed = format3Data(vel);
    _ui.vXYZLineEdit->clear();
    _ui.vXYZLineEdit->setText(displayed);
}

void MainWindow::swapCamera()
{
    ROS_DEBUG("Swap Camera callback");
    ros::ServiceClient swapCamService = getNodeHandle().serviceClient<std_srvs::Empty>("/ardrone/togglecam");
    std_srvs::Empty emptyCall;
    swapCamService.call(emptyCall);
}

void MainWindow::flatTrim()
{
    ros::ServiceClient flatTrimService = getNodeHandle().serviceClient<std_srvs::Empty>("/ardrone/flattrim");
    std_srvs::Empty emptyCall;
    flatTrimService.call(emptyCall);
    log("Flat trim executed");
}

void MainWindow::activateAutoHoverMode()
{
    /*log("Auto-hover mode activated.");*/
    ros::Publisher cmdVel = getNodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel",1);
    geometry_msgs::Twist cmd;
    cmd.linear.x=0;
    cmd.linear.y=0;
    cmd.linear.z=0;
    cmd.angular.z=0;
    cmd.angular.x=0;
    cmd.angular.y=0;
    cmdVel.publish(cmd);
}

void MainWindow::droneUp()
{
    ROS_DEBUG("Drone up");
//    log("up");
    geometry_msgs::Twist cmd;
    cmd.linear.z = _defaultSpeed;
    _cmdVelPub->publish(cmd);
    ros::spinOnce();    
}

void MainWindow::droneDown()
{
    ROS_DEBUG("Drone down");
//    log("down");
    geometry_msgs::Twist cmd;
    cmd.linear.z = -_defaultSpeed;
    _cmdVelPub->publish(cmd);
    ros::spinOnce();    
}

void MainWindow::droneForward()
{
    ROS_DEBUG("Drone forward");
//    log("Drone forward");
    geometry_msgs::Twist cmd;
    cmd.linear.x = _defaultSpeed;
    _cmdVelPub->publish(cmd);
    ros::spinOnce();
}

void MainWindow::droneBackward()
{
    ROS_DEBUG("Drone backward");
//    log("Drone backward");
    geometry_msgs::Twist cmd;
    cmd.linear.x = -_defaultSpeed;
    _cmdVelPub->publish(cmd);
    ros::spinOnce();
}

void MainWindow::droneLeft()
{
    ROS_DEBUG("Drone left");
//    log("Drone left");
    geometry_msgs::Twist cmd;
    cmd.angular.z = _defaultSpeed;
    _cmdVelPub->publish(cmd);
    ros::spinOnce();
}

void MainWindow::droneRight()
{
    ROS_DEBUG("Drone right");
//    log("Drone right");
    geometry_msgs::Twist cmd;
    cmd.angular.z = -_defaultSpeed;
    _cmdVelPub->publish(cmd);
    ros::spinOnce();
}

void MainWindow::setIsConnected(bool arg)
{
    if (m_isConnected != arg) {
        m_isConnected = arg;
        emit isConnectedChanged(arg);
    }
}

void MainWindow::droneReset()
{
    ROS_DEBUG("Reset the drone");
    log("Drone reset");
    std_msgs::Empty emptyMsg;
    ros::Publisher droneResetPub = getNodeHandle().advertise<std_msgs::Empty>("/ardrone/reset",1);
    int loop=0;
    while(ros::ok() && loop<100 && droneState()!=2) {
        droneResetPub.publish(emptyMsg);
        loop++;
        ros::spinOnce();
        _rate->sleep();
    }
}

/**
 * @brief MainWindow::navDataCallback
 * @param navData
 * Note: this is a ROS callback: not possible to access Qt objects.
 */
void MainWindow::navDataCallback(const ardrone_autonomy::Navdata &navData)
{
    emit batteryUpdated((int)navData.batteryPercent);
    QVector<float> rot;
    rot<<navData.rotX<<navData.rotY <<navData.rotZ;
    emit rotDataUpdated(rot);
    u_int32_t state = navData.state;
    setDroneState(state);
    QVector<float> vel;
    vel<<navData.vx<<navData.vy<<navData.vz;
    emit velDataUpdated(vel);
    emit altUpdated((int)navData.altd);
    emit tagCountUpdated((int)navData.tags_count);
}
} // namespace
PLUGINLIB_EXPORT_CLASS(rescuer_project::MainWindow,rqt_gui_cpp::Plugin)
