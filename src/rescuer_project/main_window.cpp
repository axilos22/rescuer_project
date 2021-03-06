#include "rescuer_project/main_window.h"
#include <QChar>
#include <QDebug>
#include <QStringList>
namespace rescuer_project {

MainWindow::MainWindow():rqt_gui_cpp::Plugin(),
    _centralWidget(0),
    m_defaultSpeed(.3),
    m_isConnected(false),
    m_autopilotActivated(false),
    m_droneState(0),
    rescuer_linearVel(0.2),
    rescuer_angularVel(0.7)
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
    connect(this,SIGNAL(droneStateChanged(int)),_ui.stateSpinBox,SLOT(setValue(int)));
    //rescuer
    connect(this,SIGNAL(camRescuerImgUpdated(QPixmap)),_ui.rescuerCamLabel,SLOT(setPixmap(QPixmap)));
    connect(this,SIGNAL(rescuerPoseUpdated(QVector<float>)),this,SLOT(updateRescuerPoseValues(QVector<float>)));
    connect(_ui.goalButton,SIGNAL(pressed()),this,SLOT(setRescuerGoal()));
    // arm
    connect(_ui.initButton,SIGNAL(pressed()),this,SLOT(initPose()));
    connect(_ui.pickButton,SIGNAL(pressed()),this,SLOT(pickPose()));
    connect(_ui.placeButton,SIGNAL(pressed()),this,SLOT(placePose()));
    connect(_ui.raiseButton,SIGNAL(pressed()),this,SLOT(raisePose()));
    connect(_ui.retrieveButton,SIGNAL(pressed()),this,SLOT(retrievePose()));
    // gripper
    connect(_ui.openButton,SIGNAL(pressed()),this,SLOT(openGripper()));
    connect(_ui.closeButton,SIGNAL(pressed()),this,SLOT(closeGripper()));
    //views
    _ui.topViewButton->setVisible(false);
    _ui.sideViewButton->setVisible(false);
    connect(_ui.topViewButton,SIGNAL(pressed()),this,SLOT(viewFromTop()));
    connect(_ui.sideViewButton,SIGNAL(pressed()),this,SLOT(viewFromSide()));

    //Adding teleoperator
    _teleop = new LineEditTeleop;
    _teleop->setEnabled(false);
    _ui.formLayout->addRow(tr("&Teleop:"),_teleop);    
    //Drone teleoperation    
    connect(_teleop,SIGNAL(pageUpPressed()),this,SLOT(droneUp()));
    connect(_teleop,SIGNAL(pageDownPressed()),this,SLOT(droneDown()));
    connect(_teleop,SIGNAL(upPressed()),this,SLOT(droneForward()));
    connect(_teleop,SIGNAL(downPressed()),this,SLOT(droneBackward()));
    connect(_teleop,SIGNAL(leftPressed()),this,SLOT(droneLeft()));
    connect(_teleop,SIGNAL(rightPressed()),this,SLOT(droneRight()));
    connect(_teleop,SIGNAL(homePressed()),this,SLOT(droneTakeOff()));
    connect(_teleop,SIGNAL(endPressed()),this,SLOT(droneLand()));
    connect(_teleop,SIGNAL(spacePressed()),this,SLOT(activateAutoHoverMode()));   

    connect(_teleop,SIGNAL(wkeyPressed()),this,SLOT(rescuerForward()));
    connect(_teleop,SIGNAL(xkeyPressed()),this,SLOT(rescuerBackward()));
    connect(_teleop,SIGNAL(dkeyPressed()),this,SLOT(rescuerTurnLeft()));
    connect(_teleop,SIGNAL(akeyPressed()),this,SLOT(rescuerTurnRight()));
    //connect(_teleop,SIGNAL(keyReleased()),this,SLOT(activateAutoHoverMode()));
    connect(_teleop,SIGNAL(skeyPressed()),this,SLOT(rescuerStop()));

    //on connection
    connect(this,SIGNAL(isConnectedChanged(bool)),_teleop,SLOT(setEnabled(bool)));
    connect(this,SIGNAL(isConnectedChanged(bool)),_ui.goToLineEdit,SLOT(setEnabled(bool)));
    connect(this,SIGNAL(isConnectedChanged(bool)),_ui.goToButton,SLOT(setEnabled(bool)));
    connect(this,SIGNAL(isConnectedChanged(bool)),_ui.rawCmdButton,SLOT(setEnabled(bool)));
    connect(this,SIGNAL(isConnectedChanged(bool)),_ui.autopilotCheckbox,SLOT(setEnabled(bool)));
    connect(this,SIGNAL(isConnectedChanged(bool)),_ui.droneTakeOffButton,SLOT(setEnabled(bool)));
    //autopilot connection
    connect(_ui.goToButton,SIGNAL(pressed()),this,SLOT(autopilotGoTo()));
    connect(_ui.rawCmdButton,SIGNAL(pressed()),this,SLOT(autopilotRawCmd()));
    connect(_ui.autopilotCheckbox,SIGNAL(toggled(bool)),this,SLOT(autopilotActivated(bool)));
    connect(this,SIGNAL(autopilotUpdated(bool)),this,SLOT(changeTakeOffButton()));
    //state behavior connection
    connect(_ui.stateComboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(stateChanged(int)));    
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
    //Unregister all ROS publishers here
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

QString MainWindow::format3Data(const QVector<float> tab,bool shortest)
{
    if(shortest) {
        QString nb1 = QString::number(tab.at(0),'f',2);
        QString nb2 = QString::number(tab.at(1),'f',2);
        QString nb3 = QString::number(tab.at(2),'f',2);
        QString out = QString("[%1,%2,%3]").arg(nb1,nb2,nb3);
        return out;
    } else {
        return "["+QString::number(tab.at(0))+","+QString::number(tab.at(1))+","+QString::number(tab.at(2))+"]";
    }
}

QVector<float> MainWindow::formatStringData(const QString inData, const QChar separator)
{
    QVector<float> outData(3,0);
    if(!inData.contains(separator)) {
        log("Wrong formatting provided (no separator found)");
        return outData;
    }
    outData.clear();
    QStringList nbList= inData.split(QRegExp(separator));
    for(int i=0;i<nbList.size();i++) {        
        float nb = nbList.at(i).toFloat();
        outData.push_back(nb);
    }    
    return outData;
}

bool MainWindow::isConnected() const
{
    return m_isConnected;
}

void MainWindow::sendAutopilotCommand(const QString cmd)
{
    std_msgs::String commandOrder;
    QString formattedOrder = "c "+cmd;
    commandOrder.data = formattedOrder.toStdString();
    log(formattedOrder);
    _autoPilotPub->publish(commandOrder);
    ros::spinOnce();
}

void MainWindow::droneTakeOff()
{
    ROS_DEBUG("Drone has to take off now.");
    if(m_autopilotActivated) {
        log("Autopilot taking off");
        QString cmd="autoInit 500 800 4000 0.5";
        sendAutopilotCommand(cmd);
        cmd="setReference $POSE$";
        sendAutopilotCommand(cmd);
//        cmd="setInitialReachDist 0.2";
//        sendAutopilotCommand(cmd);
//        cmd="setStayWithinDist 0.3";
//        cmd="setStayTime 3";
        cmd="lockScaleFP";
        sendAutopilotCommand(cmd);
        cmd="goto 0 0 0 0";
        sendAutopilotCommand(cmd);
        return;
    }
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
    /*SUBSCRIBERS*/
    ros::Subscriber droneNavDataSub = nh.subscribe("/ardrone/navdata",1,&MainWindow::navDataCallback,this);
    ROS_DEBUG("Subbed to navdata");
    ros::Subscriber testSub = nh.subscribe("/test",1,&MainWindow::testCallback,this);
    ROS_DEBUG("Subbed to test sub");
    ros::Subscriber filterStateSub=getNodeHandle().subscribe("/ardrone/predictedPose",1,&MainWindow::autopilotFilterCallback,this);
    ros::Subscriber goalStatusSub = nh.subscribe("/move_base/result",1,&MainWindow::goalStatusCallback,this);
    ROS_DEBUG("Subbed to goal status");

    /*sub management*/
    _subs.append(droneNavDataSub);
    _subs.append(testSub);
    _subs.append(filterStateSub);
    _subs.append(goalStatusSub);
    image_transport::ImageTransport it(nh);
    _itSub = new image_transport::Subscriber(it.subscribe("/ardrone/image_raw",1,&MainWindow::cameraCallback,this));
    ROS_DEBUG("Subbed to the camera");    
    /*Rescuer*/
    _itSubRescuer = new image_transport::Subscriber(it.subscribe("/camera/rgb/image_raw",1,&MainWindow::cameraRescuerCallback,this));
    ros::Subscriber rescuerPoseSub = nh.subscribe("/mobile_base/abs_pos",1,&MainWindow::rescuerPoseCallback,this);
    _subs.append(rescuerPoseSub);


    /*PUBLISHERS*/
    #if SIMULATOR==1
    _cmdVelPub = new ros::Publisher(getNodeHandle().advertise<geometry_msgs::Twist>("/quadrotor/cmd_vel",1));
    #else
    _cmdVelPub = new ros::Publisher(getNodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel",1));
    #endif
    _baseGoalPub = new ros::Publisher(getNodeHandle().advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1));   
    _baseCmdVelPub = new ros::Publisher(getNodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1));
    _autoPilotPub = new ros::Publisher(getNodeHandle().advertise<std_msgs::String>("/tum_ardrone/com",1));
    _changeModePub = new ros::Publisher(getNodeHandle().advertise<std_msgs::String>("/behaviour_mode",1));
    _armJoint1_Pub = new ros::Publisher(getNodeHandle().advertise<std_msgs::Float64>("/gwam/j1_position_controller/command",1));
    _armJoint2_Pub = new ros::Publisher(getNodeHandle().advertise<std_msgs::Float64>("/gwam/j2_position_controller/command",1));
    _armJoint4_Pub = new ros::Publisher(getNodeHandle().advertise<std_msgs::Float64>("/gwam/j4_position_controller/command",1));
    _armJoint6_Pub = new ros::Publisher(getNodeHandle().advertise<std_msgs::Float64>("/gwam/j6_position_controller/command",1));
    _gripperFinger1_Pub = new ros::Publisher(getNodeHandle().advertise<std_msgs::Float64>("/gwam/bh_j12_position_controller/command",1));
    _gripperFinger2_Pub = new ros::Publisher(getNodeHandle().advertise<std_msgs::Float64>("/gwam/bh_j22_position_controller/command",1));
    _gripperFinger3_Pub = new ros::Publisher(getNodeHandle().advertise<std_msgs::Float64>("/gwam/bh_j32_position_controller/command",1));
    /*Pub management*/
    _pubs.append(*_cmdVelPub);
    _pubs.append(*_baseGoalPub);
    _pubs.append(*_autoPilotPub);  
    _pubs.append(*_changeModePub);
    _pubs.append(*_armJoint1_Pub);
    _pubs.append(*_armJoint2_Pub);
    _pubs.append(*_armJoint4_Pub);
    _pubs.append(*_armJoint6_Pub);
    _pubs.append(*_gripperFinger1_Pub);
    _pubs.append(*_gripperFinger2_Pub);
    _pubs.append(*_gripperFinger3_Pub);
    setIsConnected(true);
    log("Drone connected.");
}

void MainWindow::updateRescuerPoseValues(QVector<float> pose)
{
    QString displayed = QString::number(pose.at(0),'f',2)+","+QString::number(pose.at(1),'f',2)+","+QString::number(pose.at(2),'f',2);
    QLineEdit* rotLe = _ui.poseLineEdit;
    rotLe->clear();
    rotLe->setText(displayed);
}

void MainWindow::updateRotValues(QVector<float> rotV)
{
    float x = rotV.at(0);
    float y = rotV.at(1);
    float z = rotV.at(2);
    QString displayed = "["+QString::number(x,'f',2)+","+QString::number(y,'f',2)+","+QString::number(z,'f',2)+"]";
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

void MainWindow::swapCamera()
{
    ROS_DEBUG("Swap Camera callback");
    #if SIMULATOR==1
        ros::ServiceClient swapCamService = getNodeHandle().serviceClient<std_srvs::Empty>("/quadrotor/ardrone/togglecam");
    #else
        ros::ServiceClient swapCamService = getNodeHandle().serviceClient<std_srvs::Empty>("/ardrone/togglecam");
    #endif
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
    log("Auto-hover mode activated.");
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
    log("Drone up");
    geometry_msgs::Twist cmd;
    cmd.linear.z = m_defaultSpeed;
    _cmdVelPub->publish(cmd);
    ros::spinOnce();    
}

void MainWindow::droneDown()
{
    log("Drone down");
    geometry_msgs::Twist cmd;
    cmd.linear.z = -m_defaultSpeed;
    _cmdVelPub->publish(cmd);
    ros::spinOnce();    
}

void MainWindow::droneForward()
{
    //log("Drone forward");
    geometry_msgs::Twist cmd;
    cmd.linear.x = m_defaultSpeed;
    _cmdVelPub->publish(cmd);
    ros::spinOnce();
}

void MainWindow::droneBackward()
{
    //log("Drone backward");
    geometry_msgs::Twist cmd;
    cmd.linear.x = -m_defaultSpeed;
    _cmdVelPub->publish(cmd);
    ros::spinOnce();
}

void MainWindow::droneLeft()
{
    //log("Drone left");
    geometry_msgs::Twist cmd;
//    cmd.angular.z = m_defaultSpeed;
    cmd.linear.y = m_defaultSpeed;
    _cmdVelPub->publish(cmd);
    ros::spinOnce();
}

void MainWindow::droneRight()
{
    //log("Drone right");
    geometry_msgs::Twist cmd;
//    cmd.angular.z = -m_defaultSpeed;
    cmd.linear.y = -m_defaultSpeed;
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
}

/**
 * @brief MainWindow::rescuerPoseCallback
 * @param msg
 * Note: this is a ROS callback: not possible to access Qt objects.
 */
void MainWindow::rescuerPoseCallback(const geometry_msgs::Pose2D &msg)
{
    QVector<float> pose;
    pose<<msg.x<<msg.y <<msg.theta;
    emit rescuerPoseUpdated(pose);
}

void MainWindow::autopilotFilterCallback(const tum_ardrone::filter_state &msg)
{
    QVector<float> state;
    state.append(msg.x);
    state.append(msg.y);
    state.append(msg.z);
    const QString display = format3Data(state,true);
    _ui.PoselineEdit->setText(display);
}

void MainWindow::setRescuerGoal()
{
    QVector<float> goal = formatStringData(_ui.goalLineEdit->text());
    if(goal.size()!=3) {
        log("Incorrect data provided. Expected 3 data, got "+QString::number(goal.size()));
        return;
    }  
    geometry_msgs::PoseStamped cmd;    
    cmd.header.frame_id = "/mobile_map";
    cmd.header.stamp = ros::Time::now();
    cmd.pose.position.x = goal.at(0);
    cmd.pose.position.y = goal.at(1);
    cmd.pose.position.z = 0.0;
    tf::Quaternion quaternion = tf::createQuaternionFromYaw(goal.at(2));
    cmd.pose.orientation.x =  quaternion.getX();
    cmd.pose.orientation.y =  quaternion.getY();
    cmd.pose.orientation.z =  quaternion.getZ();
    cmd.pose.orientation.w =  quaternion.getW();
    _baseGoalPub->publish(cmd);
    _ui.goalStatusLineEdit->setText("New plan");
    ros::spinOnce();
}

void MainWindow::autopilotGoTo()
{
    autopilotClear();
    QVector<float> goal = formatStringData(_ui.goToLineEdit->text());
    if(goal.size()!=4) {
        log("Incorrect data provided. Expected 4 data, got "+QString::number(goal.size()));
        return;
    }    
    QString formattedOrder = "goto "+QString::number(goal.at(0))+" "+QString::number(goal.at(1))+" "+QString::number(goal.at(2))+" "+QString::number(goal.at(3));
    sendAutopilotCommand(formattedOrder);
}

void MainWindow::autopilotGoToRel()
{
    QVector<float> goal = formatStringData(_ui.goToLineEdit->text());
    if(goal.size()!=4) {
        log("Incorrect data provided. Expected 4 data, got "+QString::number(goal.size()));
        return;
    }
    QString formattedOrder = "moveByRel "+QString::number(goal.at(0))+" "+QString::number(goal.at(1))+" "+QString::number(goal.at(2))+" "+QString::number(goal.at(3));
    sendAutopilotCommand(formattedOrder);
}

void MainWindow::rescuerForward()
{
    //log("Rescuer forward");
    geometry_msgs::Twist cmd;
    cmd.linear.x = rescuer_linearVel;
    _baseCmdVelPub->publish(cmd);
    ros::spinOnce();
}
void MainWindow::rescuerBackward()
{
    //log("Rescuer backward");
    geometry_msgs::Twist cmd;
    cmd.linear.x = -rescuer_linearVel;
    _baseCmdVelPub->publish(cmd);
    ros::spinOnce();
}
void MainWindow::rescuerTurnRight()
{
    //log("Rescuer turn right");
    geometry_msgs::Twist cmd;
    cmd.angular.z = rescuer_angularVel;
    _baseCmdVelPub->publish(cmd);
    ros::spinOnce();
}
void MainWindow::rescuerTurnLeft()
{
    //log("Rescuer turn left");
    geometry_msgs::Twist cmd;
    cmd.angular.z = -rescuer_angularVel;
    _baseCmdVelPub->publish(cmd);
    ros::spinOnce();
}
void MainWindow::rescuerStop()
{
    //log("Rescuer stop");
    geometry_msgs::Twist cmd;
    _baseCmdVelPub->publish(cmd);
    ros::spinOnce();
}

void MainWindow::stateChanged(int newState)
{
    std_msgs::String changeModeCommand;
    switch (newState) {
    case 0:
        log("default");
        break;
    case 1:
        log("coupled");        
        //QString formattedOrder = "c "+cmd;
        changeModeCommand.data = "coupled";
        _changeModePub->publish(changeModeCommand);
        ros::spinOnce();
        break;
    case 2:
        log("decoupled");
        changeModeCommand.data = "decoupled";
        _changeModePub->publish(changeModeCommand);
        ros::spinOnce();
        break;
    case 3:
        log("assistance");
        _ui.topViewButton->setVisible(true);
        _ui.sideViewButton->setVisible(true);
        break;
    default:
        log("? state");
        break;
    }
}

void MainWindow::changeTakeOffButton()
{
    if(m_autopilotActivated){
        _ui.droneTakeOffButton->setText("Take off-autopilot");
    } else {
        _ui.droneTakeOffButton->setText("Take off");
    }
}

void MainWindow::autopilotAutoInit()
{
    QString formattedOrder = "autoInit 500 800 4000 0.5";
    sendAutopilotCommand(formattedOrder);
}

void MainWindow::autopilotTakeover()
{
    QString formattedOrder = "autoTakeover 500 800";
    sendAutopilotCommand(formattedOrder);
}

void MainWindow::autopilotSetReference()
{
//    QString formattedOrder = "setReference 0.0 0.0 0.0 0.0";
    QString formattedOrder = "setReference $POSE$";
    sendAutopilotCommand(formattedOrder);
}

void MainWindow::autopilotClear()
{
    QString formattedOrder = "clearCommands";
    sendAutopilotCommand(formattedOrder);
}

void MainWindow::autopilotRawCmd()
{
    QString formattedOrder = _ui.rawCmdLineEdit->text();
    sendAutopilotCommand(formattedOrder);
}

void MainWindow::autopilotActivated(bool activation)
{
    m_autopilotActivated = activation;
    QString cmd("start");
    if(!activation) {
        cmd="stop";
    }
    sendAutopilotCommand(cmd);
    emit autopilotUpdated(activation);
}

void MainWindow::goalStatusCallback(const move_base_msgs::MoveBaseActionResult &msg)
{
    QString result = QString::fromStdString(msg.status.text);
    _ui.goalStatusLineEdit->setText(result);

}

void MainWindow::setArmPose(QVector<float> pos){
    std_msgs::Float64 joint_1, joint_2, joint_4, joint_6;
    joint_1.data = pos.at(0);
    joint_2.data = pos.at(1);
    joint_4.data = pos.at(2);
    joint_6.data = pos.at(3);
    _armJoint1_Pub->publish(joint_1);
    _armJoint2_Pub->publish(joint_2);
    _armJoint4_Pub->publish(joint_4);
    _armJoint6_Pub->publish(joint_6);
    ros::spinOnce();
}

void MainWindow::initPose()
{
    QVector<float> pose;
    pose.append(0.0);   //arm joint 1 position
    pose.append(0.0);   //arm joint 2 position
    pose.append(0.0);   //arm joint 4 position
    pose.append(0.0);  //arm joint 6 position
    setArmPose(pose);
}

void MainWindow::pickPose()
{
    QVector<float> pose;
    pose.append(0.0);   //arm joint 1 position
    pose.append(0.0);   //arm joint 2 position
    pose.append(2.0);   //arm joint 4 position
    pose.append(-1.0);  //arm joint 6 position    
    setArmPose(pose);
    delay(20);
    pose[1] = 0.2;
    setArmPose(pose);
}

void MainWindow::raisePose()
{
    QVector<float> pose;
    pose.append(0.0);   //arm joint 1 position
    pose.append(0.0);   //arm joint 2 position
    pose.append(0.5);   //arm joint 4 position
    pose.append(-1.0);  //arm joint 6 position
    setArmPose(pose);
    delay(20);
    pose[1] = 0.3;
    pose[4] = 0.0;
    setArmPose(pose);
}

void MainWindow::placePose()
{
    QVector<float> pose;
    pose.append(0.0);   //arm joint 1 position
    pose.append(0.5);   //arm joint 2 position
    pose.append(0.5);   //arm joint 4 position
    pose.append(-0.2);  //arm joint 6 position
    setArmPose(pose);
    delay(20);
    pose[0] = -0.7;
    setArmPose(pose);
}

void MainWindow::retrievePose()
{
    QVector<float> pose;
    pose.append(0.0);   //arm joint 1 position
    pose.append(0.5);   //arm joint 2 position
    pose.append(0.5);   //arm joint 4 position
    pose.append(-0.2);  //arm joint 6 position
    setArmPose(pose);
    delay(20);
    pose[1] = 0.0;
    setArmPose(pose);
}

void MainWindow::closeGripper()
{
    std_msgs::Float64 finger_1, finger_2, finger_3;
    finger_1.data = 1.0;
    finger_2.data = 1.0;
    finger_3.data = 1.1;
    _gripperFinger1_Pub->publish(finger_1);
    _gripperFinger2_Pub->publish(finger_2);
    _gripperFinger3_Pub->publish(finger_3);
    ros::spinOnce();
}

void MainWindow::openGripper()
{
    std_msgs::Float64 finger_1, finger_2, finger_3;
    finger_1.data = 0.0;
    finger_2.data = 0.0;
    finger_3.data = 0.0;
    _gripperFinger1_Pub->publish(finger_1);
    _gripperFinger2_Pub->publish(finger_2);
    _gripperFinger3_Pub->publish(finger_3);
    ros::spinOnce();
}

void MainWindow::viewFromTop()
{
    autopilotClear();
    QString formattedOrder = "goto -5.0 4.7 2.5 0.0";
    sendAutopilotCommand(formattedOrder);
}

void MainWindow::viewFromSide()
{
    autopilotClear();
    QString formattedOrder = "goto -5.5 4.0 1.0 0.0";
    sendAutopilotCommand(formattedOrder);
}

void MainWindow::delay(int seconds){
    QTime dieTime= QTime::currentTime().addSecs(seconds);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);

}

} // namespace
PLUGINLIB_EXPORT_CLASS(rescuer_project::MainWindow,rqt_gui_cpp::Plugin)
