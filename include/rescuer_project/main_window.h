#ifndef MAINWINDOW_H
#define MAINWINDOW_H
/*CPP*/
#include <float.h>
/*RQT*/
#include <rqt_gui_cpp/plugin.h>
#include <pluginlib/class_list_macros.h>
/*ROS*/
#include <ros/ros.h>
#include <ros/macros.h>
#include <cv_bridge/cv_bridge.h>
#include <float.h>
#include <image_transport/image_transport.h>
/*ROS-msg*/
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
/*QT*/
#include <QTextEdit>
#include <QString>
#include <QWidget>
#include <QVector>
#include <QStringList>
#include <QSpinBox>
#include <QPalette>
/*Open-CV*/
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
/*AR Drone*/
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/CamSelect.h>
#include <ardrone_autonomy/CamSelectRequest.h>
/*own*/
#include <ui_main_window.h>
#include <rescuer_project/lineeditteleop.h>

namespace rescuer_project {

class MainWindow : public rqt_gui_cpp::Plugin {
    Q_OBJECT
    Q_PROPERTY(int droneState READ droneState WRITE setDroneState NOTIFY droneStateChanged)
    Q_PROPERTY(bool isConnected READ isConnected WRITE setIsConnected NOTIFY isConnectedChanged)
public:
    MainWindow();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
    void log(QString msg);
    int droneState() const;
    QString format3Data(const QVector<float> tab);
    bool isConnected() const;

public slots:
    void droneTakeOff();
    void droneLand();
    void connectWithDrone();
    void updateRotValues(QVector<float> rotV);
    void setDroneState(int arg);
    void updateVValues(const QVector<float> vel);
    void swapCamera();
    void flatTrim();
    void activateAutoHoverMode();
    void droneUp();
    void droneDown();
    void droneForward();
    void droneBackward();
    void droneLeft();
    void droneRight();
    void setIsConnected(bool arg);
    void droneReset();

signals:
    void batteryUpdated(int percent);
    void rotDataUpdated(QVector<float>);
    void velDataUpdated(QVector<float>);
    void camImgUpdated(QPixmap);
    void camRescuerImgUpdated(QPixmap);
    void droneStateChanged(int arg);
    void altUpdated(int alt);
    void tagCountUpdated(int tc);
    void isConnectedChanged(bool arg);

protected:
    QWidget* _centralWidget;
    Ui::MainWindowWidget _ui;
    ros::NodeHandle* _nh;
    ros::Rate* _rate;
    QTextEdit *_console;    
    /*ROS*/
    QVector<ros::Subscriber> _subs;
    ros::Subscriber _droneNavDataSub, _testSub;
    QVector<ros::Publisher> _pubs;    
    ros::Publisher* _cmdVelPub;
    //cam
    cv::Mat _conversionMat;
    image_transport::Subscriber* _itSub;
    image_transport::Subscriber* _itSubRescuer;
    QPixmap _cameraPixmap;
    LineEditTeleop *_teleop;

    int sendEmptyCommand(QString commandTopic);
    void navDataCallback(const ardrone_autonomy::Navdata& navData);
    void testCallback(const std_msgs::String::ConstPtr& msg);
    void cameraCallback(const sensor_msgs::ImageConstPtr &msg);
    void cameraRescuerCallback(const sensor_msgs::ImageConstPtr &msg);
private:
    float _defaultSpeed;
    bool m_isConnected;
    int m_droneState;
};
}//namespace
#endif // MAINWINDOW_H
