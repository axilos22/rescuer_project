#ifndef MAINWINDOW_H
#define MAINWINDOW_H
/*RQT*/
#include <rqt_gui_cpp/plugin.h>
/*ROS*/
#include <ros/ros.h>
#include <ros/macros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <float.h>
/*ROS-msg*/
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
/*QT*/
#include <QTextEdit>
#include <QString>
#include <QWidget>
#include <QVector>
/*Open-CV*/
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
/*AR Drone*/
#include <ardrone_autonomy/Navdata.h>
/*own*/
#include <ui_main_window.h>
#include <rescuer_project/rescuer_widget.h>

namespace rescuer_project {

class MainWindow : public rqt_gui_cpp::Plugin {
    Q_OBJECT
public:
    MainWindow();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
    void log(QString msg);
public slots:
    void droneTakeOff();
    void droneLand();
    void connectWithDrone();
    void updateRotValues(QVector<float> rotV);
signals:
    void batteryUpdated(int percent);
    void rotDataUpdated(QVector<float>);
    void camImgUpdated(QPixmap);
protected:
    QWidget* _centralWidget;
    Ui::MainWindowWidget _ui;
    ros::NodeHandle* _nh;
    ros::Rate* _rate;
    QTextEdit *_console;
    QVector<ros::Subscriber> _subs;
    ros::Subscriber _droneNavDataSub, _testSub;
    QVector<ros::Publisher> _pubs;
    cv::Mat _conversionMat;
    //cam
    image_transport::Subscriber* _itSub;

    int sendEmptyCommand(QString commandTopic);
    void navDataCallback(const ardrone_autonomy::Navdata& navData);
    void testCallback(const std_msgs::String::ConstPtr& msg);
//    void cameraCallback(const sensor_msgs::ImageConstPtr& msg);
};
}//namespace
#endif // MAINWINDOW_H
