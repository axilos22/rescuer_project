#ifndef MAINWINDOW_H
#define MAINWINDOW_H
/*ROS*/
#include <ros/ros.h>
#include <ros/macros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
/*RQT*/
#include <rqt_gui_cpp/plugin.h>
//#include <ui_image_view.h>
/*QT*/
#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QString>
/*own*/
#include <ui_main_window.h>
#include "rescuer_project/rescuer_widget.h"

namespace rescuer_project {

class MainWindow : public rqt_gui_cpp::Plugin {
    //again Q_OBJECT add errors after few makes
//    Q_OBJECT
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
protected:
    QWidget* _centralWidget;
    Ui::MainWindowWidget _ui;
    ros::NodeHandle* _nh;
    QTextEdit *_console;
};
}//namespace
#endif // MAINWINDOW_H
