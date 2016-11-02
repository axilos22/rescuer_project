#ifndef MAINWINDOW_H
#define MAINWINDOW_H
/*RQT*/
#include <rqt_gui_cpp/plugin.h>
#include <ui_image_view.h>
#include <image_transport/image_transport.h>
#include <ros/macros.h>
/*QT*/
#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
/*own*/
#include "rescuer_project/rescuer_widget.h"

namespace rescuer_project {

class Rescuer_gui
        : public rqt_gui_cpp::Plugin {
//    uncommenting Q_OBJECT generate errors
//    Q_OBJECT
public:
    Rescuer_gui();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
private:
    QWidget* widget_;
    QPushButton *_button;
    QHBoxLayout *_hLayout;
    RescuerWidget *_rWidget;
};
}//namespace
#endif // MAINWINDOW_H
