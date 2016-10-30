#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <rqt_gui_cpp/plugin.h>

#include <QWidget>
#include <QPushButton>

namespace rescuer_project {

class Rescuer_gui
        : public rqt_gui_cpp::Plugin {
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
};
}//namespace
#endif // MAINWINDOW_H
