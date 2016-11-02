#include "rescuer_project/main_window.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace rescuer_project {

Rescuer_gui::Rescuer_gui():
    rqt_gui_cpp::Plugin()
    ,widget_(0) {
    setObjectName("ObjectName");
}

void Rescuer_gui::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget;
    _button = new QPushButton("PRESS ME");
    _hLayout = new QHBoxLayout;
    _rWidget= new RescuerWidget;
    _rWidget->initWidget();

    widget_->setLayout(_hLayout);
    _hLayout->addWidget(_rWidget);
    _hLayout->addWidget(_button);

    // add widget to the user interface
    context.addWidget(widget_);
}

void Rescuer_gui::shutdownPlugin()
{
    // TODO unregister all publishers here
}

void Rescuer_gui::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    // TODO save intrinsic configuration, usually using:
    // instance_settings.setValue(k, v)
}

void Rescuer_gui::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    // TODO restore intrinsic configuration, usually using:
    // v = instance_settings.value(k)
}
} // namespace
PLUGINLIB_EXPORT_CLASS(rescuer_project::Rescuer_gui,rqt_gui_cpp::Plugin)
