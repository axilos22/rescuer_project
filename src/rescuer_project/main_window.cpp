#include "rescuer_project/main_window.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace rescuer_project {
MyWindow::MyWindow():rqt_gui_cpp::Plugin(),widget_(0) {
	setObjectName("Ma fenetre");
}
void MyWindow::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  button_ = new QPushButton("Press me plz !!!",widget_);
  // extend the widget with all attributes and children from UI file
  //~ ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);
}

void MyWindow::shutdownPlugin()
{
  // TODO unregister all publishers here
}

void MyWindow::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void MyWindow::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}
} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_example_cpp, MyPlugin, rescuer_project::MyWindow, rqt_gui_cpp::Plugin)
