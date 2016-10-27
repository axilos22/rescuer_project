#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <rqt_gui_cpp/plugin.h>
//~ #include <rqt_gui/ui_my_plugin.h>

#include <QWidget>
#include <QMainWindow>
#include <QPushButton>

namespace rescuer_project {

class MyWindow 
  : public rqt_gui_cpp::Plugin
  {
	Q_OBJECT
	public:
		MyWindow();
		virtual void initPlugin(qt_gui_cpp::PluginContext& context);
		virtual void shutdownPlugin();
		virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
		virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
	private:
		QPushButton* m_button;
		QWidget* widget_;
		QPushButton* button_;
	};
}//namespace
#endif // MAINWINDOW_H
