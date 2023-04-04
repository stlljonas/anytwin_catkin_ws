#pragma once

// Qt
#include <QWidget>
#include <QEvent>
#include <QObject>

// rqt_gui_cpp
#include <rqt_gui_cpp/plugin.h>

// pluginlib
#include <pluginlib/class_loader.h>

// ros
#include <ros/ros.h>

namespace any_rqt_gui {

class AnyRqtGuiBase: public QWidget {

  Q_OBJECT

public:
  explicit AnyRqtGuiBase(const std::string & name,
                         const std::vector< std::string > & pluginNames = { });
  ~AnyRqtGuiBase() override = default;
  virtual void init();

  std::vector<std::string> getPluginNames() {
    return pluginNames_;
  }

  void setPluginNames( std::vector<std::string> pluginNames) {
    pluginNames_ = std::move(pluginNames);
  }

  bool eventFilter(QObject* obj, QEvent* event) override;

signals:
  void exitApp();

public slots:
  virtual void shutdown();
  virtual void add_widget(QWidget* widget);

protected:
  std::vector< std::string > pluginNames_;
  pluginlib::ClassLoader<rqt_gui_cpp::Plugin> classLoader_;
  std::vector< boost::shared_ptr<rqt_gui_cpp::Plugin> > plugins_;
};

} // any_rqt_gui
