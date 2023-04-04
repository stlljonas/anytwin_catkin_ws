#pragma once


// c++
#include <mutex>

// qt
#include <QSignalMapper>

// ros
#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>

// anymal lowlevel controller msgs
#include <anymal_msgs/AnymalLowLevelControllerGoToState.h>
#include <anymal_msgs/AnymalLowLevelControllerState.h>

// rqt anymal lowlevel controller
#include <ui_AnymalLowLevelControllerPlugin.h>


namespace rqt_anymal_lowlevel_controller {


class AnymalLowLevelControllerPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

protected:
  Ui::AnymalLowLevelControllerPluginWidget ui_;
  QWidget* widget_ = nullptr;
  QSignalMapper* signalMapper_ = nullptr;

  std::mutex activeStateMutex_;
  anymal_msgs::AnymalLowLevelControllerState activeState_;

  ros::Subscriber activeStateSubscriber_;
  ros::ServiceClient goToStateClient_;

public:
  AnymalLowLevelControllerPlugin();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();

  virtual void saveSettings(
      qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(
      const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

protected:
  void activeStateCb(const anymal_msgs::AnymalLowLevelControllerStateConstPtr& state);

signals:
  void activeStateChanged();

protected slots:
  void updateActiveState();
  void goToStateCb(int stateId);
};


} // rqt_anymal_lowlevel_controller
