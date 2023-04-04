#pragma once

#include <anydrive_monitor/AnydriveWidget.h>
#include <rqt_anydrive_monitor/ui_AnydriveMonitorPlugin.h>
#include <rqt_gui_cpp/plugin.h>
#include "AnydriveInterfaceRos.h"

#include <ros/ros.h>

#include <anydrive_msgs/ReadingsExtended.h>

#include <QObject>
#include <QTimer>
#include <QWidget>

namespace rqt_anydrive_monitor {

class AnydriveMonitorPlugin : public rqt_gui_cpp::Plugin {
  Q_OBJECT

 public:
  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  AnydriveMonitorPlugin();

  /* ======================================================================== */
  /* Initialize/Shutdown                                                      */
  /* ======================================================================== */

  void initPlugin(qt_gui_cpp::PluginContext& context) override;

  void shutdownPlugin() override;

  /* ======================================================================== */
  /* Settings                                                                 */
  /* ======================================================================== */

  void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;

  void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

 private:
  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  Ui::AnydriveMonitorPluginWidget ui_{};
  QWidget* widget_;
  ros::NodeHandle nh_;

  std::string rosPrefix_ = "/anydrive_ros";
  bool singleSubscribers_ = true;

  std::vector<AnydriveInterfaceRos*> anydriveInterfaces_;

  ros::Subscriber readingsSubscriber_;

  int updateFq_ = 30;
  QTimer* updateTimer_ = nullptr;

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  void loadAnydrives();

  void startup();

  void shutdown();

  void setupAnydrive(const std::string& name);

  /* ======================================================================== */
  /* Callbacks                                                                */
  /* ======================================================================== */

  void readingsCallback(const anydrive_msgs::ReadingsExtended& readingsMsg);

 protected slots:

  /* ======================================================================== */
  /* Slots                                                                    */
  /* ======================================================================== */

  void update();
};

}  // namespace rqt_anydrive_monitor
