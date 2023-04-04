/*
 * GaitSwitcherPlugin.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: C. Dario Bellicoso
 */


// pluginlib
#include <pluginlib/class_list_macros.h>

// qt
#include <QStringList>
#include <QGridLayout>

// gait switcher
#include "rqt_gait_switcher/GaitSwitcherPlugin.hpp"

// param io
#include <param_io/get_param.hpp>

//ros
#include <ros/package.h>

// yaml-cpp
#include <yaml-cpp/yaml.h>

GaitSwitcherPlugin::GaitSwitcherPlugin() :
    rqt_gui_cpp::Plugin()
{
  // give QObjects reasonable names
  setObjectName("GaitSwitcherPlugin");
}

void GaitSwitcherPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  // access standalone command line arguments
  QStringList argv = context.argv();

  // create the main widget, set it up and add it to the user interface
  widget_.reset(new QWidget());
  ui_.setupUi(widget_.get());
  context.addWidget(widget_.get());

  connect(ui_.resetButton, SIGNAL(pressed()),this,SLOT(buttonResetPressed()));
  connect(ui_.setButton, SIGNAL(pressed()),this,SLOT(buttonSetPressed()));
  connect(ui_.defaultConfigButton, SIGNAL(pressed()),this,SLOT(buttonDefaultConfigPressed()));

  setServiceClients();
  buttonResetPressed();
}

void GaitSwitcherPlugin::setServiceClients() {
  const std::string serviceNameGetAvailableGaits =
        param_io::param<std::string>(getNodeHandle(),
                                     "/gaitswitcher/service/available_gaits",
                                     "/dynamic_gaits_ros/get_available_gaits");
  getAvailableGaitsClient_ = getNodeHandle().serviceClient<motion_generation_msgs::GetAvailableGaits>(serviceNameGetAvailableGaits);

  const std::string serviceNameSwitchGait =
        param_io::param<std::string>(getNodeHandle(),
                                     "/gaitswitcher/service/switch_gait",
                                     "/dynamic_gaits_ros/switch_gait");
  switchGaitClient_ = getNodeHandle().serviceClient<motion_generation_msgs::SwitchGait>(serviceNameSwitchGait);

  const std::string serviceNameSwitchToDefaultConfig =
        param_io::param<std::string>(getNodeHandle(),
                                     "/gaitswitcher/service/switch_to_default_config",
                                     "/dynamic_gaits_ros/switch_to_default_config");
  switchToDefaultConfigClient_ = getNodeHandle().serviceClient<std_srvs::Empty>(serviceNameSwitchToDefaultConfig);
}

void GaitSwitcherPlugin::buttonSetPressed() {
  motion_generation_msgs::SwitchGait srv;
  srv.request.name = ui_.gaitListBox->currentText().toStdString();
  switchGaitClient_.call(srv);
}

void GaitSwitcherPlugin::buttonResetPressed() {
  motion_generation_msgs::GetAvailableGaits srv;
  getAvailableGaitsClient_.call(srv);
  ui_.gaitListBox->clear();
  for (const auto& name : srv.response.available_gaits_names) {
    ui_.gaitListBox->addItem(QString::fromStdString(name));
  }
}

void GaitSwitcherPlugin::buttonDefaultConfigPressed() {
  std_srvs::Empty srv;
  switchToDefaultConfigClient_.call(srv);
}

void GaitSwitcherPlugin::shutdownPlugin() {
  getAvailableGaitsClient_.shutdown();
  switchGaitClient_.shutdown();
  switchToDefaultConfigClient_.shutdown();
}

void GaitSwitcherPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const {
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void GaitSwitcherPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

PLUGINLIB_EXPORT_CLASS(GaitSwitcherPlugin, rqt_gui_cpp::Plugin)
