/*
 * GaitSwitcherPlugin.hpp
 *
 *  Created on: Jan 21, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

// qt
#include <rqt_gait_switcher/ui_gait_switcher_plugin.h>
#include <QWidget>

// ros
#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>
#include <motion_generation_msgs/GetAvailableGaits.h>
#include <motion_generation_msgs/SwitchGait.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

// stl
#include <memory>

class GaitSwitcherPlugin : public rqt_gui_cpp::Plugin {
  Q_OBJECT
public:
    GaitSwitcherPlugin();
    void initPlugin(qt_gui_cpp::PluginContext& context) override;
    void shutdownPlugin() override;
    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;
    void setServiceClients();

private:
    Ui::GaitSwitcherUI ui_;
    std::unique_ptr<QWidget> widget_;
    ros::ServiceClient getAvailableGaitsClient_;
    ros::ServiceClient switchGaitClient_;
    ros::ServiceClient switchToDefaultConfigClient_;

protected slots:
    virtual void buttonResetPressed();
    virtual void buttonSetPressed();
    virtual void buttonDefaultConfigPressed();
};
