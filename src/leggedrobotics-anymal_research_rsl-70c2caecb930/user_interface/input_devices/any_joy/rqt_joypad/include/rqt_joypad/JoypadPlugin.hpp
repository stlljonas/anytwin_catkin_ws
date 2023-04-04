/*
 * LocoJoypadPlugin.hpp
 *
 *  Created on: Jan 19, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

#include "joystick_label/Joystick.hpp"

#include <rqt_gui_cpp/plugin.h>
#include <ui_joypad_plugin.h>
#include <QWidget>
#include <QLabel>
#include <QTimer>

#include <ros/ros.h>
#include <ros/package.h>
#include <joy_manager_msgs/AnyJoy.h>

using namespace joystick_label;

namespace rqt_joypad {

/*!
 * Joystick plugin for rqt.
 * Simulates the operator controller.
 */
class JoypadPlugin : public rqt_gui_cpp::Plugin {
Q_OBJECT
public:

  enum JoyAxes {
    Heading = 1,
    Lateral = 0,
    Yaw = 3,
    Roll = 3,
    Pitch = 2,
    Vertical = 4
  };

  enum JoyButtons {
    EmergencyStop = 5,
    ButtonA = 0,
    ButtonB = 1,
    ButtonX = 2,
    ButtonY = 3,
    ButtonUp = 13,
    ButtonDown = 14
  };

  /*!
   * Constructor.
   * Needless to say, this constructor is called before initPlugin().
   */
  JoypadPlugin();

  /*!
   * Initialize rqt plugin.
   * @param context rqt plugin context.
   */
  virtual void initPlugin(qt_gui_cpp::PluginContext &context);

  /*!
   * Shutdown rqt plugin.
   */
  virtual void shutdownPlugin();

private:
  Ui::LocoJoypad ui_;
  QWidget *widget_;

  ros::Publisher joyPub_;
  joy_manager_msgs::AnyJoy anyJoyMsg_;
  QTimer *joyMsgRepeatTimer_;

  ros::ServiceClient controllerSrv_;

  Joystick *joystick_left_;
  Joystick *joystick_right_;

protected slots:

  virtual void buttonAReleasedFcn();

  virtual void buttonBReleasedFcn();

  virtual void buttonXReleasedFcn();

  virtual void buttonYReleasedFcn();

  virtual void buttonAPressedFcn();

  virtual void buttonBPressedFcn();

  virtual void buttonXPressedFcn();

  virtual void buttonYPressedFcn();

  virtual void buttonUpPressedFcn();

  virtual void buttonDownPressedFcn();

  virtual void buttonLeftPressedFcn();

  virtual void buttonRightPressedFcn();

  virtual void buttonUpReleasedFcn();

  virtual void buttonDownReleasedFcn();

  virtual void buttonLeftReleasedFcn();

  virtual void buttonRightReleasedFcn();

  virtual void buttonLeftSendValuesFcn();

  virtual void buttonRightSendValuesFcn();

  virtual void buttonStickResetFcn(bool checked);

  virtual void axisXLeftMoved();

  virtual void axisYLeftMoved();

  virtual void axisXRightMoved();

  virtual void axisYRightMoved();

  virtual void buttonResetAxesPressed();

  virtual void buttonEmergencyStopPressed();

  virtual void buttonEmergencyStopReleased();

  virtual void joystickLeftMoved(double x, double y);

  virtual void joystickRightMoved(double x, double y);

  virtual void publishAnyJoyMessage();

signals:

  void stateChanged();
};

} // namespace
