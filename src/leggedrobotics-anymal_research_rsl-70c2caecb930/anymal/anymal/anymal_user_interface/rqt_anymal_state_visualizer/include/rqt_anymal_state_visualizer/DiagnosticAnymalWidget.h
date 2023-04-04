/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#pragma once

// intern
#include <rqt_anymal_state_visualizer/DiagnosticComponentBase.h>
#include <rqt_anymal_state_visualizer/DiagnosticComponentBattery.h>
#include <rqt_anymal_state_visualizer/DiagnosticComponentSea.h>
#include <rqt_anymal_state_visualizer/DiagnosticComponentFoot.h>
#include <rqt_anymal_state_visualizer/DiagnosticComponentEstimator.h>
#include <rqt_anymal_state_visualizer/DiagnosticComponentImu.h>
#include <rqt_anymal_state_visualizer/DiagnosticComponentLlc.h>
#include <rqt_anymal_state_visualizer/DiagnosticComponentLmc.h>
#include <rqt_anymal_state_visualizer/Text.h>
// std
#include <mutex>
#include <deque>
// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <notification/Notification.hpp>
#include <notification/Level.hpp>
#include <anydrive/Statusword.hpp>
// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// msgs
#ifdef ANYDRIVE1X
#include <series_elastic_actuator_msgs/SeActuatorCommands.h>
#include <series_elastic_actuator_msgs/SeActuatorReadingsExtended.h>
#else
#include <anydrive_msgs/Commands.h>
#include <anydrive_msgs/ReadingsExtended.h>
#endif
#include <anymal_msgs/AnymalState.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
// QT
#include <QEvent>
#include <QImage>
#include <QPixmap>
#include <QTimer>
#include <QMouseEvent>
#include <QHoverEvent>
#include <QToolTip>
#include <QApplication>
// kindr
#include "kindr/Core"

namespace rqt_anymal_state_visualizer {

#ifdef ANYDRIVE1X
using ActuatorCommands = series_elastic_actuator_msgs::SeActuatorCommands;
using ActuatorReadings = series_elastic_actuator_msgs::SeActuatorReadingsExtended;
#else
using ActuatorCommands = anydrive_msgs::Commands;
using ActuatorReadings = anydrive_msgs::ReadingsExtended;
#endif

class DiagnosticAnymalWidget : public QWidget {
Q_OBJECT
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  explicit DiagnosticAnymalWidget(QWidget *parent = nullptr);

  ~DiagnosticAnymalWidget();

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  void setAnymalStateMsg(
      const anymal_msgs::AnymalState &anymalState);

  void setActuatorReadings(
      const ActuatorReadings &readings);

  void setNotification(const notification::Notification &notification);

  void setDiagnostic(const diagnostic_msgs::DiagnosticStatus &status);

  void setImu(const sensor_msgs::ImuConstPtr &msg);

  void setBatteryState(const sensor_msgs::BatteryStateConstPtr &msg);

  const std::deque<message_t> &getMessages() const;

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  std::map<QString, unsigned int> mapFootStringToContactId_;

  const int messagesDequeSize_ = 100;

  /* ======================================================================== */
  /* Structs                                                                  */
  /* ======================================================================== */

  struct colors_t {
    QColor blue = QColor(37, 70, 150);
    QColor green = QColor(17, 171, 30);
    QColor red = QColor(255, 0, 0);
    QColor ok = QColor(0, 255, 0);
    QColor error = QColor(255, 0, 0);
    QColor fatal = QColor(128, 0, 255);
    QColor stance = QColor(37, 70, 150);
    QColor swing = QColor(197, 215, 255);
    QColor slipping = QColor(255, 0, 251);
    QColor warning = QColor(255, 128, 0);
    QColor text = QColor(0, 0, 0);
    QColor black = QColor(0, 0, 0);
    QColor white = QColor(255, 255, 255);
    QColor actuator = QColor(237, 237, 237);
    QColor gray = QColor(156, 156, 156);
    QColor selection = QColor(255, 104, 54, 100);
  } colors_;

private:

  /* ======================================================================== */
  /* Constants                                                                */
  /* ======================================================================== */

  const std::string TAG = "DiagnosticAnymalWidget";

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  std::vector<DiagnosticComponentBase *> components_;
  std::vector<DiagnosticComponentBase *> selectedComponents_;
  std::vector<DiagnosticComponentBase *> preSelectedComponents_;
  DiagnosticComponentBase *selectedComponent_ = NULL;

  std::mutex mutexComponents_;

  anymal_msgs::AnymalState anymalState_;

  const double maxXCoord_ = 231.0;
  const double maxYCoord_ = 298.0;

  QToolTip *toolTip_;

  std::deque<message_t> messages_;

  std::string batteryFontPath_;

  // mouse selection area
  bool mouseIsPressed_ = false;
  QPoint startPoint_;
  QPoint endPoint_;
  Rectangle *mouseSelectionRectangle_ = nullptr;

  // text
  Text *text_ = nullptr;


  QTimer *repaintTimer_;

  // Overlay image.
  QPixmap *overlayPixmap_ = nullptr;

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  bool isInsideRectangle(QPoint point, QPoint rectanglePoint,
                         QSize rectangleSize);

  void updateComponent(const QString &id, int level, const QString &message,
                       const ros::Time &time);

  void pushMessage(const QString &message, int level, const ros::Time &time,
                   const QString &id);

  void initializeComponents();

  /* ======================================================================== */
  /* Events                                                                   */
  /* ======================================================================== */

  bool eventFilter(QObject *object, QEvent *event) override;

  void paintEvent(QPaintEvent *event) override;

protected slots:

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  void onUpdateBattery(const sensor_msgs::BatteryStateConstPtr &msg);

signals:

  /* ======================================================================== */
  /* Signals                                                                  */
  /* ======================================================================== */

  void updateNotificationsTableFlag();

  void updateSelectedComponents(
      std::vector<DiagnosticComponentBase *> components);

  void updateBattery(const sensor_msgs::BatteryStateConstPtr &msg);
};

} // namespace
