/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/DiagnosticAnymalWidget.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

DiagnosticAnymalWidget::DiagnosticAnymalWidget(QWidget *parent) :
    QWidget(parent) {

  // install event filter
  this->installEventFilter(this);
  this->setAttribute(Qt::WA_Hover);

  // fill foot id string to contact id map
  mapFootStringToContactId_["OPTOFORCE_LF_FOOT"] = 0;
  mapFootStringToContactId_["OPTOFORCE_RF_FOOT"] = 1;
  mapFootStringToContactId_["OPTOFORCE_LH_FOOT"] = 2;
  mapFootStringToContactId_["OPTOFORCE_RH_FOOT"] = 3;

  overlayPixmap_ = new QPixmap(":/images/anymal/anymal_state_battery.png");

  // Initialize repaint timer.
  repaintTimer_ = new QTimer(this);
  connect(repaintTimer_, SIGNAL(timeout()),
          this, SLOT(repaint()));
  repaintTimer_->start(50);

  connect(this, SIGNAL(updateBattery(sensor_msgs::BatteryStateConstPtr)),
          this, SLOT(onUpdateBattery(sensor_msgs::BatteryStateConstPtr)));

  // Initialize text.
  text_ = new Text(":/fonts/arialbd.ttf");
  // Get font file path for battery font.
  batteryFontPath_ = ":/fonts/battery.ttf";

  initializeComponents();
  mouseSelectionRectangle_ = new Rectangle(0, 10, 10, 0);
}

DiagnosticAnymalWidget::~DiagnosticAnymalWidget() {
  if (mouseSelectionRectangle_ != nullptr) {
    delete mouseSelectionRectangle_;
    mouseSelectionRectangle_ = nullptr;
  }
  if (text_ != nullptr) {
    delete text_;
    text_ = nullptr;
  }
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

void DiagnosticAnymalWidget::setAnymalStateMsg(
    const anymal_msgs::AnymalState &anymalState) {
  //std::lock_guard<std::mutex> lock_guard(mutexComponents_);

  anymalState_ = anymalState;
  for (auto &item : components_) {
    if (typeid(*item) == typeid(DiagnosticComponentFoot)) {
      auto *foot = dynamic_cast<DiagnosticComponentFoot *>(item);
      if (foot->isOk()) {
        if (anymalState_.contacts.size() !=
            mapFootStringToContactId_.size()) {
          ROS_WARN_STREAM_NAMED(TAG, TAG
              << " the size of contacts in anymal state is wrong");
          continue;
        }
        auto searchContactId = mapFootStringToContactId_.find(foot->id());
        if (searchContactId == mapFootStringToContactId_.end()) {
          ROS_WARN_STREAM_NAMED(TAG, TAG << " no foot with id: "
                                            << foot->id().toStdString());
          continue;
        }
        unsigned char state = anymalState_.contacts.at(
            searchContactId->second).state;
        switch (state) {
          case anymal_msgs::Contact::STATE_OPEN:
            foot->color() = colors_.swing;
            break;
          case anymal_msgs::Contact::STATE_CLOSED:
            foot->color() = colors_.stance;
            break;
          case anymal_msgs::Contact::STATE_SLIPPING:
            foot->color() = colors_.slipping;
            break;
          default:
//            foot->color() = colors_.black;
            break;
        }
      }
      if (anymalState_.header.stamp.toSec() == 0) {
        continue;
      }
      for (auto &selectedComponent : selectedComponents_) {
        if (selectedComponent->id() == foot->id()) {
          double force_x, force_y, force_z, force_norm;
          if (anymalState_.contacts.size() !=
              mapFootStringToContactId_.size()) {
            ROS_WARN_STREAM_NAMED(TAG, TAG
                << " the size of contacts in anymal state is wrong");
            continue;
          }
          auto searchContactId =
              mapFootStringToContactId_.find(selectedComponent->id());
          if (searchContactId == mapFootStringToContactId_.end()) {
            ROS_WARN_STREAM_NAMED(TAG, TAG
                << " no foot with id: "
                   << selectedComponent->id().toStdString());
            continue;
          }
          int contactId = searchContactId->second;
          force_x = anymalState_.contacts.at(contactId).wrench.force.x;
          force_y = anymalState_.contacts.at(contactId).wrench.force.y;
          force_z = anymalState_.contacts.at(contactId).wrench.force.z;
          force_norm = std::sqrt(force_x * force_x +
                                     force_y * force_y + force_z * force_z);

          selectedComponent->getTableModel()->setGridData(
              FOOT_FORCE, FOOT_FX, QString::number(force_x, 'f', 2));
          selectedComponent->getTableModel()->setGridData(
              FOOT_FORCE, FOOT_FY, QString::number(force_y, 'f', 2));
          selectedComponent->getTableModel()->setGridData(
              FOOT_FORCE, FOOT_FZ, QString::number(force_z, 'f', 2));
          selectedComponent->getTableModel()->setGridData(
              FOOT_FORCE, FOOT_NORM, QString::number(force_norm, 'f', 2));
          break;
        }
      }
    } else if (typeid(*item) == typeid(DiagnosticComponentEstimator)) {
      auto *estimator = dynamic_cast<DiagnosticComponentEstimator *>(item);
      if (estimator->isOk()) {
        switch (anymalState_.state) {
          case (anymal_msgs::AnymalState::STATE_OK):
            item->message() = "OK";
            item->color() = colors_.green;
            break;
          case (anymal_msgs::AnymalState::STATE_ERROR_UNKNOWN):
            item->message() = "Error Unknown";
            item->color() = colors_.error;
            break;
          case (anymal_msgs::AnymalState::STATE_ERROR_ESTIMATOR):
            item->message() = "Error Estimator";
            item->color() = colors_.error;
            break;
          case (anymal_msgs::AnymalState::STATE_ERROR_SENSOR):
            item->message() = "Error Sensor";
            item->color() = colors_.error;
            break;
          default:
            item->message() = "";
            item->color() = colors_.black;
            break;
        }
      }
      if (anymalState_.header.stamp.toSec() == 0) {
        continue;
      }
      for (auto &selectedComponent : selectedComponents_) {
        if (selectedComponent->id() == estimator->id()) {
          // transform estimator pose to map frame
          geometry_msgs::TransformStamped transform;
          for (auto trans : anymalState_.frame_transforms) {
            if (trans.child_frame_id.compare("map") == 0 ||
                trans.child_frame_id.compare("/map") == 0) {
              transform = trans;
              break;
            }
          }
          if (transform.child_frame_id.empty()) {
            ROS_WARN_STREAM_NAMED(TAG, TAG
                << " no transform from odom to map in anymal state");
            continue;
          }
          geometry_msgs::PoseStamped poseStamped;
          try {
            tf2::doTransform(anymalState_.pose, poseStamped, transform);
          } catch (tf2::TransformException &e) {
            ROS_WARN_STREAM_NAMED(TAG, TAG << " tf2 exception: " << e.what());
            continue;
          }
          // quaternion to euler angles
          kindr::RotationQuaternionPD quaternionOdom =
              kindr::RotationQuaternionPD(
                  anymalState_.pose.pose.orientation.w,
                  anymalState_.pose.pose.orientation.x,
                  anymalState_.pose.pose.orientation.y,
                  anymalState_.pose.pose.orientation.z);
          kindr::EulerAnglesZyxPD eulerOdom;
          eulerOdom = kindr::EulerAnglesZyxPD(quaternionOdom);
          eulerOdom.setUnique();
          kindr::RotationQuaternionPD quaternionMap =
              kindr::RotationQuaternionPD(
                  anymalState_.pose.pose.orientation.w,
                  anymalState_.pose.pose.orientation.x,
                  anymalState_.pose.pose.orientation.y,
                  anymalState_.pose.pose.orientation.z);
          kindr::EulerAnglesZyxPD eulerMap;
          eulerMap = kindr::EulerAnglesZyxPD(quaternionMap);
          eulerMap.setUnique();
          // odom
          selectedComponent->getTableModel()->setGridData(
              EST_ODOM, EST_PX, QString::number(
                  anymalState_.pose.pose.position.x, 'f', 4));
          selectedComponent->getTableModel()->setGridData(
              EST_ODOM, EST_PY, QString::number(
                  anymalState_.pose.pose.position.y, 'f', 4));
          selectedComponent->getTableModel()->setGridData(
              EST_ODOM, EST_PZ, QString::number(
                  anymalState_.pose.pose.position.z, 'f', 4));
          selectedComponent->getTableModel()->setGridData(
              EST_ODOM, EST_YAW, QString::number(
                  eulerOdom.yaw(), 'f', 4));
          selectedComponent->getTableModel()->setGridData(
              EST_ODOM, EST_PITCH, QString::number(
                  eulerOdom.pitch(), 'f', 4));
          selectedComponent->getTableModel()->setGridData(
              EST_ODOM, EST_ROLL, QString::number(
                  eulerOdom.roll(), 'f', 4));
          // map
          selectedComponent->getTableModel()->setGridData(
              EST_MAP, EST_PX, QString::number(
                  poseStamped.pose.position.x, 'f', 4));
          selectedComponent->getTableModel()->setGridData(
              EST_MAP, EST_PY, QString::number(
                  poseStamped.pose.position.y, 'f', 4));
          selectedComponent->getTableModel()->setGridData(
              EST_MAP, EST_PZ, QString::number(
                  poseStamped.pose.position.z, 'f', 4));
          selectedComponent->getTableModel()->setGridData(
              EST_MAP, EST_YAW, QString::number(eulerMap.yaw(), 'f', 4));
          selectedComponent->getTableModel()->setGridData(
              EST_MAP, EST_PITCH, QString::number(eulerMap.pitch(), 'f', 4));
          selectedComponent->getTableModel()->setGridData(
              EST_MAP, EST_ROLL, QString::number(eulerMap.roll(), 'f', 4));
          break;
        }
      }
    }
  }
}

void DiagnosticAnymalWidget::setActuatorReadings(
    const ActuatorReadings &readings) {
  std::lock_guard<std::mutex> lock_guard(mutexComponents_);

#ifdef ANYDRIVE1X
  using namespace anydrive::state;
#else
  using namespace anydrive::fsm;
#endif

  int actuatorId = 0;
  for (auto &item : components_) {
    if (typeid(*item) == typeid(DiagnosticComponentSea)) {
      auto *sea = dynamic_cast<DiagnosticComponentSea *>(item);
      if (actuatorId >= readings.readings.size()) {
        // ROS_WARN_STREAM_NAMED(TAG, TAG
        //     << " Could not find drive id: " << sea->id());
        continue;
      }
      ros::Time time = ros::Time::now();
      // Decrypt statusword.
      int level = -1;
      uint32_t statuswordUint32 =
          readings.readings[actuatorId].state.statusword;
      anydrive::Statusword statusword(statuswordUint32);
      sea->mode() = QString::fromStdString(anydrive::mode::modeEnumToShortName(
          statusword.getModeEnum()));
      StateEnum stateEnum = statusword.getStateEnum();
      std::string stateName = stateEnumToName(stateEnum);
      std::string modeName =
          anydrive::mode::modeEnumToName(statusword.getModeEnum());
#ifdef ANYDRIVE1X
      std::vector<std::string> errors = statusword.getErrors();
#else
      std::vector<std::string> infos, warnings, errors, fatals;
      statusword.getMessages(infos, warnings, errors, fatals);
#endif
      // Update sea color.
      if (!fatals.empty() || stateEnum == anydrive::fsm::StateEnum::Fatal) {
        sea->color() = colors_.fatal;
      } else if (!errors.empty() || stateEnum == anydrive::fsm::StateEnum::Error) {
        sea->color() = colors_.error;
      } else if (!warnings.empty()) {
        sea->color() = colors_.warning;
      } else {
        sea->color() = colors_.ok;
      }
      // Check if the same as last time. If it is the same, update only the
      // timestamp. Otherwise, add a new message to the sea.
      message_t latestMessage;
      if (item->getNumberOfMessages() > 0) {
        latestMessage = item->getLatestMessage();
      }
      if (item->getNumberOfMessages() == 0 ||
          !latestMessage.compare(statuswordUint32) ||
          level != latestMessage.level) {
        // Build the new message.
        std::string message = "";
        message += "Mode: " + modeName + "\n";
        message += "State: " + stateName;
        if (errors.size() > 0) {
          message += "\nErrors:";
          for (auto error : errors) {
            message += "\n  " + error;
          }
        }
        // Set the new message.
        QString qmessage = QString::fromStdString(message);
        item->message() = qmessage;
        item->pushMessage(qmessage, level, time, (int)statuswordUint32);
        pushMessage(qmessage, level, time, item->id());
      } else {
        item->updateLatestMessageTime(time);
      }

      // Update table.
      for (auto &selectedComponent : selectedComponents_) {
        if (selectedComponent->id() == sea->id()) {

          // state
          selectedComponent->getTableModel()->setGridData(
              SEA_STATE, SEA_POSITION, QString::number(
                  readings.readings[actuatorId].state.joint_position, 'f', 2));
          selectedComponent->getTableModel()->setGridData(
              SEA_STATE, SEA_VELOCITY, QString::number(
                  readings.readings[actuatorId].state.joint_velocity, 'f', 2));
          selectedComponent->getTableModel()->setGridData(
              SEA_STATE, SEA_TORQUE, QString::number(
                  readings.readings[actuatorId].state.joint_torque, 'f', 2));
          selectedComponent->getTableModel()->setGridData(
              SEA_STATE, SEA_CURRENT, QString::number(
                  readings.readings[actuatorId].state.current, 'f', 2));
          // commanded
#ifdef ANYDRIVE1X
          const double jointPosition =
              readings.readings[actuatorId].commanded.position;
          const double jointVelocity =
              readings.readings[actuatorId].commanded.velocity;
#else
          const double jointPosition =
              readings.readings[actuatorId].commanded.joint_position;
          const double jointVelocity =
              readings.readings[actuatorId].commanded.joint_velocity;
#endif
          selectedComponent->getTableModel()->setGridData(
              SEA_COMMANDED, SEA_POSITION, QString::number(jointPosition, 'f', 2));
          selectedComponent->getTableModel()->setGridData(
              SEA_COMMANDED, SEA_VELOCITY, QString::number(jointVelocity, 'f', 2));
          selectedComponent->getTableModel()->setGridData(
              SEA_COMMANDED, SEA_TORQUE, QString::number(
                  readings.readings[actuatorId].commanded.joint_torque, 'f', 2));
          selectedComponent->getTableModel()->setGridData(
              SEA_COMMANDED, SEA_CURRENT, QString::number(
                  readings.readings[actuatorId].commanded.current, 'f', 2));
          break;
        }
      }
      actuatorId++;
    }
  }
  emit updateNotificationsTableFlag();
}

void DiagnosticAnymalWidget::setImu(const sensor_msgs::ImuConstPtr &msg) {
  std::lock_guard<std::mutex> lock_guard(mutexComponents_);

  for (auto &item : components_) {
    if (typeid(*item) == typeid(DiagnosticComponentImu)) {
      auto *imu = dynamic_cast<DiagnosticComponentImu *>(item);
      for (auto &selectedComponent : selectedComponents_) {
        if (selectedComponent->id() == imu->id()) {
          // state
          selectedComponent->getTableModel()->setGridData(
              IMU_VALUE, IMU_LX,
              QString::number(msg->linear_acceleration.x, 'f', 4));
          selectedComponent->getTableModel()->setGridData(
              IMU_VALUE, IMU_LY,
              QString::number(msg->linear_acceleration.y, 'f', 4));
          selectedComponent->getTableModel()->setGridData(
              IMU_VALUE, IMU_LZ,
              QString::number(msg->linear_acceleration.z, 'f', 4));
          selectedComponent->getTableModel()->setGridData(
              IMU_VALUE, IMU_AX,
              QString::number(msg->angular_velocity.x, 'f', 4));
          selectedComponent->getTableModel()->setGridData(
              IMU_VALUE, IMU_AY,
              QString::number(msg->angular_velocity.y, 'f', 4));
          selectedComponent->getTableModel()->setGridData(
              IMU_VALUE, IMU_AZ,
              QString::number(msg->angular_velocity.z, 'f', 4));
          break;
        }
      }
      break;
    }
  }
}

void DiagnosticAnymalWidget::setNotification(
    const notification::Notification &notification) {
  std::lock_guard<std::mutex> lock_guard(mutexComponents_);

  QString id = QString::fromStdString(notification.id_);
  QString message = QString::fromStdString(notification.description_);
  int level = -1;
  switch (notification.getLevel()) {
    case notification::Level::LEVEL_INFO:
      level = OK;
      break;
    case notification::Level::LEVEL_WARN:
      level = WARN;
      break;
    case notification::Level::LEVEL_ERROR:
    case notification::Level::LEVEL_FATAL:
      level = ERROR;
      break;
    default:
      break;
  }
  if (level != -1) {
    updateComponent(id, level, message, notification.header_.stamp);
  }
}

void DiagnosticAnymalWidget::setDiagnostic(
    const diagnostic_msgs::DiagnosticStatus &status) {
  std::lock_guard<std::mutex> lock_guard(mutexComponents_);

  QString id = QString::fromStdString(status.hardware_id);
  QString message = QString::fromStdString(status.message);
  int level = -1;
  switch (status.level) {
    case diagnostic_msgs::DiagnosticStatus::OK:
      level = OK;
      break;
    case diagnostic_msgs::DiagnosticStatus::WARN:
      level = WARN;
      break;
    case diagnostic_msgs::DiagnosticStatus::ERROR:
    case diagnostic_msgs::DiagnosticStatus::STALE:
      level = ERROR;
      break;
    default:
      break;
  }
  if (level != -1) {
    updateComponent(id, level, message, ros::Time::now());
  }
}

const std::deque<message_t> &DiagnosticAnymalWidget::getMessages() const {
  return messages_;
}

void DiagnosticAnymalWidget::setBatteryState(
    const sensor_msgs::BatteryStateConstPtr &msg) {
  emit updateBattery(msg);
}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */

void DiagnosticAnymalWidget::initializeComponents() {
  // add components
  // sea
  components_.push_back(
      new DiagnosticComponentSea(
          "ANYDRIVE LF_HAA", "LF_HAA", QPoint(59, 56),
          QSize(33, 36), colors_.gray, colors_.red, "", QPoint(59, 85),
          QSize(32, 7), colors_.blue, QPoint(62, 76), colors_.text));
  components_.push_back(
      new DiagnosticComponentSea(
          "ANYDRIVE LF_HFE", "LF_HFE", QPoint(64, 18),
          QSize(36, 33), colors_.gray, colors_.red, "", QPoint(93, 17),
          QSize(7, 32), colors_.blue, QPoint(69, 39), colors_.text));
  components_.push_back(
      new DiagnosticComponentSea(
          "ANYDRIVE LF_KFE", "LF_KFE", QPoint(23, 85),
          QSize(36, 33), colors_.gray, colors_.red, "", QPoint(52, 84),
          QSize(7, 32), colors_.blue, QPoint(29, 106), colors_.text));
  components_.push_back(
      new DiagnosticComponentSea(
          "ANYDRIVE RF_HAA", "RF_HAA", QPoint(147, 56),
          QSize(33, 36), colors_.gray, colors_.red, "", QPoint(147, 85),
          QSize(32, 7), colors_.blue, QPoint(150, 76), colors_.text));
  components_.push_back(
      new DiagnosticComponentSea(
          "ANYDRIVE RF_HFE", "RF_HFE", QPoint(139, 18),
          QSize(36, 33), colors_.gray, colors_.red, "", QPoint(138, 17),
          QSize(7, 32), colors_.blue, QPoint(144, 39), colors_.text));
  components_.push_back(
      new DiagnosticComponentSea(
          "ANYDRIVE RF_KFE", "RF_KFE", QPoint(180, 85),
          QSize(36, 33), colors_.gray, colors_.red, "", QPoint(180, 84),
          QSize(7, 32), colors_.blue, QPoint(184, 106), colors_.text));
  components_.push_back(
      new DiagnosticComponentSea(
          "ANYDRIVE LH_HAA", "LH_HAA", QPoint(59, 214),
          QSize(33, 36), colors_.gray, colors_.red, "", QPoint(59, 213),
          QSize(32, 7), colors_.blue, QPoint(62, 239), colors_.text));
  components_.push_back(
      new DiagnosticComponentSea(
          "ANYDRIVE LH_HFE", "LH_HFE", QPoint(64, 255),
          QSize(36, 33), colors_.gray, colors_.red, "", QPoint(93, 254),
          QSize(7, 32), colors_.blue, QPoint(69, 276), colors_.text));
  components_.push_back(
      new DiagnosticComponentSea(
          "ANYDRIVE LH_KFE", "LH_KFE", QPoint(23, 188),
          QSize(36, 33), colors_.gray, colors_.red, "", QPoint(52, 187),
          QSize(7, 32), colors_.blue, QPoint(29, 209), colors_.text));
  components_.push_back(
      new DiagnosticComponentSea(
          "ANYDRIVE RH_HAA", "RH_HAA", QPoint(147, 214),
          QSize(33, 36), colors_.gray, colors_.red, "", QPoint(147, 213),
          QSize(32, 7), colors_.blue, QPoint(150, 239), colors_.text));
  components_.push_back(
      new DiagnosticComponentSea(
          "ANYDRIVE RH_HFE", "RH_HFE", QPoint(139, 255),
          QSize(36, 33), colors_.gray, colors_.red, "", QPoint(137, 254),
          QSize(7, 32), colors_.blue, QPoint(144, 276), colors_.text));
  components_.push_back(
      new DiagnosticComponentSea(
          "ANYDRIVE RH_KFE", "RH_KFE", QPoint(180, 188),
          QSize(36, 33), colors_.gray, colors_.red, "", QPoint(180, 187),
          QSize(7, 32), colors_.blue, QPoint(184, 209), colors_.text));
  // foot
  components_.push_back(
      new DiagnosticComponentFoot(
          "Foot LH", "OPTOFORCE_LH_FOOT", QPoint(3, 278), QSize(24, 24),
          colors_.gray, colors_.red, ""));
  components_.push_back(
      new DiagnosticComponentFoot(
          "Foot RH", "OPTOFORCE_RH_FOOT", QPoint(211, 278), QSize(24, 24),
          colors_.gray, colors_.red, ""));
  components_.push_back(
      new DiagnosticComponentFoot(
          "Foot LF", "OPTOFORCE_LF_FOOT", QPoint(3, 3), QSize(24, 24),
          colors_.gray, colors_.red, ""));
  components_.push_back(
      new DiagnosticComponentFoot(
          "Foot RF", "OPTOFORCE_RF_FOOT", QPoint(211, 3), QSize(24, 24),
          colors_.gray, colors_.red, ""));
  // estimator
  components_.push_back(
      new DiagnosticComponentEstimator(
          "Estimator", "ESTIMATOR", QPoint(107, 116), QSize(24, 18),
          colors_.gray, colors_.red, ""));
  // lowlevel controller llc
  components_.push_back(
      new DiagnosticComponentLlc(
          "Lowlevel controller", "LLC", QPoint(107, 134), QSize(24, 18),
          colors_.gray, colors_.red, ""));
  // locomotion controller lmc
  components_.push_back(
      new DiagnosticComponentLmc(
          "Locomotion controller", "LMC", QPoint(107, 152), QSize(24, 18),
          colors_.gray, colors_.red, ""));
  // imu
  components_.push_back(
      new DiagnosticComponentImu(
          "IMU", "IMU", QPoint(110, 178), QSize(18, 18), colors_.gray,
          colors_.red, ""));
  // battery
  components_.push_back(
      new DiagnosticComponentBattery(
          "Battery", "Battery", QPoint(81, 116), QSize(23, 88), colors_.gray,
          colors_.red, "", batteryFontPath_.c_str(), QPoint(77, 172)));

  // tests
//    dynamic_cast<DiagnosticComponentSea*>(components_.front())->isSelected() = true;
//    dynamic_cast<DiagnosticComponentFoot*>(components_.back())->isSelected() = true;
//    dynamic_cast<DiagnosticComponentFoot*>(components_.back())->message() = "bla bla";
}

bool DiagnosticAnymalWidget::isInsideRectangle(QPoint point,
                                                  QPoint rectanglePoint,
                                                  QSize rectangleSize) {
  return (point.x() > rectanglePoint.x() &&
          point.x() < rectanglePoint.x() + rectangleSize.width() &&
          point.y() > rectanglePoint.y() &&
          point.y() < rectanglePoint.y() + rectangleSize.height());
}

void DiagnosticAnymalWidget::updateComponent(const QString &id, int level,
                                                const QString &message,
                                                const ros::Time &time) {
  for (auto &item : components_) {
    if (id.compare(item->id()) == 0) {
      item->message() = message;
      item->pushMessage(message, level, time);
      if (typeid(*item) == typeid(DiagnosticComponentSea)) {
        auto *sea = dynamic_cast<DiagnosticComponentSea *>(item);
        switch (level) {
          case OK:
            sea->color() = colors_.blue;
            break;
          case WARN:
            sea->color() = colors_.warning;
            break;
          case ERROR:
            sea->color() = colors_.error;
            break;
          default:
            sea->color() = colors_.gray;
            break;
        }
      } else if (typeid(*item) == typeid(DiagnosticComponentFoot)) {
        auto *foot = dynamic_cast<DiagnosticComponentFoot *>(item);
        switch (level) {
          case OK:
            foot->color() = colors_.green;
            foot->isOk() = true;
            break;
          case WARN:
            foot->color() = colors_.warning;
            foot->isOk() = false;
            break;
          case ERROR:
            foot->color() = colors_.error;
            foot->isOk() = false;
            break;
          default:
            foot->color() = colors_.gray;
            foot->isOk() = true;
            break;
        }
      } else if (typeid(*item) == typeid(DiagnosticComponentEstimator)) {
        auto *estimator = dynamic_cast<DiagnosticComponentEstimator *>(item);
        switch (level) {
          case OK:
            estimator->color() = colors_.green;
            estimator->isOk() = true;
            break;
          case WARN:
            estimator->color() = colors_.warning;
            estimator->isOk() = false;
            break;
          case ERROR:
            estimator->color() = colors_.error;
            estimator->isOk() = false;
            break;
          default:
            estimator->color() = colors_.gray;
            estimator->isOk() = true;
            break;
        }
      } else {
        switch (level) {
          case OK:
            item->color() = colors_.green;
            break;
          case WARN:
            item->color() = colors_.warning;
            break;
          case ERROR:
            item->color() = colors_.error;
            break;
          default:
            item->color() = colors_.gray;
            break;
        }
      }
      break;
    }
  }

  pushMessage(message, level, time, id);
  emit updateNotificationsTableFlag();
}

void DiagnosticAnymalWidget::pushMessage(const QString &message, int level,
                                            const ros::Time &time,
                                            const QString &id) {
  message_t message_t1;
  message_t1.message = message;
  message_t1.level = level;
  message_t1.time = time;
  message_t1.id = id;
  messages_.push_front(message_t1);
  if (messages_.size() > messagesDequeSize_) {
    messages_.pop_back();
  }
}

/* ========================================================================== */
/* Events                                                                     */
/* ========================================================================== */

bool DiagnosticAnymalWidget::eventFilter(QObject *object, QEvent *event) {
  std::lock_guard<std::mutex> lock_guard(mutexComponents_);

  // click event
  if (event->type() == QEvent::MouseButtonPress) {
    auto *mouseEvent = static_cast<QMouseEvent *>(event);
    startPoint_ = mouseEvent->pos();
    endPoint_ = startPoint_;
    mouseIsPressed_ = true;
  }
  // move event
  if (event->type() == QEvent::MouseMove && mouseIsPressed_) {
    auto *mouseEvent = static_cast<QMouseEvent *>(event);
    QPoint point = mouseEvent->pos();
    if (point.x() < 0) { point.setX(0); }
    if (point.x() > this->width()) { point.setX(this->width()); }
    if (point.y() < 0) { point.setY(0); }
    if (point.y() > this->height()) { point.setY(this->height()); }
    endPoint_ = point;

    preSelectedComponents_.clear();
    for (auto item : components_) {
      QRect rectSelection;
      if (startPoint_.x() < endPoint_.x()) {
        rectSelection.setX(startPoint_.x());
        rectSelection.setWidth(endPoint_.x() - startPoint_.x());
      } else {
        rectSelection.setX(endPoint_.x());
        rectSelection.setWidth(startPoint_.x() - endPoint_.x());
      }
      if (startPoint_.y() < endPoint_.y()) {
        rectSelection.setY(startPoint_.y());
        rectSelection.setHeight(endPoint_.y() - startPoint_.y());
      } else {
        rectSelection.setY(endPoint_.y());
        rectSelection.setHeight(startPoint_.y() - endPoint_.y());
      }
      QRect rectItem(item->point(), item->size());
      if (rectSelection.left() < rectItem.right() &&
          rectSelection.right() > rectItem.left() &&
          rectSelection.bottom() > rectItem.top() &&
          rectSelection.top() < rectItem.bottom()) {
        preSelectedComponents_.push_back(item);
      }
    }
  }
  // release event
  if (event->type() == QEvent::MouseButtonRelease) {
    auto *mouseEvent = static_cast<QMouseEvent *>(event);
    QPoint point = mouseEvent->pos();
    if (point.x() < 0) { point.setX(0); }
    if (point.x() > this->width()) { point.setX(this->width()); }
    if (point.y() < 0) { point.setY(0); }
    if (point.y() > this->height()) { point.setY(this->height()); }
    endPoint_ = point;
    mouseIsPressed_ = false;
    preSelectedComponents_.clear();
//    ROS_INFO_STREAM_NAMED(TAG, TAG
//        << " area: "
//        << abs(startPoint_.x()-endPoint_.x())*
//           abs(startPoint_.y()-endPoint_.y()));
    if (abs(startPoint_.x() - endPoint_.x()) *
            abs(startPoint_.y() - endPoint_.y()) >= 100) {
      // check for components inside selection
      if (Qt::ControlModifier == QApplication::queryKeyboardModifiers()) {
      } else {
        selectedComponents_.clear();
      }
      for (auto item : components_) {
        bool isAlreadySelected = false;
        for (auto selectedComponent : selectedComponents_) {
          if (selectedComponent->id() == item->id()) {
            isAlreadySelected = true;
            break;
          }
        }
        if (isAlreadySelected) {
          continue;
        }
        QRect rectSelection;
        if (startPoint_.x() < endPoint_.x()) {
          rectSelection.setX(startPoint_.x());
          rectSelection.setWidth(endPoint_.x() - startPoint_.x());
        } else {
          rectSelection.setX(endPoint_.x());
          rectSelection.setWidth(startPoint_.x() - endPoint_.x());
        }
        if (startPoint_.y() < endPoint_.y()) {
          rectSelection.setY(startPoint_.y());
          rectSelection.setHeight(endPoint_.y() - startPoint_.y());
        } else {
          rectSelection.setY(endPoint_.y());
          rectSelection.setHeight(startPoint_.y() - endPoint_.y());
        }
        QRect rectItem(item->point(), item->size());
        if (rectSelection.left() < rectItem.right() &&
            rectSelection.right() > rectItem.left() &&
            rectSelection.bottom() > rectItem.top() &&
            rectSelection.top() < rectItem.bottom()) {
          selectedComponents_.push_back(item);
        }
      }
      emit updateSelectedComponents(selectedComponents_);
      emit updateNotificationsTableFlag();
    } else {
      for (auto item : components_) {
        if (isInsideRectangle(mouseEvent->pos(), item->point(), item->size())) {
          bool isSelected = false;
          auto it = selectedComponents_.begin();
          while (it != selectedComponents_.end()) {
            if (item->id() == (*it)->id()) {
              selectedComponents_.erase(it);
              isSelected = true;
              break;
            } else {
              ++it;
            }
          }
          if (Qt::ControlModifier == QApplication::queryKeyboardModifiers()) {
            if (!isSelected) {
              selectedComponents_.push_back(item);
            }
          } else {
            if (/*isSelected && selectedComponents_.size() > 1||*/!isSelected) {
              selectedComponents_.clear();
              selectedComponents_.push_back(item);
            } else {
              selectedComponents_.clear();
            }
          }
          emit updateSelectedComponents(selectedComponents_);
          emit updateNotificationsTableFlag();
          break;
        }
      }
    }
  }
  // tool tip
  if (event->type() == QEvent::HoverMove) {
    QHoverEvent *hoverEvent = static_cast<QHoverEvent *>(event);
    bool isToolTip = false;
    for (auto item : components_) {
      if (isInsideRectangle(hoverEvent->pos(), item->point(), item->size())) {
        QString toolTipStr;
        toolTipStr.append(item->name());
        if (!item->message().isEmpty()) {
          toolTipStr.append("\n").append(item->message());
        }
        toolTip_->showText(this->mapToGlobal(hoverEvent->pos()), toolTipStr);
        isToolTip = true;
        break;
      }
    }
    if (!isToolTip) {
      toolTip_->hideText();
    }
  }
  return QWidget::eventFilter(object, event);
}

//#define PAINT_EVENT_TIMING
void DiagnosticAnymalWidget::paintEvent(QPaintEvent *event) {
  std::lock_guard<std::mutex> lock_guard(mutexComponents_);

#ifdef PAINT_EVENT_TIMING
  static int counter = 0;
  static double seconds = 0.0;

  ros::Time start = ros::Time::now();
#endif

  // Initialize painter.
  auto *painter = new QPainter(this);
  painter->setRenderHint(QPainter::Antialiasing);

  // Draw components.
  for (auto item : components_) {
    if (typeid(*item) == typeid(DiagnosticComponentSea)) {
      auto *sea = dynamic_cast<DiagnosticComponentSea *>(item);
      sea->drawDiagnosticRectangle(painter);
    } else if (typeid(*item) == typeid(DiagnosticComponentFoot)) {
      auto *foot = dynamic_cast<DiagnosticComponentFoot *>(item);
      foot->drawDiagnosticRectangle(painter);
    } else if (typeid(*item) == typeid(DiagnosticComponentEstimator)) {
      auto *estimator = dynamic_cast<DiagnosticComponentEstimator *>(item);
      estimator->drawDiagnosticRectangle(painter);
    } else if (typeid(*item) == typeid(DiagnosticComponentBattery)) {
      auto *battery = dynamic_cast<DiagnosticComponentBattery *>(item);
      battery->drawBatteryLevel(painter);
      battery->drawChargingState(painter);
    } else {
      item->drawDiagnosticRectangle(painter);
    }
  }

  // Draw overlay image.
  painter->save();
  painter->drawPixmap(0, 0, *overlayPixmap_);
  painter->restore();

  // Draw sea control mode text.
  for (auto item : components_) {
    if (typeid(*item) == typeid(DiagnosticComponentSea)) {
      auto *sea = dynamic_cast<DiagnosticComponentSea *>(item);
      text_->renderText(painter, sea->mode(),
                        sea->controlModePoint().x(),
                        sea->controlModePoint().y(),
                        sea->controlModeColor());
    }
  }

  // Draw component selection.
  for (auto item : selectedComponents_) {
    if (typeid(*item) == typeid(DiagnosticComponentSea)) {
      auto *sea = dynamic_cast<DiagnosticComponentSea *>(item);
      sea->drawSelectionRectangleBorder(painter);
    } else if (typeid(*item) == typeid(DiagnosticComponentFoot)) {
      auto *foot = dynamic_cast<DiagnosticComponentFoot *>(item);
      foot->drawSelectionCircle(painter);
    } else if (typeid(*item) == typeid(DiagnosticComponentEstimator)) {
      auto *estimator = dynamic_cast<DiagnosticComponentEstimator *>(item);
      estimator->drawSelectionRectangleBorder(painter);
    } else if (typeid(*item) == typeid(DiagnosticComponentBattery)) {
      auto *battery = dynamic_cast<DiagnosticComponentBattery *>(item);
      battery->drawSelectionRectangleBorder(painter);
    } else {
      item->drawSelectionRectangleBorder(painter);
    }
  }

  // Draw preselection.
  if (!preSelectedComponents_.empty()) {
    for (auto item : preSelectedComponents_) {
      if (typeid(*item) == typeid(DiagnosticComponentFoot)) {
        item->drawCircle(painter, colors_.selection);
        QColor color(colors_.selection);
        color.setAlpha(255);
        item->drawCircleBorder(painter, color);
      } else {
        item->drawRectangle(painter, colors_.selection);
        QColor color(colors_.selection);
        color.setAlpha(255);
        item->drawRectangleBorder(painter, color);
      }
    }
  }

  // Draw mouse selection area.
  if (mouseIsPressed_) {
    if (startPoint_ != endPoint_) {
      mouseSelectionRectangle_->setRectangle(startPoint_.x(), startPoint_.y(),
                                             endPoint_.x(), endPoint_.y());
      mouseSelectionRectangle_->draw(painter, colors_.selection);
      mouseSelectionRectangle_->drawBorder(painter,
                                           QColor(colors_.selection.red(),
                                                  colors_.selection.green(),
                                                  colors_.selection.blue(),
                                                  255),
                                           startPoint_.x(), startPoint_.y(),
                                           endPoint_.x(), endPoint_.y());
    }
  }

  painter->end();

#ifdef PAINT_EVENT_TIMING
  ros::Duration duration = ros::Time::now() - start;
  seconds += duration.toSec();
  counter++;

  if (counter % 100 == 0) {
    ROS_INFO_STREAM_NAMED(TAG, TAG << " time: " << seconds / (double)counter);
    counter = 0;
    seconds = 0.0;
  }
#endif
}

/* ========================================================================== */
/* Slots                                                                      */
/* ========================================================================== */

void DiagnosticAnymalWidget::onUpdateBattery(
    const sensor_msgs::BatteryStateConstPtr &msg) {
  std::lock_guard<std::mutex> lock_guard(mutexComponents_);

  for (auto &item : components_) {
    if (typeid(*item) == typeid(DiagnosticComponentBattery)) {
      auto *battery = dynamic_cast<DiagnosticComponentBattery *>(item);

      battery->setPercentage(msg->percentage);

      battery->getTableModel()->setGridData(
          BATTERY_VALUE, BATTERY_VOLTAGE, QString::number(
              (double)msg->voltage, 'f', 2));
      battery->getTableModel()->setGridData(
          BATTERY_VALUE, BATTERY_CURRENT, QString::number(
              (double)msg->current, 'f', 2));
      battery->getTableModel()->setGridData(
          BATTERY_VALUE, BATTERY_PERCENTAGE, QString::number(
              (double)msg->percentage * 100.0, 'f', 2));

      QString health = "";
      switch (msg->power_supply_health) {
        case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN:
          health = "unknown";
          break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD:
          health = "good";
          break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT:
          health = "overheat";
          break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD:
          health = "dead";
          break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE:
          health = "overvoltage";
          break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE:
          health = "unspec failure";
          break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_COLD:
          health = "cold";
          break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE:
          health = "watchdog timer expire";
          break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE:
          health = "safety timer expire";
          break;
        default:
          health = "???";
      }
      battery->getTableModel()->setGridData(BATTERY_VALUE, BATTERY_HEALTH,
                                            health);

      QString status = "";
      switch (msg->power_supply_status) {
        case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN:
          battery->setChargingState(BatteryChargingState::UNKNOWN);
          status = "unknown";
          break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING:
          battery->setChargingState(BatteryChargingState::CHARGING);
          status = "charging";
          break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING:
          battery->setChargingState(BatteryChargingState::NOT_CHARGING);
          status = "discharging";
          break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING:
          battery->setChargingState(BatteryChargingState::NOT_CHARGING);
          status = "not charging";
          break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL:
          battery->setChargingState(BatteryChargingState::NOT_CHARGING);
          status = "full";
          break;
        default:
          battery->setChargingState(BatteryChargingState::UNKNOWN);
          status = "???";
      }
      battery->getTableModel()->setGridData(BATTERY_VALUE, BATTERY_STATUS,
                                            status);

      for (int i = 0; i < msg->cell_voltage.size(); ++i) {
        int row = BATTERY_CELL_1 + i;
        if (row < battery->getTableModel()->rowCount()) {
          battery->getTableModel()->setGridData(
              BATTERY_VALUE, row,
              QString::number((double)msg->cell_voltage[i], 'f', 2));
        } else {
          break;
        }
      }

      break;
    }
  }
}

} // namespace
