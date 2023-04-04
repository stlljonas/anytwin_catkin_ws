#pragma once

#include <atomic>
#include <mutex>

#include <ros/ros.h>

#include <anydrive/ReadingExtended.hpp>

#include <anydrive_msgs/ReadingExtended.h>

#include <anydrive_monitor/AnydriveInterface.h>

namespace rqt_anydrive_monitor {

class AnydriveInterfaceRos : public anydrive_monitor::AnydriveInterface {
  Q_OBJECT

 public:
  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  AnydriveInterfaceRos(ros::NodeHandle nh, std::string deviceName, std::string rosPrefix, bool enableSubscriber, QWidget* parent = nullptr);

  ~AnydriveInterfaceRos() override = default;

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  void startup();

  void shutdown();

  void updateAnydrive();

  /* ======================================================================== */
  /* Callbacks                                                                */
  /* ======================================================================== */

  void readingCallback(const anydrive_msgs::ReadingExtended& msg);

 private:
  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  ros::NodeHandle nh_;
  const std::string deviceName_;
  const std::string rosPrefix_;
  const bool enableSubscriber_;

  ros::Subscriber readingSubscriber_;
  ros::ServiceClient setGoalStateClient_;
  ros::ServiceClient sendControlwordClient_;
  ros::Publisher commandPublisher_;

  std::recursive_mutex readingMutex_;
  anydrive::ReadingExtended reading_;
  std::atomic<bool> readingReceived_;

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

 protected slots:

  /* ======================================================================== */
  /* Slots                                                                    */
  /* ======================================================================== */

  void setGoalState(const QString& stateCommand);

  void sendControlword();

  void sendCommand();

  void resetCommand();

  void sendCommandDisable();

  void sendCommandFreeze();
};

}  // namespace rqt_anydrive_monitor
