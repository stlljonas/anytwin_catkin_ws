#pragma once

#include <atomic>
#include <memory>

#include <sensor_msgs/JointState.h>

#include <anydrive/AnydriveManager.hpp>

#include <anydrive_msgs/AchieveJointPositionConfiguration.h>
#include <anydrive_msgs/Calibrate.h>
#include <anydrive_msgs/Commands.h>
#include <anydrive_msgs/EraseFlashStorage.h>
#include <anydrive_msgs/GetCalibrationState.h>
#include <anydrive_msgs/GetControlGains.h>
#include <anydrive_msgs/GetDirection.h>
#include <anydrive_msgs/GetDriveInfo.h>
#include <anydrive_msgs/GetErrorStateBehavior.h>
#include <anydrive_msgs/GetJointPositionLimits.h>
#include <anydrive_msgs/GetMaxCurrent.h>
#include <anydrive_msgs/GetMaxMotorVelocity.h>
#include <anydrive_msgs/JointPositionConfigurations.h>
#include <anydrive_msgs/Readings.h>
#include <anydrive_msgs/ReadingsExtended.h>
#include <anydrive_msgs/ResetCustomCalibrationsToFactory.h>
#include <anydrive_msgs/ResetFlashStorageSections.h>
#include <anydrive_msgs/SendControlword.h>
#include <anydrive_msgs/SendSdoRead.h>
#include <anydrive_msgs/SendSdoWrite.h>
#include <anydrive_msgs/SetControlGains.h>
#include <anydrive_msgs/SetDirection.h>
#include <anydrive_msgs/SetDriveInfoString.h>
#include <anydrive_msgs/SetDriveInfoUint16.h>
#include <anydrive_msgs/SetErrorStateBehavior.h>
#include <anydrive_msgs/SetFsmGoalState.h>
#include <anydrive_msgs/SetJointPositionLimits.h>
#include <anydrive_msgs/SetMaxCurrent.h>
#include <anydrive_msgs/SetMaxMotorVelocity.h>
#include <anydrive_msgs/WriteConfiguration.h>
#include <anydrive_msgs/WriteFactoryCalibration.h>

#include "anydrive_ros/AnydriveRos.hpp"

namespace anydrive_ros {

class AnydriveManagerRos : public anydrive::AnydriveManager {
 protected:
  ros::NodeHandle& nh_;

  std::string rosPrefix_;

  bool runCommandsSubscriber_ = true;

  bool runReadingsPublisher_ = true;

  bool runReadingsExtendedThrottledPublisher_ = true;
  unsigned int readingsExtendedThrottledPublisherCounter_ = 0;
  unsigned int readingsExtendedThrottledPublisherDecimation_ = 0;

  bool runJointStatesPublisher_ = true;

  bool runJointStatesThrottledPublisher_ = true;
  unsigned int jointStatesThrottledPublisherCounter_ = 0;
  unsigned int jointStatesThrottledPublisherDecimation_ = 0;

  ros::Subscriber commandsSubscriber_;

  any_node::ThreadedPublisherPtr<anydrive_msgs::Readings> readingsPublisher_;
  any_node::ThreadedPublisherPtr<anydrive_msgs::ReadingsExtended> readingsExtendedThrottledPublisher_;
  any_node::ThreadedPublisherPtr<sensor_msgs::JointState> jointStatesPublisher_;
  any_node::ThreadedPublisherPtr<sensor_msgs::JointState> jointStatesThrottledPublisher_;

  ros::Publisher availableJointPositionConfigurationsPublisher_;

  // Send SDOs.
  ros::ServiceServer sendSdoReadServer_;
  ros::ServiceServer sendSdoWriteServer_;

  // Drive info.
  ros::ServiceServer getDriveInfoServer_;
  ros::ServiceServer setDriveInfoSerialNumberServer_;
  ros::ServiceServer setDriveInfoNameServer_;
  ros::ServiceServer setDriveInfoIdServer_;
  ros::ServiceServer setDriveInfoBootloaderVersionServer_;

  // Flash storage.
  ros::ServiceServer eraseFlashStorage_;
  ros::ServiceServer resetFlashStorageSections_;

  // Calibration.
  ros::ServiceServer getCalibrationStateServer_;
  ros::ServiceServer calibrateServer_;
  ros::ServiceServer resetCustomCalibrationsToFactoryServer_;
  ros::ServiceServer writeFactoryCalibrationServer_;

  // Configuration.
  ros::ServiceServer getMaxCurrentServer_;
  ros::ServiceServer setMaxCurrentServer_;
  ros::ServiceServer getMaxMotorVelocityServer_;
  ros::ServiceServer setMaxMotorVelocityServer_;
  ros::ServiceServer getJointPositionLimitsSdkServer_;
  ros::ServiceServer setJointPositionLimitsSdkServer_;
  ros::ServiceServer getJointPositionLimitsSoftServer_;
  ros::ServiceServer setJointPositionLimitsSoftServer_;
  ros::ServiceServer getJointPositionLimitsHardServer_;
  ros::ServiceServer setJointPositionLimitsHardServer_;
  ros::ServiceServer getControlGainsServer_;
  ros::ServiceServer setControlGainsServer_;
  ros::ServiceServer getErrorStateBehaviorServer_;
  ros::ServiceServer setErrorStateBehaviorServer_;
  ros::ServiceServer writeConfigurationServer_;

  // Control.
  ros::ServiceServer setGoalStateServer_;
  ros::ServiceServer sendControlwordServer_;
  ros::ServiceServer achieveJointPositionConfigurationServer_;

  std::recursive_mutex readingsMsgMutex_;
  std::atomic<bool> readingsMsgUpdated_;
  anydrive_msgs::Readings readingsMsg_;
  std::recursive_mutex readingsExtendedMsgMutex_;
  std::atomic<bool> readingsExtendedMsgUpdated_;
  anydrive_msgs::ReadingsExtended readingsExtendedMsg_;
  std::recursive_mutex jointStatesMsgMutex_;
  std::atomic<bool> jointStatesMsgUpdated_;
  sensor_msgs::JointState jointStatesMsg_;

  //! Publish options.
  const unsigned int maxPublishMessageBufferSize_ = 10;
  bool createPublishWorker_ = true;

  //! Publish worker.
  std::shared_ptr<any_worker::Worker> publishWorker_;
  std::mutex notifyPublishWorkerMutex_;
  std::atomic<uint64_t> publishCounter_;
  std::condition_variable notifyPublishWorkerCv_;
  std::atomic<bool> shutdownPublishWorkerRequested_;

 public:
  AnydriveManagerRos(const bool standalone, const bool installSignalHandler, const double timeStep, ros::NodeHandle& nh,
                     std::string rosPrefix);
  ~AnydriveManagerRos() override = default;

  anydrive_msgs::Readings getReadingsMsg();
  anydrive_msgs::ReadingsExtended getReadingsExtendedMsg();
  sensor_msgs::JointState getJointStatesMsg();

  bool startup() override;
  bool startup(unsigned int startupRetries) override;
  void updateProcessReadings() override;
  void shutdown() override;

  void sendRos();

  void startupRosInterface();

  //! Can throw an anydrive::Exception.
  void shutdownRosInterface();

 protected:
  anydrive::AnydrivePtr createAnydrive() const override;

  bool publishWorkerCb(const any_worker::WorkerEvent& event);

  void commandsCb(const anydrive_msgs::CommandsConstPtr& commandsMsg);

  // Send SDOs.
  virtual bool sendSdoReadCb(anydrive_msgs::SendSdoReadRequest& req, anydrive_msgs::SendSdoReadResponse& res);
  virtual bool sendSdoWriteCb(anydrive_msgs::SendSdoWriteRequest& req, anydrive_msgs::SendSdoWriteResponse& res);

  // Drive info.
  bool getDriveInfoCb(anydrive_msgs::GetDriveInfoRequest& req, anydrive_msgs::GetDriveInfoResponse& res);
  bool setDriveInfoSerialNumber(anydrive_msgs::SetDriveInfoStringRequest& req, anydrive_msgs::SetDriveInfoStringResponse& res);
  bool setDriveInfoName(anydrive_msgs::SetDriveInfoStringRequest& req, anydrive_msgs::SetDriveInfoStringResponse& res);
  bool setDriveInfoId(anydrive_msgs::SetDriveInfoUint16Request& req, anydrive_msgs::SetDriveInfoUint16Response& res);
  bool setDriveInfoBootloaderVersion(anydrive_msgs::SetDriveInfoStringRequest& req, anydrive_msgs::SetDriveInfoStringResponse& res);

  // Flash storage.
  bool eraseFlashStorageCb(anydrive_msgs::EraseFlashStorageRequest& req, anydrive_msgs::EraseFlashStorageResponse& res);
  bool resetFlashStorageSectionsCb(anydrive_msgs::ResetFlashStorageSectionsRequest& req,
                                   anydrive_msgs::ResetFlashStorageSectionsResponse& res);

  // Calibration.
  bool getCalibrationStateCb(anydrive_msgs::GetCalibrationStateRequest& req, anydrive_msgs::GetCalibrationStateResponse& res);
  bool calibrateCb(anydrive_msgs::CalibrateRequest& req, anydrive_msgs::CalibrateResponse& res);
  bool resetCustomCalibrationsToFactoryCb(anydrive_msgs::ResetCustomCalibrationsToFactoryRequest& req,
                                          anydrive_msgs::ResetCustomCalibrationsToFactoryResponse& res);
  bool writeFactoryCalibrationCb(anydrive_msgs::WriteFactoryCalibrationRequest& req, anydrive_msgs::WriteFactoryCalibrationResponse& res);

  // Configuration.
  bool getMaxCurrentCb(anydrive_msgs::GetMaxCurrentRequest& req, anydrive_msgs::GetMaxCurrentResponse& res);
  bool setMaxCurrentCb(anydrive_msgs::SetMaxCurrentRequest& req, anydrive_msgs::SetMaxCurrentResponse& res);
  bool getMaxMotorVelocityCb(anydrive_msgs::GetMaxMotorVelocityRequest& req, anydrive_msgs::GetMaxMotorVelocityResponse& res);
  bool setMaxMotorVelocityCb(anydrive_msgs::SetMaxMotorVelocityRequest& req, anydrive_msgs::SetMaxMotorVelocityResponse& res);
  bool getDirectionCb(anydrive_msgs::GetDirectionRequest& req, anydrive_msgs::GetDirectionResponse& res);
  bool setDirectionCb(anydrive_msgs::SetDirectionRequest& req, anydrive_msgs::SetDirectionResponse& res);
  bool getJointPositionLimitsSdkCb(anydrive_msgs::GetJointPositionLimitsRequest& req, anydrive_msgs::GetJointPositionLimitsResponse& res);
  bool setJointPositionLimitsSdkCb(anydrive_msgs::SetJointPositionLimitsRequest& req, anydrive_msgs::SetJointPositionLimitsResponse& res);
  bool getJointPositionLimitsSoftCb(anydrive_msgs::GetJointPositionLimitsRequest& req, anydrive_msgs::GetJointPositionLimitsResponse& res);
  bool setJointPositionLimitsSoftCb(anydrive_msgs::SetJointPositionLimitsRequest& req, anydrive_msgs::SetJointPositionLimitsResponse& res);
  bool getJointPositionLimitsHardCb(anydrive_msgs::GetJointPositionLimitsRequest& req, anydrive_msgs::GetJointPositionLimitsResponse& res);
  bool setJointPositionLimitsHardCb(anydrive_msgs::SetJointPositionLimitsRequest& req, anydrive_msgs::SetJointPositionLimitsResponse& res);
  bool getControlGainsCb(anydrive_msgs::GetControlGainsRequest& req, anydrive_msgs::GetControlGainsResponse& res);
  bool setControlGainsCb(anydrive_msgs::SetControlGainsRequest& req, anydrive_msgs::SetControlGainsResponse& res);
  bool getErrorStateBehaviorCb(anydrive_msgs::GetErrorStateBehaviorRequest& req, anydrive_msgs::GetErrorStateBehaviorResponse& res);
  bool setErrorStateBehaviorCb(anydrive_msgs::SetErrorStateBehaviorRequest& req, anydrive_msgs::SetErrorStateBehaviorResponse& res);
  bool writeConfigurationCb(anydrive_msgs::WriteConfigurationRequest& req, anydrive_msgs::WriteConfigurationResponse& res);

  // Control.
  bool setGoalStateCb(anydrive_msgs::SetFsmGoalStateRequest& req, anydrive_msgs::SetFsmGoalStateResponse& res);
  bool sendControlwordCb(anydrive_msgs::SendControlwordRequest& req, anydrive_msgs::SendControlwordResponse& res);
  bool achieveJointPositionConfigurationCb(anydrive_msgs::AchieveJointPositionConfigurationRequest& req,
                                           anydrive_msgs::AchieveJointPositionConfigurationResponse& res);
};

}  // namespace anydrive_ros
