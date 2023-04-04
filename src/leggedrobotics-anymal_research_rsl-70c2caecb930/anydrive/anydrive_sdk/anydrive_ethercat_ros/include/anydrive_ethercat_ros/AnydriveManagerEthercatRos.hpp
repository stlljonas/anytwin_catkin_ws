#pragma once

#include <anydrive_ros/AnydriveManagerRos.hpp>

#include <anydrive_ethercat/AnydriveEthercatBusManager.hpp>

namespace anydrive_ethercat_ros {

class AnydriveManagerEthercatRos : public anydrive_ros::AnydriveManagerRos {
 public:
  /*!
   * Constructor.
   * @param standalone           If true, this class will start its own worker.
   * @param installSignalHandler If true, this class will install its own signal handler.
   * @param timeStep             Time step for standalone mode.
   */
  AnydriveManagerEthercatRos(const bool standalone, const bool installSignalHandler, const double timeStep, ros::NodeHandle& nh,
                             const std::string& rosPrefix);

  /*!
   * Destructor.
   */
  ~AnydriveManagerEthercatRos() override = default;

  /*!
   * Create a new default setup.
   * @return New default setup.
   */
  anydrive::setup::SetupPtr createSetup() const override;

  // Send SDOs.
  bool sendSdoReadCb(anydrive_msgs::SendSdoReadRequest& req, anydrive_msgs::SendSdoReadResponse& res) override;
  bool sendSdoWriteCb(anydrive_msgs::SendSdoWriteRequest& req, anydrive_msgs::SendSdoWriteResponse& res) override;
};

using AnydriveManagerEthercatRosPtr = std::shared_ptr<AnydriveManagerEthercatRos>;

}  // namespace anydrive_ethercat_ros
