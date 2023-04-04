/*!
 * @file    AnymalStateEstimatorBasic.hpp
 * @author  Markus Staeuble
 * @date    May, 2018
 */

#pragma once

#include <anymal_state_estimator/AnymalStateEstimator.hpp>
#include <anymal_description/AnymalDescription.hpp>
#include <anymal_model/AnymalState.hpp>
#include <anymal_model/AnymalModel.hpp>
#include <anymal_model/AnymalFramesGenerator.hpp>
#include <anymal_model_ros/AnymalContainersRos.hpp>

#include <probabilistic_contact_estimation/ProbabilisticContactEstimation.hpp>

#include <tinyxml_tools/tinyxml_tools.hpp>

namespace anymal_state_estimator {

template <typename Filter_>
using AnymalStateEstimatorBase = AnymalStateEstimator<
  anymal_description::ConcreteAnymalDescription,
  anymal_model::AnymalState,
  anymal_model_ros::AnymalContainersRos,
  Filter_>;

/**
 * @brief      The StateEstimator implementation for an anymal without any
 *             custom attachments
 *
 * @tparam     Filter_  The filter type. Needs to derive from AnymalFilter
 */
template <typename Filter_>
class AnymalStateEstimatorBasic : public AnymalStateEstimatorBase<Filter_> {
public:
  using NodeBase = any_node::Node;
  using EstimatorBase = AnymalStateEstimatorBase<Filter_>;
  using typename EstimatorBase::ActuatorEnum;
  using typename EstimatorBase::ContactStateEnumDetector;
  using typename EstimatorBase::ContactEnum;
  using typename EstimatorBase::BranchEnum;
  using AnymalModel = anymal_model::AnymalModel;
  using AnymalFramesGenerator = anymal_model::AnymalFramesGenerator<anymal_description::ConcreteAnymalDescription,
                                                                             anymal_model::AnymalState>;
  using AnymalState = anymal_model::AnymalState;
  using AD = anymal_description::AnymalDescription;

  template <typename ValueType_>
  using ContactEnumContainer = std_utils::EnumArray<ContactEnum, ValueType_>;

  AnymalStateEstimatorBasic() = delete;
  explicit AnymalStateEstimatorBasic(any_node::Node::NodeHandlePtr nh);

  ~AnymalStateEstimatorBasic() override = default;

protected:
  void initModel() final;
  void resetModelState(const kindr::HomTransformQuatD& pose) final;
  void initializeRobotStateMsgs() final;
  void fillJointStates() final;
  void updateEstimatedState() final;

  void readParametersImpl() final;
  void addVariablesToLogImpl() final;
  void resetModulesImpl() final;

  void initializeContactDetectors() final;
  void advanceContactDetector(ContactEnumContainer<ContactStateEnumDetector>& contactStates) final;

  std::unique_ptr<contact_estimation::ProbabilisticContactEstimation> probabilisticContactDetectorPtr_;

  //set true to disable base contact detectors and override with probabilistic contact estimator (PCE)
  bool useProbabilisticContactEstimation_{false};

  //if true, will use contact detector that thresholds contact forces, but logs state of the probabilistic contact detector.
  bool compareWithThresholdingContactDetector_{false};

  //if true, logs signals from probabilistic contact detector.
  bool enableLogging_{false};

  //! If true, contact detection uses height information from depth camera.
  //! Notice that if enabled, the contact estimator will silently subscribe to the elevation map.
  bool usePerceptionForProbabilisticContactDetection_{false};

  //path to xml parameter doc to read probabilistic contact estimation params from
  std::string pceParameterFile_{""};

private:
  //helper to determine whether the model state is valid for PCE.
  //if not, the PCE will switch to a safe mode which only uses the dynamics measurement model based on measured joint states
  bool isModelStateValidForPce() const;
  //helper to initialize the PCE
  void initializePce();

};

} // namespace anymal_state_estimator

// gets compiled in anymal_state_estimator_lwf and anymal_state_estimator_tsif
// #include <anymal_state_estimator/anymal_state_estimator_basic/AnymalStateEstimatorBasic.tpp>
