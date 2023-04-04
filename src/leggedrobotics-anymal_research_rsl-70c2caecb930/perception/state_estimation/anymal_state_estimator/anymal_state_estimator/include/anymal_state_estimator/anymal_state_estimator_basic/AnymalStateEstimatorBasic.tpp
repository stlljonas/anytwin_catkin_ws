/*!
 * @file    AnymalStateEstimatorBasic.cpp
 * @author  Markus Staeuble
 * @date    May, 2018
 */

#include <anymal_state_estimator/anymal_state_estimator_basic/AnymalStateEstimatorBasic.hpp>

#include <anymal_model_ros/initializations.hpp>

#include <anymal_model/StateStatus.hpp>

#include <probabilistic_contact_estimation/StateElevationMap.hpp>

namespace anymal_state_estimator {

template <typename Filter_>
AnymalStateEstimatorBasic<Filter_>::AnymalStateEstimatorBasic(NodeBase::NodeHandlePtr nh) :
EstimatorBase(nh)
{
}

template <typename Filter_>
void AnymalStateEstimatorBasic<Filter_>::advanceContactDetector(
    ContactEnumContainer<ContactStateEnumDetector>& contactStates){
  if(useProbabilisticContactEstimation_  || compareWithThresholdingContactDetector_){
    probabilisticContactDetectorPtr_->setIsRobotStateValid(isModelStateValidForPce());
    if(!probabilisticContactDetectorPtr_->advance(this->timeStep_)) {
      MELO_WARN_STREAM("Failed to advance probabilistic contact estimator.");
    }
    for (const auto contactKey : AD::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();
      contactStates[contactEnum] = probabilisticContactDetectorPtr_->getContactState(contactEnum);
    }

    if (compareWithThresholdingContactDetector_) {
      EstimatorBase::advanceContactDetector(contactStates);
    }
  }
  else{
    EstimatorBase::advanceContactDetector(contactStates);
  }
}

template <typename Filter_>
void AnymalStateEstimatorBasic<Filter_>::readParametersImpl() {
  useProbabilisticContactEstimation_ = NodeBase::param<bool>("probabilistic_contact_estimation/enable", false);
  compareWithThresholdingContactDetector_ = NodeBase::param<bool>("probabilistic_contact_estimation/compare_with_thresholding_contact_detector", false);
  enableLogging_ = NodeBase::param<bool>("probabilistic_contact_estimation/logging", false);
  usePerceptionForProbabilisticContactDetection_ = NodeBase::param<bool>("probabilistic_contact_estimation/use_perception", false);
  pceParameterFile_ = std::string{NodeBase::param<std::string>("parameter_path", "") + "/probabilistic_contact_estimation" + ".xml"};
}

template <typename Filter_>
void AnymalStateEstimatorBasic<Filter_>::initializeContactDetectors() {
  if(useProbabilisticContactEstimation_ || compareWithThresholdingContactDetector_){
    MELO_INFO_STREAM("Using probabilistic contact estimation.");
    auto* anymalModel = static_cast<AnymalModel*>(this->robotModelPtr_.get());

    if (usePerceptionForProbabilisticContactDetection_) {
      probabilisticContactDetectorPtr_.reset(new contact_estimation::ProbabilisticContactEstimation(new contact_estimation::StateElevationMap(anymalModel, this->getNodeHandle()), anymalModel));
    } else {
      probabilisticContactDetectorPtr_.reset(new contact_estimation::ProbabilisticContactEstimation(new contact_estimation::State(anymalModel), anymalModel));
    }

    initializePce();
    probabilisticContactDetectorPtr_->setEnableLogging(enableLogging_);

    if (compareWithThresholdingContactDetector_) {
      EstimatorBase::initializeContactDetectors();
    }
  }
  else{
    EstimatorBase::initializeContactDetectors();
  }
}

template <typename Filter_>
void AnymalStateEstimatorBasic<Filter_>::addVariablesToLogImpl() {
  if(useProbabilisticContactEstimation_) { probabilisticContactDetectorPtr_->addVariablesToLog(); }
}

template <typename Filter_>
void AnymalStateEstimatorBasic<Filter_>::resetModulesImpl() {
  if (useProbabilisticContactEstimation_) { initializePce(); }
}

template <typename Filter_>
void AnymalStateEstimatorBasic<Filter_>::resetModelState(const kindr::HomTransformQuatD& pose) {
  auto state = this->robotModelPtr_->getState();
  // Set pose
  state.setPoseBaseToWorld(pose);
  for (auto actuatorKey : AD::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const auto actuatorId = actuatorKey.getId();
    state.getJointPositions()(actuatorId) = this->actuatorReadings_[actuatorEnum].getState().getJointPosition();
  }
  // set state
  this->robotModelPtr_->setState(state, true, false, false);
}

template <typename Filter_>
void AnymalStateEstimatorBasic<Filter_>::initializeRobotStateMsgs() {
  anymal_model_ros::initialize(this->measJointStatesRos_);
  anymal_model_ros::initialize(this->rosEstRobotState_);
}

template <typename Filter_>
void AnymalStateEstimatorBasic<Filter_>::initModel() {

  this->robotModelPtr_.reset(new AnymalModel());
  std::string robotUrdfDescription = NodeBase::param<std::string>("/anymal_description", "");

  if (!std::static_pointer_cast<AnymalModel>(this->robotModelPtr_)->initializeFromUrdf(robotUrdfDescription)) {
    MELO_FATAL_STREAM("Could not load model from URDF! " << robotUrdfDescription);
    return;
  }

  this->framesGeneratorPtr_.reset(new AnymalFramesGenerator());
}

template <typename Filter_>
void AnymalStateEstimatorBasic<Filter_>::fillJointStates() {
  for (auto jointKey : AD::getJointKeys()) {
    const auto jointEnum = jointKey.getEnum();
    const auto jointId = jointKey.getId();
    const auto& reading = this->actuatorReadings_[AD::template mapEnums<ActuatorEnum>(jointEnum)];
    this->measJointStates_[jointEnum].time_ = reading.getState().getStamp();
    this->measJointStates_[jointEnum].position_ = reading.getState().getJointPosition();
    this->measJointStates_[jointEnum].velocity_ = reading.getState().getJointVelocity();
    this->measJointStates_[jointEnum].acceleration_ = reading.getState().getJointAcceleration();
    this->measJointStates_[jointEnum].effort_ = reading.getState().getJointTorque();
  }
}

template <typename Filter_>
void AnymalStateEstimatorBasic<Filter_>::updateEstimatedState() {
  this->estimatedState_.anymalState_ = this->filterState_;

  std::lock_guard<std::mutex> lock(this->mutexFilter_);

  this->estimatedState_.time_ = this->filter_.getLastEstimatedStateStamp();
  this->estimatedState_.status_ = this->estimatorStatus_;

  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();

    this->estimatedState_.contacts_[contactEnum].state_ = static_cast<unsigned int>(this->contacts_[contactEnum].state_);
    this->estimatedState_.contacts_[contactEnum].time_ = this->contacts_[contactEnum].stamp_;
    this->estimatedState_.contacts_[contactEnum].position_ = this->contacts_[contactEnum].contactPointInOdom_;
    this->estimatedState_.contacts_[contactEnum].normal_ = kindr::VectorTypeless3D(0.0, 0.0, 1.0);
    this->estimatedState_.contacts_[contactEnum].wrench_ = this->contactWrenchPublishers_[contactEnum]->getWrench();
    std::lock_guard<std::mutex> lockCalibrators(this->mutexForceCalibrators_);
    this->forceCalibrators_[contactEnum]->getStatistics(this->estimatedState_.forceCalibratorStats_[contactEnum]);
  }
}

template <typename Filter_>
bool AnymalStateEstimatorBasic<Filter_>::isModelStateValidForPce() const {
  return ((this->estimatorStatus_ == anymal_model::StateStatus::STATUS_OK) && !this->zeroVelocityUpdatesEnabled_);
}

template <typename Filter_>
void AnymalStateEstimatorBasic<Filter_>::initializePce() {
  tinyxml_tools::DocumentHandleXML pceParameterDoc;
  if (!pceParameterDoc.create(pceParameterFile_, tinyxml_tools::DocumentMode::READ)) {
    MELO_FATAL_STREAM("Could not load probabilistic contact estimation parameters document from " << pceParameterFile_);
  }
  probabilisticContactDetectorPtr_->setIsRobotStateValid(isModelStateValidForPce());
  if(!probabilisticContactDetectorPtr_->loadParameters(pceParameterDoc.getDocumentHandle())) {
    MELO_FATAL_STREAM("Failed to load parameters for probabilistic contact estimator.");
  }
  if(!probabilisticContactDetectorPtr_->initialize(this->timeStep_)) {
    MELO_ERROR_STREAM("Failed to initialize probabilistic contact estimator.");
  }
}

}
