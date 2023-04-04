/*
 * BaseAuto.cpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// message logger
#include <message_logger/message_logger.hpp>

#include "free_gait_core/base_motion/BaseAuto.hpp"
#include "free_gait_core/leg_motion/EndEffectorMotionBase.hpp"
#include "free_gait_core/leg_motion/Footstep.hpp"

#include <math.h>

namespace free_gait {

BaseAuto::BaseAuto()
    : BaseMotionBase(BaseMotionBase::Type::Auto),
      ignoreTimingOfLegMotion_(false),
      averageLinearVelocity_(0.0),
      averageAngularVelocity_(0.0),
      duration_(0.0),
      supportMargin_(0.0),
      minimumDuration_(0.0),
      centerOfMassTolerance_(0.0),
      legLengthTolerance_(0.0),
      minLimbLengthScale_(0.0),
      maxLimbLengthAtClosingContactScale_(0.0),
      maxLimbLengthAtOpeningContactScale_(0.0),
      isComputed_(false),
      tolerateFailingOptimization_(false),
      controlSetup_ { {ControlLevel::Position, true}, {ControlLevel::Velocity, true},
                      {ControlLevel::Acceleration, false}, {ControlLevel::Effort, false} }
{
}

BaseAuto::~BaseAuto()
{
}

BaseAuto::BaseAuto(const BaseAuto& other) :
    BaseMotionBase(other),
    ignoreTimingOfLegMotion_(other.ignoreTimingOfLegMotion_),
    averageLinearVelocity_(other.averageLinearVelocity_),
    averageAngularVelocity_(other.averageAngularVelocity_),
    supportMargin_(other.supportMargin_),
    minimumDuration_(other.minimumDuration_),
    centerOfMassTolerance_(other.centerOfMassTolerance_),
    legLengthTolerance_(other.legLengthTolerance_),
    minLimbLengthScale_(other.minLimbLengthScale_),
    maxLimbLengthAtClosingContactScale_(other.maxLimbLengthAtClosingContactScale_),
    maxLimbLengthAtOpeningContactScale_(other.maxLimbLengthAtOpeningContactScale_),
    start_(other.start_),
    target_(other.target_),
    duration_(other.duration_),
    nominalPlanarStanceInBaseFrame_(other.nominalPlanarStanceInBaseFrame_),
    controlSetup_(other.controlSetup_),
    trajectory_(other.trajectory_),
    footholdsToReach_(other.footholdsToReach_),
    footholdsInSupport_(other.footholdsInSupport_),
    footholdsOfNextLegMotion_(other.footholdsOfNextLegMotion_),
    nominalStanceInBaseFrame_(other.nominalStanceInBaseFrame_),
    isComputed_(other.isComputed_),
    tolerateFailingOptimization_(other.tolerateFailingOptimization_)
{
  if (other.height_) height_.reset(new double(*(other.height_)));
}

std::unique_ptr<BaseMotionBase> BaseAuto::clone() const
{
  std::unique_ptr<BaseMotionBase> pointer(new BaseAuto(*this));
  return pointer;
}

const ControlSetup BaseAuto::getControlSetup() const
{
  return controlSetup_;
}

void BaseAuto::updateStartPose(const Pose& startPose)
{
  isComputed_ = false;
  start_ = startPose;
}

bool BaseAuto::prepareComputation(const State& state, const Step& step, const StepQueue& queue, const AdapterBase& adapter)
{
  // TODO This shouldn't be necessary if we could create copies of the adapter.
  adapter.createCopyOfState();

  if (!height_) {
    if (!computeHeight(state, queue, adapter)) {
      MELO_ERROR_STREAM("Could not compute height.")
      return false;
    }
  }
  if (!generateFootholdLists(state, step, queue, adapter)) {
    MELO_ERROR_STREAM("Could not generate foothold lists.")
    return false;
  }

  // Define support region.
  grid_map::Polygon supportRegion;
  std::vector<Position> footholdsOrdered;
  getFootholdsCounterClockwiseOrdered(footholdsInSupport_, footholdsOrdered);
  for (auto foothold : footholdsOrdered) {
    supportRegion.addVertex(foothold.vector().head<2>());
  }
  bool isLinePolygon = false;
  if (supportRegion.nVertices() == 2) {
    supportRegion.thickenLine(0.001);
    isLinePolygon = true;
  }
  if (!isLinePolygon) supportRegion.offsetInward(supportMargin_);

  // Define min./max. leg lengths.
  for (const auto& limb : adapter.getLimbs()) {
    // Computing the hip to foot distance when all the limb joints are at zero
    Position positionBaseToFootAtZeroInBaseFrame = adapter.getPositionBaseToFootInBaseFrame(limb, JointPositionsLeg());
    Position positionBaseToHipInBaseFrame = adapter.getPositionBaseToHipInBaseFrame(limb);
    const double distanceHipToFootAtZero = (positionBaseToFootAtZeroInBaseFrame - positionBaseToHipInBaseFrame).norm();
    // Compute the min limb length by multiplying hip to foot distance with a scaling factor
    minLimbLenghts_[limb] = minLimbLengthScale_ * distanceHipToFootAtZero;
    // Compute the max limb length by multiplying hip to foot distance with a scaling factor
    if (footholdsOfNextLegMotion_.find(limb) == footholdsOfNextLegMotion_.end()) {
      maxLimbLenghts_[limb] = maxLimbLengthAtClosingContactScale_ * distanceHipToFootAtZero; // Foot stays in contact.
    } else {
      maxLimbLenghts_[limb] = maxLimbLengthAtOpeningContactScale_ * distanceHipToFootAtZero; // Foot leaves contact.
    }
  }

  poseOptimizationGeometric_.reset(new PoseOptimizationGeometric(adapter));
  poseOptimizationGeometric_->setStance(footholdsToReach_);
  poseOptimizationGeometric_->setSupportStance(footholdsInSupport_);
  poseOptimizationGeometric_->setNominalStance(nominalStanceInBaseFrame_);
  poseOptimizationGeometric_->setSupportRegion(supportRegion);
  poseOptimizationGeometric_->setStanceForOrientation(footholdsForOrientation_);

  poseOptimizationQP_.reset(new PoseOptimizationQP(adapter));
  poseOptimizationQP_->setCurrentState(state);
  poseOptimizationQP_->setStance(footholdsToReach_);
  poseOptimizationQP_->setSupportStance(footholdsInSupport_);
  poseOptimizationQP_->setNominalStance(nominalStanceInBaseFrame_);
  poseOptimizationQP_->setSupportRegion(supportRegion);

  constraintsChecker_.reset(new PoseConstraintsChecker(adapter));
  constraintsChecker_->setCurrentState(state);
  constraintsChecker_->setStance(footholdsToReach_);
  constraintsChecker_->setSupportStance(footholdsInSupport_);
  constraintsChecker_->setSupportRegion(supportRegion);
  constraintsChecker_->setLimbLengthConstraints(minLimbLenghts_, maxLimbLenghts_);
  constraintsChecker_->setTolerances(centerOfMassTolerance_, legLengthTolerance_);

  poseOptimizationSQP_.reset(new PoseOptimizationSQP(adapter));
  poseOptimizationSQP_->setCurrentState(state);
  poseOptimizationSQP_->setStance(footholdsToReach_);
  poseOptimizationSQP_->setSupportStance(footholdsInSupport_);
  poseOptimizationSQP_->setNominalStance(nominalStanceInBaseFrame_);
  poseOptimizationSQP_->setSupportRegion(supportRegion);
  poseOptimizationSQP_->setLimbLengthConstraints(minLimbLenghts_, maxLimbLenghts_);

  if (!optimizePose(target_)) {
    MELO_ERROR_STREAM("Could not compute pose optimization."
                      << "\n Printing optimization problem:"
                      << "\nStance:\n"
                      << footholdsToReach_ << "\nSupport stance:\n"
                      << footholdsInSupport_ << "\nSupport margin: " << supportMargin_ << "\nNominal stance (in base frame):\n"
                      << nominalStanceInBaseFrame_ << "\nLimb length constraints:\n")
    for (const auto& limb : adapter.getLimbs()) {
      MELO_ERROR_STREAM("[" << limb << "] min: " << minLimbLenghts_[limb] << ", max: " << maxLimbLenghts_[limb])
    }
    if (!tolerateFailingOptimization_) return false;
  }

  computeDuration(state, step, adapter);
  computeTrajectory();

  // TODO This shouldn't be necessary if we could create copies of the adapter.
  adapter.resetToCopyOfState();

  return isComputed_ = true;
}

bool BaseAuto::needsComputation() const
{
  return false;
}

bool BaseAuto::isComputed() const
{
  return isComputed_;
}

void BaseAuto::reset()
{
  start_.setIdentity();
  target_.setIdentity();
  duration_ = 0.0;
  isComputed_ = false;
}

Pose BaseAuto::evaluatePose(const double time) const
{
  const double timeInRange = mapTimeWithinDuration(time);
  Pose pose;
  trajectory_.evaluate(pose, timeInRange);
  return pose;
}

Twist BaseAuto::evaluateTwist(const double time) const
{
  const double timeInRange = mapTimeWithinDuration(time);
  curves::CubicHermiteSE3Curve::DerivativeType derivative;
  trajectory_.evaluateDerivative(derivative, timeInRange, 1);
  Twist twist(derivative.getTranslationalVelocity().vector(),
              derivative.getRotationalVelocity().vector());
  return twist;
}

double BaseAuto::getDuration() const
{
  return duration_;
}

const std::string& BaseAuto::getFrameId(const ControlLevel& controlLevel) const
{
  return frameId_;
}

void BaseAuto::setNominalStanceInBaseFrame(const PlanarStance& nominalPlanarStanceInBaseFrame)
{
  nominalPlanarStanceInBaseFrame_ = nominalPlanarStanceInBaseFrame;
}

void BaseAuto::setHeight(const double height)
{
  height_.reset(new double(height));
}

double BaseAuto::getHeight() const
{
  if (height_) return *height_;
  throw std::runtime_error("Height of BaseAuto has not been set yet.");
}

void BaseAuto::setAverageLinearVelocity(const double averageLinearVelocity)
{
  averageLinearVelocity_ = averageLinearVelocity;
}

double BaseAuto::getAverageLinearVelocity() const
{
  return averageLinearVelocity_;
}

void BaseAuto::setAverageAngularVelocity(const double averageAngularVelocity)
{
  averageAngularVelocity_ = averageAngularVelocity;
}

double BaseAuto::getAverageAngularVelocity() const
{
  return averageAngularVelocity_;
}

double BaseAuto::getSupportMargin() const
{
  return supportMargin_;
}

void BaseAuto::setSupportMargin(double supportMargin)
{
  supportMargin_ = supportMargin;
}

void BaseAuto::setTolerateFailingOptimization(const bool tolerateFailingOptimization)
{
  tolerateFailingOptimization_ = tolerateFailingOptimization;
}

double BaseAuto::getCenterOfMassTolerance() const
{
  return centerOfMassTolerance_;
}

void BaseAuto::setCenterOfMassTolerance(const double tolerance)
{
  centerOfMassTolerance_ = tolerance;
}

double BaseAuto::getLegLengthTolerance() const
{
  return legLengthTolerance_;
}

void BaseAuto::setLegLengthTolerance(const double tolerance)
{
  legLengthTolerance_ = tolerance;
}

double BaseAuto::getMinLimbLengthScale() const
{
  return minLimbLengthScale_;
}

void BaseAuto::setMinLimbLengthScale(const double scale)
{
  minLimbLengthScale_ = scale;
}

double BaseAuto::getMaxLimbLengthAtClosingContactScale() const
{
  return maxLimbLengthAtClosingContactScale_;
}

void BaseAuto::setMaxLimbLengthAtClosingContactScale(const double scale)
{
  maxLimbLengthAtClosingContactScale_ = scale;
}

double BaseAuto::getMaxLimbLengthAtOpeningContactScale() const
{
  return maxLimbLengthAtOpeningContactScale_;
}

void BaseAuto::setMaxLimbLengthAtOpeningContactScale(const double scale)
{
  maxLimbLengthAtOpeningContactScale_ = scale;
}

bool BaseAuto::computeHeight(const State& state, const StepQueue& queue, const AdapterBase& adapter)
{
  if (queue.previousStepExists()) {
    if (queue.getPreviousStep().hasBaseMotion()) {
      if (queue.getPreviousStep().getBaseMotion().getType() == BaseMotionBase::Type::Auto) {
        const auto& previousBaseMotion = dynamic_cast<const BaseAuto&>(queue.getPreviousStep().getBaseMotion());
        height_.reset(new double(previousBaseMotion.getHeight()));
        return true;
      }
    }
  }

  unsigned n = 0;
  double heightSum = 0;
  for (const auto& limb : adapter.getLimbs()) {
    if (!state.isIgnoreForPoseAdaptation(limb)) {
      double legHeight = -adapter.getPositionBaseToFootInBaseFrame(limb, state.getJointPositionsForLimb(limb)).z();
      heightSum += legHeight;
      ++n;
    }
  }
  if (n == 0) return false;
  height_.reset(new double(heightSum / (double)(n)));
  return true;
}

bool BaseAuto::generateFootholdLists(const State& state, const Step& step, const StepQueue& queue, const AdapterBase& adapter)
{
  footholdsInSupport_.clear();
  footholdsOfNextLegMotion_.clear();
  bool prepareForNextStep = false;
  if (!step.hasLegMotion() && queue.size() > 1) {
    if (queue.getNextStep().hasLegMotion()) prepareForNextStep = true;
  }

  if (prepareForNextStep) {
    // Auto motion for preparation of next step.
    for (const auto& limb : adapter.getLimbs()) {
      if (!state.isIgnoreContact(limb) && !queue.getNextStep().hasLegMotion(limb)) {
        footholdsInSupport_[limb] = adapter.getPositionWorldToFootInWorldFrame(limb);
      } else {
        footholdsOfNextLegMotion_[limb] = adapter.getPositionWorldToFootInWorldFrame(limb);
      }
    }
  } else {
    // Auto motion for current step.
    for (const auto& limb : adapter.getLimbs()) {
      if (!step.hasLegMotion(limb) && !state.isIgnoreContact(limb)) {
        footholdsInSupport_[limb] = adapter.getPositionWorldToFootInWorldFrame(limb);
      }
    }
  }

  footholdsToReach_.clear();
  for (const auto& limb : adapter.getLimbs()) {
    if (step.hasLegMotion(limb)) {
      if (step.getLegMotion(limb).isIgnoreForPoseAdaptation()) continue;
      // Double check if right format.
      if (step.getLegMotion(limb).getTrajectoryType() == LegMotionBase::TrajectoryType::EndEffector
          && step.getLegMotion(limb).getControlSetup().at(ControlLevel::Position)) {
        // Use target end effector position.
        const auto& legMotion = dynamic_cast<const EndEffectorMotionBase&>(step.getLegMotion(limb));
        footholdsToReach_[limb] = adapter.transformPosition(legMotion.getFrameId(ControlLevel::Position),
                                                            adapter.getWorldFrameId(),
                                                            legMotion.getTargetPosition());
      }
      else {
        return false;
      }
    } else {
      // Use current foot position.
      if (!state.isIgnoreForPoseAdaptation(limb)) {
        footholdsToReach_[limb] = adapter.getPositionWorldToFootInWorldFrame(limb);
      }
    }
  }

  footholdsForOrientation_ = footholdsToReach_;
  for (const auto& limb : adapter.getLimbs()) {
    if (footholdsForOrientation_.count(limb) == 0) footholdsForOrientation_[limb] = adapter.getPositionWorldToFootInWorldFrame(limb);
  }

  nominalStanceInBaseFrame_.clear();
  for (const auto& stance : nominalPlanarStanceInBaseFrame_) {
    nominalStanceInBaseFrame_.emplace(stance.first, Position(stance.second(0), stance.second(1), -*height_));
  }

  return true;
}

bool BaseAuto::optimizePose(Pose& pose)
{
  if (!poseOptimizationGeometric_->optimize(pose)) {
    MELO_WARN("Geometric pose optimization returned false.")
  }

  if (!poseOptimizationQP_->optimize(pose)) {
    MELO_WARN("QP optimization returned false.")
    return false;
  }

  if (constraintsChecker_->check(pose)) return true;

  if (!poseOptimizationSQP_->optimize(pose)) {
    MELO_WARN("SQP optimization returned false.")
    return false;
  }

  return true;
}

void BaseAuto::computeDuration(const State& state, const Step& step, const AdapterBase& adapter)
{
  // Compute nominal speed.
  double distance = (target_.getPosition() - start_.getPosition()).norm();
  double translationDuration = distance / averageLinearVelocity_;
  double angle = fabs(target_.getRotation().getDisparityAngle(start_.getRotation()));
  double rotationDuration = angle / averageAngularVelocity_;
  duration_ = translationDuration > rotationDuration ? translationDuration : rotationDuration;
  duration_ = duration_ < minimumDuration_ ? minimumDuration_ : duration_;

  if (ignoreTimingOfLegMotion_ || !step.hasLegMotion()) return;
  double desiredDuration = duration_;

  // Adapting timing to reach goal with fastest leg motion.
  for (auto& legMotion : step.getLegMotions()) {
    if (legMotion.second->isIgnoreForPoseAdaptation()) continue;
    if (duration_ < legMotion.second->getDuration()) duration_ = legMotion.second->getDuration();
  }

  if (duration_ > desiredDuration) return;

  // Adapt timing of footsteps to base motion if base motion is faster than specified.
  // Other leg motions are ignored.
  duration_ = desiredDuration;
  for (auto& legMotion : step.getLegMotions()) {
    if (legMotion.second->isIgnoreForPoseAdaptation()) continue;
    if (legMotion.second->getType() == free_gait::LegMotionBase::Type::Footstep) {
      auto& footstep = dynamic_cast<free_gait::Footstep&>(*legMotion.second);
      if (footstep.getDuration() >= desiredDuration) continue;
      footstep.setMinimumDuration(desiredDuration);
      footstep.prepareComputation(state, step, adapter);
    }
  }
}

bool BaseAuto::computeTrajectory()
{
  std::vector<Time> times;
  std::vector<ValueType> values;

  times.push_back(0.0);
  values.push_back(start_);

  times.push_back(duration_);
  values.push_back(target_);

  trajectory_.fitCurve(times, values);
  return true;
}

std::ostream& operator<<(std::ostream& out, const BaseAuto& baseAuto)
{
  out << "Frame: " << baseAuto.frameId_ << std::endl;
  out << "Height: " << *(baseAuto.height_) << std::endl;
  out << "Ignore timing of leg motion: " << (baseAuto.ignoreTimingOfLegMotion_ ? "True" : "False") << std::endl;
  out << "Average Linear Velocity: " << baseAuto.averageLinearVelocity_ << std::endl;
  out << "Average Angular Velocity: " << baseAuto.averageAngularVelocity_ << std::endl;
  out << "Support Margin: " << baseAuto.supportMargin_ << std::endl;
  out << "Duration: " << baseAuto.duration_ << std::endl;
  out << "CoM tolerance: " << baseAuto.centerOfMassTolerance_ << std::endl;
  out << "Leg length tolerance: " << baseAuto.legLengthTolerance_ << std::endl;
  out << "Minimum limb length scaling factor: " << baseAuto.minLimbLengthScale_ << std::endl;
  out << "Maximum limb length at closing contact scaling factor: " << baseAuto.maxLimbLengthAtClosingContactScale_ << std::endl;
  out << "Maximum limb length at opening contact scaling factor: " << baseAuto.maxLimbLengthAtOpeningContactScale_ << std::endl;
  out << "Start Position: " << baseAuto.start_.getPosition() << std::endl;
  out << "Start Orientation: " << baseAuto.start_.getRotation() << std::endl;
  out << "Start Orientation (yaw, pitch, roll) [deg]: " << 180.0 / M_PI * EulerAnglesZyx(baseAuto.start_.getRotation()).getUnique().vector().transpose() << std::endl;
  out << "Target Position: " << baseAuto.target_.getPosition() << std::endl;
  out << "Target Orientation: " << baseAuto.target_.getRotation() << std::endl;
  out << "Target Orientation (yaw, pitch, roll) [deg]: " << 180.0 / M_PI * EulerAnglesZyx(baseAuto.target_.getRotation()).getUnique().vector().transpose() << std::endl;
  out << "Footholds in support: ";
  for (const auto& f : baseAuto.footholdsInSupport_) out << f.first << " ";
  out << std::endl;
  out << "Footholds to reach: ";
  for (const auto& f : baseAuto.footholdsToReach_) out << f.first << " ";
  out << std::endl;
  out << "Footholds of next leg motion: ";
  for (const auto& f : baseAuto.footholdsOfNextLegMotion_) out << f.first << " ";
  out << std::endl;
  return out;
}

} /* namespace */
