/*
 * SteeringModeModule.hpp
 *
 *  Created on: Aug 20, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once

// parameter handler
#include "parameter_handler/parameter_handler.hpp"

// loco
#include "loco/common/WholeBody.hpp"
#include "loco/common/typedefs.hpp"

// STL
#include <limits>
#include <memory>

class TiXmlHandle;

namespace loco {

class SteeringModeModule {
 public:
  //! Default Constructor
  explicit SteeringModeModule(std::string name);

  //! Default destructor
  virtual ~SteeringModeModule() = default;

  virtual const std::string& getModuleName() const { return name_; }

  virtual bool initialize(double dt, loco::WholeBody& wholeBody) = 0;

  virtual bool advance(double dt, loco::WholeBody& wholeBody) = 0;

  //! @param  steering angle
  void setSteeringAngle(const double steeringAngle) { steeringAngle_ = steeringAngle; }

  //! @return steering angle
  double getSteeringAngle() const { return steeringAngle_; }

  //! @param minimal steering angle
  void setMinSteeringAngle(const double minSteeringAngle) { minSteeringAngle_ = minSteeringAngle; }

  //! @return minimal steering angle
  double getMinSteeringAngle() const { return minSteeringAngle_; }

  //! @param maximal steering angle
  void setMaxSteeringAngle(const double maxSteeringAngle) { maxSteeringAngle_ = maxSteeringAngle; }

  //! @return maximal steering angle
  double getMaxSteeringAngle() const { return maxSteeringAngle_; }

  //! @return center of rotation (default: infinity)
  const Position& getPositionBaseProjectionToCoRInControlFrame() { return positionBaseProjectionToCoRInControlFrame_; }

  virtual bool loadModuleParameters(const TiXmlHandle& handle);
  virtual bool addModuleParametersToHandler(const std::string& ns);
  virtual bool addModuleVariablesToLog(const std::string& ns) const;

 protected:
  const std::string name_;

  //! Center of rotation
  Position positionBaseProjectionToCoRInControlFrame_;

  //! Steering angle
  const double defaultSteeringAngleLimit_ = 80.0 * M_PI / 180.0;
  double steeringAngle_;
  double minSteeringAngle_;
  double maxSteeringAngle_;

  //! Parameters
  parameter_handler::Parameter<double> steeringIncrement_;
  parameter_handler::Parameter<double> velocityFactor_;
};

using SteeringModeModulePtr = std::unique_ptr<SteeringModeModule>;

} /* namespace loco */
