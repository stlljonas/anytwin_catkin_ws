/*!
 * @file     LocomotionControllerModules.hpp
 * @author   Gabriel Hottiger
 * @date     Jan, 2016
 */

#pragma once

// loco
#include "loco/common/ModuleBase.hpp"
#include "loco/common/ParameterSet.hpp"
#include "loco/locomotion_controller/LocomotionControllerBase.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// STL
#include <map>

namespace loco {

class LocomotionControllerModules : public LocomotionControllerBase {
 public:
  /*! Constucts a LocomotionController
   */
  LocomotionControllerModules();

  //! Default destructor
  ~LocomotionControllerModules() override = default;

  /*!
   * Initializes locomotion controller
   * @param dt the time step [s]
   * @return true if successful.
   */
  bool initialize(double dt) override;

  /** Loads parameters from an xml file
   *  @param handle tinyxml handle
   *  @return true, iff successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /*! Advance measurements in time
   * @param dt  time step [s]
   * @return true if successful.
   */
  bool advanceMeasurements(double dt) override;

  /*! Advance setpoints in time
   * @param dt  time step [s]
   * @return true if successful.
   */
  bool advanceSetPoints(double dt) override;

  /** Adds variables to the signal logger
   * @param ns            namespace of the variables
   * @return              true, iff successful
   */
  bool addVariablesToLog(const std::string& ns) const override;

  /** Adds parameters to the parameter handler
   * @param  ns, namespace  of the parameters
   * @return true, iff successful
   */
  bool addParametersToHandler(const std::string& ns) override;

  /** Removes parameters from the parameter handler
   * @return true, iff successful
   */
  bool removeParametersFromHandler() override;

  /** Virtual print helper method for polymorph << ostream operator overload
   *  @param out ostream of the << operation
   */
  void print(std::ostream& out) const override;

  /** Function that is called after emergency stop. Use this function to properly shut down
   * threads, to enforce a certain state or to prepare a following initialization step.
   */
  bool stop() override;

  /*! Add measurement module at index
   *  @param index  index in module map -> defines execution order
   *  @param module module to be added
   */
  void addMeasurementModule(const unsigned int index, loco::ModuleBase* module);

  /*! Add measurement module at end -> executed after all previously added modules
   *  @param module module to be added
   */
  void pushBackMeasurementModule(loco::ModuleBase* module);

  /*! Add measurement modules at end -> executed after all previously added modules
   *  @param module module to be added
   */
  void pushBackMeasurementModules(const std::vector<loco::ModuleBase*>& modules);

  /*! Remove measurement module at index
   *  @param index  map index of module to be removed
   */
  void removeMeasurementModule(const unsigned int index);

  /*! Add setpoint module at index
   *  @param index  index in module map -> defines execution order
   *  @param module module to be added
   */
  void addSetPointModule(const unsigned int index, loco::ModuleBase* module);

  /*! Add setpoint module at end -> executed after all previously added modules
   *  @param module module to be added
   */
  void pushBackSetPointModule(loco::ModuleBase* module);

  /*! Add setpoint modules at end -> executed after all previously added modules
   *  @param module module to be added
   */
  void pushBackSetPointModules(const std::vector<loco::ModuleBase*>& modules);

  /*! Remove setpoint module at index
   *  @param index  map index of module to be removed
   */
  void removeSetPointModule(const unsigned int index);

 protected:
  //! Map of measurement modules
  std::map<unsigned int, loco::ModuleBase*> measurementModules_;
  //! Map of setpoint modules
  std::map<unsigned int, loco::ModuleBase*> setpointModules_;
};

} /* namespace loco */
