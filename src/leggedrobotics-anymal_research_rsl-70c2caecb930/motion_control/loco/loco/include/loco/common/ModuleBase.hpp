/*!
 * @file	  ModuleBase.hpp
 * @author	Gabriel Hottiger
 * @date	  Jan 9, 2017
 */

#pragma once

// tinyxml
#include <tinyxml.h>

// STL
#include <string>
#include <utility>

namespace loco {

class ModuleBase {
 public:
  //! Default Constructor
  explicit ModuleBase(std::string name = "unknown") : name_(std::move(name)) {}

  //! Default Destructor
  virtual ~ModuleBase() = default;

  //! @return name of the module
  const std::string& getName() const { return name_; }

  //! @param name of the module
  void setName(const std::string& name) { name_ = name; }

  /** Initializes the module
   * @param dt  time step
   * @return true, iff successful
   */
  virtual bool initialize(double dt) = 0;

  /** Advances the module
   * @param dt  time step
   * @return    true, iff successful
   */
  virtual bool advance(double dt) = 0;

  /** Loads parameters from an xml file
   *  @param handle tinyxml handle
   *  @return true, iff successful
   */
  virtual bool loadParameters(const TiXmlHandle& /* handle */) { return true; }

  /** Adds parameters to the parameter handler
   * @param  ns, namespace  of the parameters
   * @return true, iff successful
   */
  virtual bool addParametersToHandler(const std::string& /* ns */) { return true; }

  /** Adds parameters to the parameter handler
   * @return true, iff successful
   */
  bool addParametersToHandler() { return addParametersToHandler(""); }

  /** Removes parameters from the parameter handler
   * @return true, iff successful
   */
  virtual bool removeParametersFromHandler() { return true; }

  /** Adds variables to the signal logger
   * @param ns            namespace of the variables
   * @return              true, iff successful
   */
  virtual bool addVariablesToLog(const std::string& /* ns */) const { return true; }

  /** Adds variables to the signal logger
   * @return              true, iff successful
   */
  bool addVariablesToLog() const { return addVariablesToLog(""); }

  /** Overloads to << ostream operator for modules, this allows e.g. std::cout << mymodule << std::endl
   * @param out ostream of the << operation
   * @return ostream
   */
  friend std::ostream& operator<<(std::ostream& out, const ModuleBase& module) {
    module.print(out);
    return out;
  }

  /** Function that is called after emergency stop. Use this function to properly shut down
   * threads, to enforce a certain state or to prepare a following initialization step.
   */
  virtual bool stop() { return true; }

 protected:
  /** Virtual print helper method for polymorph << ostream operator overload
   *  @param out ostream of the << operation
   */
  virtual void print(std::ostream& /* out */) const {};

 private:
  //! Module name
  std::string name_;
};

}  // namespace loco
