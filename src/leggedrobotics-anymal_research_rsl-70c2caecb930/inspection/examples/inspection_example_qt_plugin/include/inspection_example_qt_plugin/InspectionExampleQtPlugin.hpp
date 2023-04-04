#pragma once

#include <pluginlib/class_loader.h>

#include <environment_item_qt_ros/EnvironmentItemQtPluginInterfaceRos.hpp>
#include <environment_item_qt_ros/EnvironmentItemWidgetInterfaceRos.hpp>

namespace inspection_example_qt_plugin {

class InspectionExampleQtPlugin : public environment_item_qt_ros::EnvironmentItemQtPluginInterfaceRos {
 public:
  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  explicit InspectionExampleQtPlugin();

  ~InspectionExampleQtPlugin() override = default;

 private:
  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  std::string getItemTypeImpl() override;

  environment_item::ItemBasePtr createItemImpl() override;

  environment_item_qt_ros::EnvironmentItemWidgetInterfaceRos* createWidgetImpl() override;

  std::vector<environment_item::ItemBasePtr> getItemsFromRosParameterServerImpl(ros::NodeHandle& nodeHandle,
                                                                                const std::string& container) override;

  XmlRpc::XmlRpcValue convertItemToXmlRpcValueImpl(environment_item::ItemBasePtr item) override;
};

}  // namespace inspection_example_qt_plugin
