#include <pluginlib/class_list_macros.h>

#include <environment_utils/EnvironmentItemUtils.hpp>
#include <inspection_example/Item.hpp>
#include <inspection_example/conversions.hpp>
#include <message_logger/message_logger.hpp>

#include "inspection_example_qt_plugin/InspectionExampleQtPlugin.hpp"
#include "inspection_example_qt_plugin/InspectionExampleWidget.hpp"

// Register this inspection as a EnvironmentItemQtPluginInterface plugin.
PLUGINLIB_EXPORT_CLASS(inspection_example_qt_plugin::InspectionExampleQtPlugin,
                       environment_item_qt_ros::EnvironmentItemQtPluginInterfaceRos)

namespace inspection_example_qt_plugin {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

InspectionExampleQtPlugin::InspectionExampleQtPlugin() : environment_item_qt_ros::EnvironmentItemQtPluginInterfaceRos() {}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */

std::string InspectionExampleQtPlugin::getItemTypeImpl() {
  return inspection_example::Item::type_;
}

environment_item::ItemBasePtr InspectionExampleQtPlugin::createItemImpl() {
  std::shared_ptr<inspection_example::Item> item = std::make_shared<inspection_example::Item>();
  return item;
}

environment_item_qt_ros::EnvironmentItemWidgetInterfaceRos* InspectionExampleQtPlugin::createWidgetImpl() {
  auto widget = new InspectionExampleWidget;
  return widget;
}

std::vector<environment_item::ItemBasePtr> InspectionExampleQtPlugin::getItemsFromRosParameterServerImpl(ros::NodeHandle& nodeHandle,
                                                                                                         const std::string& container) {
  std::vector<environment_item::ItemBasePtr> items;

  for (auto item : environment_utils::EnvironmentItemUtils<inspection_example::Item>::getItems(nodeHandle, container)) {
    items.push_back(std::make_shared<inspection_example::Item>(item.second));
  }

  return items;
}

XmlRpc::XmlRpcValue InspectionExampleQtPlugin::convertItemToXmlRpcValueImpl(environment_item::ItemBasePtr item) {
  return environment_utils::EnvironmentItemUtils<inspection_example::Item>::itemToXmlRpcValue(
      *(std::dynamic_pointer_cast<inspection_example::Item>(item)));
}

}  // namespace inspection_example_qt_plugin
