#include <algorithm>

#include <pluginlib/class_list_macros.h>

#include <anydrive/common/Macros.hpp>
#include <anydrive/common/sorting.hpp>
#include <param_io/get_param.hpp>

#include "rqt_anydrive_monitor/AnydriveMonitorPlugin.h"

namespace rqt_anydrive_monitor {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

AnydriveMonitorPlugin::AnydriveMonitorPlugin() : rqt_gui_cpp::Plugin(), widget_(nullptr) {
  setObjectName("AnydriveMonitorPlugin");
}

/* ========================================================================== */
/* Initialize/Shutdown                                                        */
/* ========================================================================== */

void AnydriveMonitorPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // Load setup.
  std::string actuatorReadingsExtendedTopic = "/anydrive_ros/readings_extended_throttled";
  param_io::getParam(getNodeHandle(), "ros_prefix", rosPrefix_);
  param_io::getParam(getNodeHandle(), "single_subscribers", singleSubscribers_);
  param_io::getParam(getNodeHandle(), "actuator_readings_extended_topic", actuatorReadingsExtendedTopic);
  loadAnydrives();

  // Initialize readings subscriber.
  if (!singleSubscribers_) {
    readingsSubscriber_ = getNodeHandle().subscribe(actuatorReadingsExtendedTopic, 1, &AnydriveMonitorPlugin::readingsCallback, this);
  }

  // Initialize update timer.
  updateTimer_ = new QTimer();
  connect(updateTimer_, SIGNAL(timeout()), this, SLOT(update()));
  updateTimer_->start(updateFq_);

  // Start up.
  startup();
}

void AnydriveMonitorPlugin::shutdownPlugin() {
  updateTimer_->stop();

  if (!singleSubscribers_) {
    readingsSubscriber_.shutdown();
  }

  shutdown();
}

/* ========================================================================== */
/* Settings                                                                   */
/* ========================================================================== */

void AnydriveMonitorPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& /*instance_settings*/) const {
  plugin_settings.setValue("feedback_section_state", ui_.widgetAnydriveMonitor->getFeedbackSectionState());
  plugin_settings.setValue("parameters_section_state", ui_.widgetAnydriveMonitor->getParametersSectionState());
  plugin_settings.setValue("command_section_state", ui_.widgetAnydriveMonitor->getCommandSectionState());
}

void AnydriveMonitorPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                            const qt_gui_cpp::Settings& /*instance_settings*/) {
  ui_.widgetAnydriveMonitor->setFeedbackSectionState(plugin_settings.value("feedback_section_state", true).toBool());
  ui_.widgetAnydriveMonitor->setParametersSectionState(plugin_settings.value("parameters_section_state", false).toBool());
  ui_.widgetAnydriveMonitor->setCommandSectionState(plugin_settings.value("command_section_state", true).toBool());
}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */

void AnydriveMonitorPlugin::loadAnydrives() {
  // Read parameter server.
  const std::string container = "/anymal_lowlevel_controller/anydrive_setup/anydrives";
  XmlRpc::XmlRpcValue params;
  try {
    if (!param_io::getParam(getNodeHandle(), container, params)) {
      ANYDRIVE_ERROR("Container '" << container << "' does not exist.")
    }
  } catch (const XmlRpc::XmlRpcException& exception) {
    ANYDRIVE_ERROR("Caught an XmlRpc exception while getting container '" << container << "': " << exception.getMessage() << ".")
  }

  std::vector<int> order;
  std::vector<std::string> anydrives;
  for (auto& param : params) {
    // Read id.
    if (param.second.hasMember("id")) {
      order.push_back(param_io::getMember<int>(param.second, "id"));
    } else {
      ANYDRIVE_ERROR("Every ANYdrive needs an id.")
      return;
    }

    // Read name.
    anydrives.push_back(param.first);
  }

  // Order the ANYdrives according to the order.
  auto permutation = anydrive::common::sortPermutation(order, [](int const& a, int const& b) { return a < b; });
  order = anydrive::common::applyPermutation(order, permutation);
  auto adjacent = std::adjacent_find(order.begin(), order.end());
  if (adjacent != order.end()) {
    ANYDRIVE_ERROR("Duplicated ids (" << *adjacent << ") are not allowed.")
    return;
  }
  anydrives = anydrive::common::applyPermutation(anydrives, permutation);

  // Setup ANYdrives.
  for (const auto& item : anydrives) {
    setupAnydrive(item);
  }

  ui_.widgetAnydriveMonitor->addSpacer();
}

void AnydriveMonitorPlugin::startup() {
  for (auto& anydriveInterface : anydriveInterfaces_) {
    anydriveInterface->startup();
  }
}

void AnydriveMonitorPlugin::shutdown() {
  for (auto& anydriveInterface : anydriveInterfaces_) {
    anydriveInterface->shutdown();
  }
}

void AnydriveMonitorPlugin::setupAnydrive(const std::string& name) {
  anydriveInterfaces_.push_back(new AnydriveInterfaceRos(getNodeHandle(), name, rosPrefix_, singleSubscribers_, ui_.widgetAnydriveMonitor));

  ui_.widgetAnydriveMonitor->pushBackAnydriveInterface(anydriveInterfaces_.back());
}

/* ========================================================================== */
/* Callbacks                                                                  */
/* ========================================================================== */

void AnydriveMonitorPlugin::readingsCallback(const anydrive_msgs::ReadingsExtended& readingsMsg) {
  for (unsigned int i = 0; i < anydriveInterfaces_.size(); i++) {
    anydriveInterfaces_[i]->readingCallback(readingsMsg.readings[i]);
  }
}

/* ========================================================================== */
/* Slots                                                                      */
/* ========================================================================== */

void AnydriveMonitorPlugin::update() {
  for (auto& anydriveInterface : anydriveInterfaces_) {
    anydriveInterface->updateAnydrive();
  }
}

}  // namespace rqt_anydrive_monitor

PLUGINLIB_EXPORT_CLASS(rqt_anydrive_monitor::AnydriveMonitorPlugin, rqt_gui_cpp::Plugin)
