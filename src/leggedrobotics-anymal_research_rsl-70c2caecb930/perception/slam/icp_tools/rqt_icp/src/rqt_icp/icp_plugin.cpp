#include "rqt_icp/icp_plugin.h"

// ros pluginlib
#include <pluginlib/class_list_macros.h>

// qt
#include <QStringList>

// std srvs
#include <std_srvs/Empty.h>

// any msgs
#include <any_msgs/ProcessFile.h>
#include <any_msgs/Toggle.h>

// message logger
#include <message_logger/message_logger.hpp>

// Subscriber callback parameterization
#include <boost/bind.hpp>
#include <boost/bind/make_adaptable.hpp>

IcpPlugin::IcpPlugin() : rqt_gui_cpp::Plugin() {
  setObjectName("IcpPlugin");
}

#define MAP_SIGNAL_WITH_IDX(my_signal, callback, idx)                      \
  {                                                                        \
    QSignalMapper* signalMapper = new QSignalMapper(this);                 \
    connect(ui_->my_signal, SIGNAL(pressed()), signalMapper, SLOT(map())); \
    signalMapper->setMapping(ui_->my_signal, idx);                         \
    connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(callback(int))); \
    signalMappers_.push_back(signalMapper);                                \
  }

void IcpPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  // access standalone command line arguments
  QStringList argv = context.argv();
  // Create the tabbed pane
  tabwidget_ = new QTabWidget();
  tabwidget_->setTabPosition(QTabWidget::TabPosition::South);

  // Get the list of tabs to show from the parameter server
  XmlRpc::XmlRpcValue tablist;
  if (!getNodeHandle().getParam("/rqt_icp/tabs", tablist)) {
    MELO_ERROR_STREAM("Parameters for ICP Plugin not loaded");
    return;
  }

  // Add all of the ICP tabs listed in the configs to the widget
  int tab_idx = 0;
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator tab = tablist.begin(); tab != tablist.end(); ++tab) {
    XmlRpc::XmlRpcValue tab_vals = tab->second;
    if (!checkRqtIcpConfigParams(tab->first, tab_vals)) {
      continue;
    }

    auto ui_ = new Ui::my_plugin();
    uis_.push_back(ui_);

    auto newwidget_ = new QWidget();
    widgets_.push_back(newwidget_);
    ui_->setupUi(newwidget_);

    ui_->startLocalizationButton->setCheckable(true);
    ui_->stopLocalizationButton->setCheckable(true);
    ui_->startPublishPoseButton->setCheckable(true);
    ui_->stopPublishPoseButton->setCheckable(true);
    ui_->startMappingButton->setCheckable(true);
    ui_->stopMappingButton->setCheckable(true);

    QObject::connect(this, SIGNAL(nodeStateChanged(int)), this, SLOT(updateStateVisualization(int)));

    // Connect the signals to parameterized slots
    connect(ui_->startLocalizationButton, &QPushButton::pressed, this, [this, tab_idx] { startLocalization(tab_idx); });
    connect(ui_->stopLocalizationButton, &QPushButton::pressed, this, [this, tab_idx] { stopLocalization(tab_idx); });
    connect(ui_->startPublishPoseButton, &QPushButton::pressed, this, [this, tab_idx] { startPublishPose(tab_idx); });
    connect(ui_->stopPublishPoseButton, &QPushButton::pressed, this, [this, tab_idx] { stopPublishPose(tab_idx); });
    connect(ui_->startMappingButton, &QPushButton::pressed, this, [this, tab_idx] { startMapping(tab_idx); });
    connect(ui_->stopMappingButton, &QPushButton::pressed, this, [this, tab_idx] { stopMapping(tab_idx); });
    connect(ui_->saveMapToFileButton, &QPushButton::pressed, this, [this, tab_idx] { saveMapToFile(tab_idx); });
    connect(ui_->loadMapFromFileButton, &QPushButton::pressed, this, [this, tab_idx] { loadMapFromFile(tab_idx); });
    connect(ui_->clearMapButton, &QPushButton::pressed, this, [this, tab_idx] { clearMap(tab_idx); });
    connect(ui_->publishMapButton, &QPushButton::pressed, this, [this, tab_idx] { publishMap(tab_idx); });
    connect(ui_->readConfigFilesButton, &QPushButton::pressed, this, [this, tab_idx] { readConfigFiles(tab_idx); });
    connect(ui_->refreshMapFileButton, &QPushButton::pressed, this, [this, tab_idx] { refreshMapFile(tab_idx); });

    ui_->localizationPipelineIsRunning->setScaledContents(true);
    pixmapOk_ = QPixmap(":/icons/ok.png");
    pixmapError_ = QPixmap(":/icons/error.png");

    toggleLocalizationClients_.push_back(getNodeHandle().serviceClient<any_msgs::Toggle>(tab_vals["toggle_localization_service"]));
    togglePublishPoseClients_.push_back(getNodeHandle().serviceClient<any_msgs::Toggle>(tab_vals["toggle_publish_pose_service"]));
    toggleMappingClients_.push_back(getNodeHandle().serviceClient<any_msgs::Toggle>(tab_vals["toggle_mapping_service"]));
    saveMapToFileClients_.push_back(getNodeHandle().serviceClient<any_msgs::ProcessFile>(tab_vals["save_map_to_file_service"]));
    loadMapFromFileClients_.push_back(getNodeHandle().serviceClient<any_msgs::ProcessFile>(tab_vals["load_map_from_file_service"]));
    clearMapClients_.push_back(getNodeHandle().serviceClient<std_srvs::Empty>(tab_vals["clear_map_service"]));
    publishMapClients_.push_back(getNodeHandle().serviceClient<std_srvs::Empty>(tab_vals["publish_map_service"]));
    readConfigFilesClients_.push_back(getNodeHandle().serviceClient<std_srvs::Empty>(tab_vals["read_config_files_service"]));

    nodeStateSubscribers_.push_back(getNodeHandle().subscribe<slam_common_msgs::NodeState>(
        tab_vals["node_state_topic"], 1, boost::bind(&IcpPlugin::nodeStateCallback, this, _1, tab_idx)));

    // Set the parameter for file to load
    mapOnLaunchFiles_.push_back(new std::string(tab_vals["load_map_on_launch_param"]));

    // Add this widget to the respective tab pane
    tabwidget_->addTab(newwidget_, QString::fromStdString(tab_vals["tab_name"]));

    // Add a blank node state to the vector
    nodeStateMsgs_.emplace_back(slam_common_msgs::NodeState());

    // Increment the tab index value
    tab_idx++;
  }
  for (int i = 0; i < tab_idx; i++) {
    refreshMapFile(i);
  }
  // add tabbed pane to the user interface
  context.addWidget(tabwidget_);
}

void IcpPlugin::startLocalization(int idx) {
  if (!toggleLocalizationClients_[idx].waitForExistence(ros::Duration(2.0))) {
    MELO_ERROR_STREAM("Toggle localization service has not been advertised.");
    return;
  }

  any_msgs::Toggle toggle;
  toggle.request.enable = true;  // NOLINT
  if (!toggleLocalizationClients_[idx].call(toggle)) {
    MELO_ERROR_STREAM("Failed to call service to toggle localization.");
    return;
  }
}

void IcpPlugin::stopLocalization(int idx) {
  if (!toggleLocalizationClients_[idx].waitForExistence(ros::Duration(2.0))) {
    MELO_ERROR_STREAM("Toggle localization service has not been advertised.");
    return;
  }

  any_msgs::Toggle toggle;
  toggle.request.enable = false;  // NOLINT
  if (!toggleLocalizationClients_[idx].call(toggle)) {
    MELO_ERROR_STREAM("Failed to call service to toggle localization.");
    return;
  }
}

void IcpPlugin::startPublishPose(int idx) {
  if (!togglePublishPoseClients_[idx].waitForExistence(ros::Duration(2.0))) {
    MELO_ERROR_STREAM("Toggle publish pose service has been advertised.");
    return;
  }

  any_msgs::Toggle toggle;
  toggle.request.enable = true;  // NOLINT
  if (!togglePublishPoseClients_[idx].call(toggle)) {
    MELO_ERROR_STREAM("Failed to call service to toggle publish pose.");
    return;
  }
}

void IcpPlugin::stopPublishPose(int idx) {
  if (!togglePublishPoseClients_[idx].waitForExistence(ros::Duration(2.0))) {
    MELO_ERROR_STREAM("Toggle publish pose service has been advertised.");
    return;
  }

  any_msgs::Toggle toggle;
  toggle.request.enable = false;  // NOLINT
  if (!togglePublishPoseClients_[idx].call(toggle)) {
    MELO_ERROR_STREAM("Failed to call service to toggle publish pose.");
    return;
  }
}

void IcpPlugin::startMapping(int idx) {
  if (!toggleMappingClients_[idx].waitForExistence(ros::Duration(2.0))) {
    MELO_ERROR_STREAM("Toggle mapping service has been advertised.");
    return;
  }

  any_msgs::Toggle toggle;
  toggle.request.enable = true;  // NOLINT
  if (!toggleMappingClients_[idx].call(toggle)) {
    MELO_ERROR_STREAM("Failed to call service to toggle mapping.");
    return;
  }
}

void IcpPlugin::stopMapping(int idx) {
  if (!toggleMappingClients_[idx].waitForExistence(ros::Duration(2.0))) {
    MELO_ERROR_STREAM("Toggle mapping service has been advertised.");
    return;
  }

  any_msgs::Toggle toggle;
  toggle.request.enable = false;  // NOLINT
  if (!toggleMappingClients_[idx].call(toggle)) {
    MELO_ERROR_STREAM("Failed to call service to toggle mapping.");
    return;
  }
}

void IcpPlugin::saveMapToFile(int idx) {
  if (!saveMapToFileClients_[idx].waitForExistence(ros::Duration(2.0))) {
    MELO_ERROR_STREAM("Save map to file service has been advertised.");
    return;
  }

  any_msgs::ProcessFile processFile;
  processFile.request.file_path = uis_[idx]->mapFileField->text().toStdString();
  if (!saveMapToFileClients_[idx].call(processFile)) {
    MELO_ERROR_STREAM("Failed to call service to save map to file.");
    return;
  }
}

void IcpPlugin::loadMapFromFile(int idx) {
  if (!loadMapFromFileClients_[idx].waitForExistence(ros::Duration(2.0))) {
    MELO_ERROR_STREAM("Load map from file service has been advertised.");
    return;
  }

  any_msgs::ProcessFile processFile;
  processFile.request.file_path = uis_[idx]->mapFileField->text().toStdString();
  if (!loadMapFromFileClients_[idx].call(processFile)) {
    MELO_ERROR_STREAM("Failed to call service to load map from file.");
    return;
  }
}

void IcpPlugin::clearMap(int idx) {
  if (!clearMapClients_[idx].waitForExistence(ros::Duration(2.0))) {
    MELO_ERROR_STREAM("Clear map service has been advertised.");
    return;
  }

  std_srvs::Empty empty;
  if (!clearMapClients_[idx].call(empty)) {
    MELO_ERROR_STREAM("Failed to call service to clear map.");
    return;
  }
}

void IcpPlugin::publishMap(int idx) {
  if (!publishMapClients_[idx].waitForExistence(ros::Duration(2.0))) {
    MELO_ERROR_STREAM("Publish map service has been advertised.");
    return;
  }

  std_srvs::Empty empty;
  if (!publishMapClients_[idx].call(empty)) {
    MELO_ERROR_STREAM("Failed to call service to publish map.");
    return;
  }
}

void IcpPlugin::readConfigFiles(int idx) {
  if (!readConfigFilesClients_[idx].waitForExistence(ros::Duration(2.0))) {
    MELO_ERROR_STREAM("Read config files service has been advertised.");
    return;
  }

  std_srvs::Empty empty;
  if (!readConfigFilesClients_[idx].call(empty)) {
    MELO_ERROR_STREAM("Failed to call service to read config files.");
    return;
  }
}

void IcpPlugin::refreshMapFile(int idx) {
  const std::string param = *mapOnLaunchFiles_[idx];
  std::string file;
  if (!getNodeHandle().getParam(param, file)) {
    MELO_INFO_STREAM("Could not acquire parameter '" << param << "' from server, has ICP been started?");
    return;
  }
  uis_[idx]->mapFileField->setText(QString::fromStdString(file));
}

void IcpPlugin::updateStateVisualization(int idx) {
  uis_[idx]->startLocalizationButton->setChecked(static_cast<bool>(nodeStateMsgs_[idx].input_point_cloud_matching_is_enabled));
  uis_[idx]->stopLocalizationButton->setChecked(!static_cast<bool>(nodeStateMsgs_[idx].input_point_cloud_matching_is_enabled));
  uis_[idx]->startPublishPoseButton->setChecked(static_cast<bool>(nodeStateMsgs_[idx].localization_is_enabled));
  uis_[idx]->stopPublishPoseButton->setChecked(!static_cast<bool>(nodeStateMsgs_[idx].localization_is_enabled));
  uis_[idx]->startMappingButton->setChecked(static_cast<bool>(nodeStateMsgs_[idx].mapping_is_enabled));
  uis_[idx]->stopMappingButton->setChecked(!static_cast<bool>(nodeStateMsgs_[idx].mapping_is_enabled));

  if (static_cast<bool>(nodeStateMsgs_[idx].localization_is_successful)) {
    uis_[idx]->localizationPipelineIsRunning->setPixmap(pixmapOk_);
  } else {
    uis_[idx]->localizationPipelineIsRunning->setPixmap(pixmapError_);
  }

  QString text;

  text.setNum(nodeStateMsgs_[idx].input_point_cloud_age, 'f', 3);
  uis_[idx]->inputPointCloudAge->setText(text);
  uis_[idx]->inputPointCloudSize->setNum(static_cast<int>(nodeStateMsgs_[idx].input_point_cloud_size));
  uis_[idx]->mapSize->setNum(static_cast<int>(nodeStateMsgs_[idx].map_point_cloud_size));
  uis_[idx]->staticMapSize->setNum(static_cast<int>(nodeStateMsgs_[idx].static_map_point_cloud_size));

  text.setNum(nodeStateMsgs_[idx].registration_overlap, 'f', 3);
  uis_[idx]->icpOverlap->setText(text);

  text.setNum(nodeStateMsgs_[idx].time_used_for_localization, 'f', 3);
  uis_[idx]->timeUsedForLocalization->setText(text);
  text.setNum(nodeStateMsgs_[idx].time_between_input_point_clouds, 'f', 3);
  uis_[idx]->timeAvailableForLocalization->setText(text);
  text.setNum(nodeStateMsgs_[idx].time_used_for_mapping, 'f', 3);
  uis_[idx]->timeUsedForMapping->setText(text);
}

void IcpPlugin::nodeStateCallback(const slam_common_msgs::NodeState::ConstPtr& stateMsg, int idx) {
  nodeStateMsgs_[idx] = *stateMsg;
  emit nodeStateChanged(idx);
}

#define SHUTDOWN_ALL_SERVICES(list)                                                    \
  std::for_each(list.begin(), list.end(), [](ros::ServiceClient x) { x.shutdown(); }); \
  list.clear();
#define SHUTDOWN_ALL_SUBSCRIBERS(list)                                              \
  std::for_each(list.begin(), list.end(), [](ros::Subscriber x) { x.shutdown(); }); \
  list.clear();
#define DELETE_ALL_PTRS(list) \
  for (auto ptr : list) {     \
    delete ptr;               \
  }                           \
  list.clear();

void IcpPlugin::shutdownPlugin() {
  SHUTDOWN_ALL_SERVICES(toggleLocalizationClients_)
  SHUTDOWN_ALL_SERVICES(togglePublishPoseClients_)
  SHUTDOWN_ALL_SERVICES(toggleMappingClients_)
  SHUTDOWN_ALL_SERVICES(saveMapToFileClients_)
  SHUTDOWN_ALL_SERVICES(loadMapFromFileClients_)
  SHUTDOWN_ALL_SERVICES(clearMapClients_)
  SHUTDOWN_ALL_SERVICES(publishMapClients_)
  SHUTDOWN_ALL_SERVICES(readConfigFilesClients_)

  SHUTDOWN_ALL_SUBSCRIBERS(nodeStateSubscribers_);

  // Cleanup dynamically allocated objets
  DELETE_ALL_PTRS(uis_)
  DELETE_ALL_PTRS(widgets_)
  DELETE_ALL_PTRS(signalMappers_)
  DELETE_ALL_PTRS(mapOnLaunchFiles_)
  nodeStateMsgs_.clear();
}

void IcpPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const {
  // TODO(PerceptionTeam) save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void IcpPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {
  // TODO(PerceptionTeam) restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

#define MISSING_PARAM(tab_key, values, param)                                                                       \
  if (!values.hasMember(param)) {                                                                                   \
    MELO_ERROR_STREAM("Config for tab '" << tab_key << "' missing parameter '" << param << "'. Skipping this tab"); \
    return false;                                                                                                   \
  }

bool IcpPlugin::checkRqtIcpConfigParams(std::string tab_key, XmlRpc::XmlRpcValue values) {
  MISSING_PARAM(tab_key, values, "tab_name")
  MISSING_PARAM(tab_key, values, "toggle_localization_service")
  MISSING_PARAM(tab_key, values, "toggle_publish_pose_service")
  MISSING_PARAM(tab_key, values, "toggle_mapping_service")
  MISSING_PARAM(tab_key, values, "save_map_to_file_service")
  MISSING_PARAM(tab_key, values, "load_map_from_file_service")
  MISSING_PARAM(tab_key, values, "clear_map_service")
  MISSING_PARAM(tab_key, values, "publish_map_service")
  MISSING_PARAM(tab_key, values, "read_config_files_service")
  MISSING_PARAM(tab_key, values, "node_state_topic")
  MISSING_PARAM(tab_key, values, "load_map_on_launch_param")
  return true;
}

PLUGINLIB_EXPORT_CLASS(IcpPlugin, rqt_gui_cpp::Plugin)
