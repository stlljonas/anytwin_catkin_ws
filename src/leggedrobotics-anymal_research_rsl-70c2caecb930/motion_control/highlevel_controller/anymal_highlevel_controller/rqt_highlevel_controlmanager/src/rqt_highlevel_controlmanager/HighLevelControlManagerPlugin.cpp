/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Dario Bellicoso                                                    *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include <QStringList>
#include <QGridLayout>
#include <rqt_highlevel_controlmanager/HighLevelControlManagerPlugin.h>
#include <ros/package.h>

namespace rqt_highlevel_controlmanager {

/*****************************************************************************/
/** Constructor/Destructor                                                  **/
/*****************************************************************************/

HighLevelControlManagerPlugin::HighLevelControlManagerPlugin() :
    rqt_gui_cpp::Plugin(),
    widget_(nullptr),
    nodeHandle_() {
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("HighLevelControlManagerPlugin");

  qRegisterMetaType<rocoma_msgs::SwitchControllerResponse>
      ("rocoma_msgs::SwitchControllerResponse");
  qRegisterMetaType<anymal_msgs::SwitchControllerResponse>
      ("anymal_msgs::SwitchControllerResponse");
  qRegisterMetaType<anymal_msgs::AnymalLowLevelControllerGoToStateResponse>
      ("anymal_msgs::AnymalLowLevelControllerGoToStateResponse");
  qRegisterMetaType<any_msgs::SetUInt32Response>
      ("any_msgs::SetUInt32Response");
  qRegisterMetaType<QModelIndex>("QModelIndex");
  qRegisterMetaType<std::string>("std::string");
}

/*****************************************************************************/
/** Init/Shutdown Plugin                                                    **/
/*****************************************************************************/
void HighLevelControlManagerPlugin::init(ros::NodeHandle& nodeHandle, QWidget* widget) {
  // create the main widget, set it up and add it to the user interface
  ui_.setupUi(widget);
  nodeHandle_ = nodeHandle;

  // Set pixmaps
  pixmapOk_ = QPixmap(
      QString::fromStdString(
          ros::package::getPath("rqt_highlevel_controlmanager").append(
              "/resource/ok.png")));
  pixmapError_ = QPixmap(
      QString::fromStdString(
          ros::package::getPath("rqt_highlevel_controlmanager").append(
              "/resource/error.png")));

  // Set images to buttons
  ui_.pushButtonRefreshController->setIcon(
      QIcon(QString::fromStdString(
          ros::package::getPath("rqt_highlevel_controlmanager").append(
              "/resource/refresh.png"))));
  ui_.pushButtonRefreshController->setIconSize(
      ui_.pushButtonRefreshController->size());

  ui_.pushButtonSetController->setIcon(
      QIcon(QString::fromStdString(
          ros::package::getPath("rqt_highlevel_controlmanager").append(
              "/resource/set.png"))));
  ui_.pushButtonSetController->setIconSize(ui_.pushButtonSetController->size());

  ui_.pushButtonRefreshMode->setIcon(
      QIcon(QString::fromStdString(
          ros::package::getPath("rqt_highlevel_controlmanager").append(
              "/resource/refresh.png"))));
  ui_.pushButtonRefreshMode->setIconSize(ui_.pushButtonRefreshMode->size());

  ui_.pushButtonSetMode->setIcon(
      QIcon(QString::fromStdString(
          ros::package::getPath("rqt_highlevel_controlmanager").append(
              "/resource/set.png"))));
  ui_.pushButtonSetMode->setIconSize(ui_.pushButtonSetMode->size());

  // Connect ui forms to actions
  connect(ui_.pushButtonRefreshController, SIGNAL(clicked()),
          this, SLOT(buttonRefreshControllerClicked()));
  connect(ui_.pushButtonSetController, SIGNAL(clicked()),
          this, SLOT(buttonSetControllerClicked()));
  connect(ui_.pushButtonRefreshMode, SIGNAL(clicked()),
          this, SLOT(buttonRefreshModeClicked()));
  connect(ui_.pushButtonSetMode, SIGNAL(clicked()),
          this, SLOT(buttonSetModeClicked()));
  connect(ui_.pushButtonEmergencyStop, SIGNAL(clicked()),
          this, SLOT(buttonEmergencyStopClicked()));
  connect(ui_.pushButtonClearEmergencyStop, SIGNAL(clicked()),
          this, SLOT(buttonClearEmergencyStopClicked()));
  connect(ui_.pushButtonResetStateEstimator, SIGNAL(clicked()),
          this, SLOT(buttonResetStateEstimatorClicked()));
  connect(ui_.pushButtonResetStateEstimatorHere, SIGNAL(clicked()),
          this, SLOT(buttonResetStateEstimatorHereClicked()));
  connect(ui_.pushButtonCalibrateFeet, SIGNAL(clicked()),
          this, SLOT(buttonCalibrateFeetClicked()));
  connect(this, SIGNAL(updateActiveController(std::string)),
          this, SLOT(activeControllerUpdated(std::string)));
  connect(this, SIGNAL(updateEmergencyState(const bool)),
          this, SLOT(emergencyStateUpdated(const bool)));
  connect(this, SIGNAL(updateManagerState(const int)),
  this, SLOT(managerStateUpdated(const int)));
  // set properties for image placeholders
  ui_.labelSwitchControllerIcon->setScaledContents(true);
  ui_.labelSwitchModeIcon->setScaledContents(true);
  ui_.labelEmergencyStateIcon->setScaledContents(true);

  // get simulation parameter
  simulation_ =
      param_io::param<bool>(
          nodeHandle_, "/controlmanager/simulation", false);

  // get services from parameter server
  serviceSwitchLowlevelController_ =
      param_io::param<std::string>(
          nodeHandle_, "/controlmanager/service/switch_lowlevel_controller", "/anymal_lowlevel_controller/go_to_state");
  serviceSwitchController_ =
      param_io::param<std::string>(
          nodeHandle_, "/controlmanager/service/switch_controller", "/anymal_highlevel_controller/switch_controller");
  serviceGetAvailableControllers_ =
      param_io::param<std::string>(
          nodeHandle_, "/controlmanager/service/get_available_controllers", "/anymal_highlevel_controller/get_available_controllers");
  serviceEmergencyStop_ =
      param_io::param<std::string>(
          nodeHandle_, "/controlmanager/service/emergency_stop", "/anymal_highlevel_controller/emergency_stop");
  serviceClearEmergencyStop_ =
      param_io::param<std::string>(
          nodeHandle_, "/controlmanager/service/clear_emergency_stop", "/anymal_highlevel_controller/clear_emergency_stop");
  serviceStateEstimatorReset_ =
      param_io::param<std::string>(
          nodeHandle_, "/controlmanager/service/state_estimator_reset", "/state_estimator/reset");
  serviceStateEstimatorResetHere_ =
      param_io::param<std::string>(
          nodeHandle_, "/controlmanager/service/state_estimator_reset_here", "/state_estimator/reset_here");
  serviceCalibrateFeet_ =
      param_io::param<std::string>(
          nodeHandle_, "/controlmanager/service/calibrate_feet", "/state_estimator/calibrate_forces");

  // get topics from parameter server
  topicActiveController_ =
      param_io::param<std::string>(
          nodeHandle_, "/controlmanager/topic/active_controller", "/anymal_highlevel_controller/notify_active_controller");
  topicControllerManagerState_ =
      param_io::param<std::string>(
          nodeHandle_, "/controlmanager/topic/controller_manager_state", "/anymal_highlevel_controller/notify_controller_manager_state");

  // setup service clients
  switchLowLevelControllerClient_ =
      nodeHandle_.serviceClient<anymal_msgs::AnymalLowLevelControllerGoToState>(
          serviceSwitchLowlevelController_);
  switchControllerClient_ =
      nodeHandle_.serviceClient<rocoma_msgs::SwitchController>(
          serviceSwitchController_);
  getAvailableControllersClient_ =
      nodeHandle_.serviceClient<rocoma_msgs::GetAvailableControllers>(
          serviceGetAvailableControllers_);
  emergencyStopClient_ =
      nodeHandle_.serviceClient<std_srvs::Trigger>(
          serviceEmergencyStop_);
  clearEmergencyStopClient_ =
      nodeHandle_.serviceClient<std_srvs::Trigger>(
          serviceClearEmergencyStop_);
  resetStateEstimatorClient_ =
      nodeHandle_.serviceClient<anymal_msgs::ResetStateEstimator>(
          serviceStateEstimatorReset_);
  resetStateEstimatorHereClient_ =
      nodeHandle_.serviceClient<std_srvs::Trigger>(
          serviceStateEstimatorResetHere_);
  calibrateFeetClient_ =
      nodeHandle_.serviceClient<any_msgs::SetUInt32>(
          serviceCalibrateFeet_);

  // setup subscribers
  activeControllerSubscriber_ =
      nodeHandle_.subscribe<std_msgs::String>(
          topicActiveController_, 1,
          &HighLevelControlManagerPlugin::activeControllerCallback, this);
  controllerManagerStateSubscriber_ =
      nodeHandle_.subscribe<rocoma_msgs::ControllerManagerState>(
          topicControllerManagerState_, 1,
          &HighLevelControlManagerPlugin::controllerManagerStateCallback, this);
}

void HighLevelControlManagerPlugin::initPlugin(
    qt_gui_cpp::PluginContext &context) {
  // create the main widget, set it up and add it to the user interface
  widget_ = new QWidget();
  init(getNodeHandle(), widget_);
  context.addWidget(widget_);
}

void HighLevelControlManagerPlugin::shutdownPlugin() {
  switchLowLevelControllerClient_.shutdown();
  switchControllerClient_.shutdown();
  getAvailableControllersClient_.shutdown();
  emergencyStopClient_.shutdown();
  clearEmergencyStopClient_.shutdown();
  resetStateEstimatorClient_.shutdown();
  resetStateEstimatorHereClient_.shutdown();
  switchControllerModeClient_.shutdown();
  getAvailableControllerModesClient_.shutdown();
  calibrateFeetClient_.shutdown();

  activeControllerSubscriber_.shutdown();
  controllerManagerStateSubscriber_.shutdown();
}

/*****************************************************************************/
/** Settings                                                                **/
/*****************************************************************************/

void HighLevelControlManagerPlugin::saveSettings(
    qt_gui_cpp::Settings &plugin_settings,
    qt_gui_cpp::Settings &instance_settings) const {
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void HighLevelControlManagerPlugin::restoreSettings(
    const qt_gui_cpp::Settings &plugin_settings,
    const qt_gui_cpp::Settings &instance_settings) {
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

/*****************************************************************************/
/** Slots                                                                   **/
/*****************************************************************************/

void HighLevelControlManagerPlugin::buttonRefreshControllerClicked() {
  updateAvailableControllers();
}

void HighLevelControlManagerPlugin::buttonSetControllerClicked() {
  mutex_.lock();

  anymal_msgs::AnymalLowLevelControllerGoToStateRequest lowLevelSwitchReq;
  lowLevelSwitchReq.state.state = anymal_msgs::AnymalLowLevelControllerState::STATE_OPERATIONAL;
  rocoma_msgs::SwitchControllerRequest switchReq;
  switchReq.name = ui_.comboBoxAvailableControllers->currentText().toStdString();

  // disable elements
  deactivateUi();

  // start non blocking service call
  WorkerThreadSetController *workerThreadSetController =
      new WorkerThreadSetController;
  connect(workerThreadSetController,
          SIGNAL(resultLowLevelReady(
              bool,
              anymal_msgs::AnymalLowLevelControllerGoToStateResponse)),
          this,
          SLOT(setLowLevelControllerResult(
              bool,
              anymal_msgs::AnymalLowLevelControllerGoToStateResponse)));
  connect(workerThreadSetController,
          SIGNAL(resultReady(
              bool,
              rocoma_msgs::SwitchControllerResponse)),
          this,
          SLOT(setControllerResult(
              bool,
              rocoma_msgs::SwitchControllerResponse)));
  connect(workerThreadSetController, SIGNAL(finished()),
          workerThreadSetController, SLOT(deleteLater()));
  workerThreadSetController->setSimulation(simulation_);
  workerThreadSetController->setLowLevelClient(switchLowLevelControllerClient_);
  workerThreadSetController->setClient(switchControllerClient_);
  workerThreadSetController->setLowLevelRequest(lowLevelSwitchReq);
  workerThreadSetController->setRequest(switchReq);
  workerThreadSetController->start();
}

void HighLevelControlManagerPlugin::setLowLevelControllerResult(
    bool isOk,
    anymal_msgs::AnymalLowLevelControllerGoToStateResponse responseLowLevel) {
  // enable elements
  activateUi();

  if (!isOk) {
    ROS_ERROR_STREAM_NAMED(TAG, TAG
        << " Could not call lowlevel_controller switch service.");
    QPixmap switchControllerImageError(
        QString::fromStdString(
            ros::package::getPath("rqt_highlevel_controlmanager").append(
                "/resource/error.png")));
    ui_.labelSwitchControllerIcon->setPixmap(switchControllerImageError);
  }

  mutex_.unlock();
}

void HighLevelControlManagerPlugin::setControllerResult(
    bool isOk,
    rocoma_msgs::SwitchControllerResponse response) {
  rocoma_msgs::SwitchControllerRequest switchReq;
  switchReq.name = ui_.comboBoxAvailableControllers->currentText().toStdString();

  // enable elements
  activateUi();

  if (isOk) {
    switch (response.status) {
      case (rocoma_msgs::SwitchController::ResponseType::STATUS_RUNNING):
      case (rocoma_msgs::SwitchController::ResponseType::STATUS_SWITCHED): {
        ui_.labelSwitchControllerIcon->setPixmap(pixmapOk_);
        initRefreshModesClient(switchReq.name);
        updateAvailableModes();
      }
        break;
      case (rocoma_msgs::SwitchController::ResponseType::STATUS_ERROR):
      case (rocoma_msgs::SwitchController::ResponseType::STATUS_NOTFOUND): {
        ui_.labelSwitchControllerIcon->setPixmap(pixmapError_);
      }
        break;
    }
  } else {
    QPixmap switchControllerImageError(
        QString::fromStdString(
            ros::package::getPath("rqt_highlevel_controlmanager").append(
                "/resource/error.png")));
    ui_.labelSwitchControllerIcon->setPixmap(switchControllerImageError);
  }

  ui_.labelControllerStatus->setText(
      QString::fromStdString(getStatusName(response)));

  mutex_.unlock();
}

void HighLevelControlManagerPlugin::buttonRefreshModeClicked() {
  initRefreshModesClient(
      ui_.comboBoxAvailableControllers->currentText().toStdString());
  updateAvailableModes();
}

void HighLevelControlManagerPlugin::buttonSetModeClicked() {
  mutex_.lock();

  anymal_msgs::SwitchControllerRequest switchReq;
  switchReq.name = ui_.comboBoxAvailableModes->currentText().toStdString();
  switchReq.block = false;

  // disable elements
  deactivateUi();

  switchControllerModeClient_.shutdown();
  switchControllerModeClient_ = nodeHandle_.
      serviceClient<anymal_msgs::SwitchController>(
      "/" + ui_.comboBoxAvailableControllers->currentText().toStdString() +
          "/go_to_mode");

  // start non blocking service call
  WorkerThreadSetMode *workerThreadSetMode = new WorkerThreadSetMode;
  connect(workerThreadSetMode,
          SIGNAL(resultReady(
              bool, anymal_msgs::SwitchControllerResponse)),
          this,
          SLOT(setModeResult(
              bool, anymal_msgs::SwitchControllerResponse)));
  connect(workerThreadSetMode, SIGNAL(finished()), workerThreadSetMode,
          SLOT(deleteLater()));
  workerThreadSetMode->setClient(switchControllerModeClient_);
  workerThreadSetMode->setRequest(switchReq);
  workerThreadSetMode->start();
}

void HighLevelControlManagerPlugin::buttonCalibrateFeetClicked() {
  // disable elements
  deactivateUi();

  any_msgs::SetUInt32Request request;
  request.data = 1000;

  // start non blocking service call
  WorkerThreadCalibrateFeet *workerThreadCalibrateFeet =
      new WorkerThreadCalibrateFeet;
  connect(workerThreadCalibrateFeet,
          SIGNAL(resultReady(bool, any_msgs::SetUInt32Response)),
          this, SLOT(calibrateFeetResult(bool, any_msgs::SetUInt32Response)));
  connect(workerThreadCalibrateFeet, SIGNAL(finished()),
          workerThreadCalibrateFeet, SLOT(deleteLater()));
  workerThreadCalibrateFeet->setClient(calibrateFeetClient_);
  workerThreadCalibrateFeet->setRequest(request);
  workerThreadCalibrateFeet->start();
}

void HighLevelControlManagerPlugin::calibrateFeetResult(
    bool isOk, any_msgs::SetUInt32Response response) {
  // enable elements
  activateUi();

  if (isOk && response.success) {
    ROS_INFO_STREAM_NAMED(TAG, TAG << " Feet successful calibrated.");
  } else {
    ROS_ERROR_STREAM_NAMED(TAG, TAG << " Could not calibrate the feet.");
  }
}

void HighLevelControlManagerPlugin::setModeResult(
    bool isOk, anymal_msgs::SwitchControllerResponse response) {

  // enable elements
  activateUi();

  if (isOk) {
    switch (response.status) {
      case (anymal_msgs::SwitchController::ResponseType::STATUS_RUNNING):
      case (anymal_msgs::SwitchController::ResponseType::STATUS_SWITCHED): {
        ui_.labelSwitchModeIcon->setPixmap(pixmapOk_);
      }
        break;

      case (anymal_msgs::SwitchController::ResponseType::STATUS_ERROR):
      case (anymal_msgs::SwitchController::ResponseType::STATUS_NOTFOUND): {
        ui_.labelSwitchModeIcon->setPixmap(pixmapError_);
      }
        break;
    }
  } else {
    ROS_ERROR_STREAM_NAMED(TAG, TAG
        << " Could not switch controller. Response was: "
        << getStatusName(response));
    ui_.labelSwitchModeIcon->setPixmap(pixmapError_);
  }

  mutex_.unlock();
}

void HighLevelControlManagerPlugin::activeControllerUpdated(
    std::string activeController) {
  std::lock_guard<std::mutex> lock_guard(mutex_);
  // Ensure the process cannot be interfered.
  deactivateUi();
  // Update active controller label.
  ui_.labelControllerName->setText(activeController.c_str());
  // Update the dropdown for available controllers.
  if (ui_.comboBoxAvailableControllers->findText(
      QString::fromStdString(activeController)) == -1) {
    // The controller does not yet exist in the list of controllers.
    if (!updateAvailableControllers()) {
      ROS_WARN_STREAM_NAMED(TAG,
                            TAG << " cannot update available controllers.");
      activateUi();
      return;
    }
    // Select active controller.
    int index = ui_.comboBoxAvailableControllers->findText(
        QString::fromStdString(activeController));
    if (index != -1) { // -1 for not found
      ui_.comboBoxAvailableControllers->setCurrentIndex(index);
    } else {
      ROS_WARN_STREAM_NAMED(TAG, TAG << " active controller is not available.");
      ui_.comboBoxAvailableModes->clear();
      activateUi();
      return;
    }
  }
  // Update available modes for the active controller.
  initRefreshModesClient(activeController);
  if (!updateAvailableModes()) {
    ROS_INFO_STREAM_NAMED(TAG, TAG << " no modes are available.");
  }
  // Release the ui lock.
  activateUi();
}

void HighLevelControlManagerPlugin::emergencyStateUpdated(const bool isOk)
{
  std::lock_guard<std::mutex> lock_guard(mutex_);
  if (isOk) {
    ui_.labelEmergencyStateIcon->setPixmap(pixmapOk_);
  } else {
    ui_.labelEmergencyStateIcon->setPixmap(pixmapError_);
  }
}

void HighLevelControlManagerPlugin::managerStateUpdated(const int state)
{
  std::lock_guard<std::mutex> lock_guard(mutex_);
  auto setColor = [](QLabel* l, std::string color){
    l->setStyleSheet(QString::fromStdString("QLabel { background-color : " + color + ";}")); };
  setColor(ui_.labelOK, state == rocoma_msgs::ControllerManagerState::STATE_OK ? "green" : "lightgray");
  setColor(ui_.labelEmergency, state == rocoma_msgs::ControllerManagerState::STATE_EMERGENCY ? "orange" : "lightgray");
  setColor(ui_.labelFailproof, state == rocoma_msgs::ControllerManagerState::STATE_FAILURE ? "red" : "lightgray");
}

void HighLevelControlManagerPlugin::buttonEmergencyStopClicked() {
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;

  if (!emergencyStopClient_.call(req, res)) {
    ROS_ERROR_STREAM_NAMED(TAG, TAG << " Emergency stop failed.");
  }
  else {
    if (!res.success) {
      ROS_ERROR_STREAM_NAMED(TAG, TAG << " Emergency stop failed.");
    }
  }
}

void HighLevelControlManagerPlugin::buttonClearEmergencyStopClicked() {
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;

  if (!clearEmergencyStopClient_.call(req, res)) {
    ROS_ERROR_STREAM_NAMED(TAG, TAG << " Clear Emergency stop failed.");
  }
  else {
    if (!res.success) {
      ROS_ERROR_STREAM_NAMED(TAG, TAG << " Clear Emergency stop failed.");
    }
  }
}

void HighLevelControlManagerPlugin::buttonResetStateEstimatorClicked() {
  anymal_msgs::ResetStateEstimator srv;

  srv.request.pose.orientation.w = 1.0;
  srv.request.pose.position.z = 0.0;

  if (!resetStateEstimatorClient_.call(srv)) {
    ROS_ERROR_STREAM_NAMED(TAG, TAG << " Could not reset state estimator.");
  }
}

void HighLevelControlManagerPlugin::buttonResetStateEstimatorHereClicked() {
  std_srvs::Trigger trigger;

  if (!resetStateEstimatorHereClient_.call(trigger)) {
    ROS_ERROR_STREAM_NAMED(TAG, TAG << " Could not reset state estimator.");
  }
}

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

std::string HighLevelControlManagerPlugin::getStatusName(
    const rocoma_msgs::SwitchController::Response &res) {
  std::string statusName = "";
  switch (res.status) {
    case (rocoma_msgs::SwitchControllerResponse::STATUS_ERROR): {
      statusName = "STATUS_ERROR";
    }
      break;
    case (rocoma_msgs::SwitchControllerResponse::STATUS_NOTFOUND): {
      statusName = "STATUS_NOTFOUND";
    }
      break;
    case (rocoma_msgs::SwitchControllerResponse::STATUS_SWITCHED): {
      statusName = "STATUS_SWITCHED";
    }
      break;
    case (rocoma_msgs::SwitchControllerResponse::STATUS_RUNNING): {
      statusName = "STATUS_RUNNING";
    }
      break;
    default:
      statusName = "undefined";
      break;
  }

  return statusName;
}

std::string HighLevelControlManagerPlugin::getStatusName(
    const anymal_msgs::SwitchController::Response &res) {
  std::string statusName = "";
  switch (res.status) {
    case (anymal_msgs::SwitchControllerResponse::STATUS_ERROR): {
      statusName = "STATUS_ERROR";
    }
      break;
    case (anymal_msgs::SwitchControllerResponse::STATUS_NOTFOUND): {
      statusName = "STATUS_NOTFOUND";
    }
      break;
    case (anymal_msgs::SwitchControllerResponse::STATUS_SWITCHED): {
      statusName = "STATUS_SWITCHED";
    }
      break;
    case (anymal_msgs::SwitchControllerResponse::STATUS_RUNNING): {
      statusName = "STATUS_RUNNING";
    }
      break;
    default:
      statusName = "undefined";
      break;
  }

  return statusName;
}

bool HighLevelControlManagerPlugin::updateAvailableControllers() {
  rocoma_msgs::GetAvailableControllers::Request req;
  rocoma_msgs::GetAvailableControllers::Response res;

  ui_.comboBoxAvailableControllers->clear();

  bool isOk = false;
  if (getAvailableControllersClient_.call(req, res)) {
    isOk = true;
    for (auto name : res.available_controllers) {
      ui_.comboBoxAvailableControllers->addItem(QString::fromStdString(name));
    }
  }

  return isOk;
}

bool HighLevelControlManagerPlugin::updateAvailableModes() {
  anymal_msgs::GetAvailableControllers::Request req;
  anymal_msgs::GetAvailableControllers::Response res;

  ui_.comboBoxAvailableModes->clear();

  bool isOk = false;
  if (getAvailableControllerModesClient_.call(req, res)) {
    isOk = true;
    for (auto name : res.available_controllers) {
      ui_.comboBoxAvailableModes->addItem(QString::fromStdString(name));
    }
  }
  if (ui_.comboBoxAvailableModes->count() > 0) {
    ui_.comboBoxAvailableModes->setCurrentIndex(0);
  }

  return isOk;
}

void HighLevelControlManagerPlugin::deactivateUi() {
  ui_.comboBoxAvailableControllers->setEnabled(false);
  ui_.pushButtonRefreshController->setEnabled(false);
  ui_.pushButtonSetController->setEnabled(false);
  ui_.comboBoxAvailableModes->setEnabled(false);
  ui_.pushButtonRefreshMode->setEnabled(false);
  ui_.pushButtonSetMode->setEnabled(false);
  ui_.pushButtonCalibrateFeet->setEnabled(false);
}

void HighLevelControlManagerPlugin::activateUi() {
  ui_.comboBoxAvailableControllers->setEnabled(true);
  ui_.pushButtonRefreshController->setEnabled(true);
  ui_.pushButtonSetController->setEnabled(true);
  ui_.comboBoxAvailableModes->setEnabled(true);
  ui_.pushButtonRefreshMode->setEnabled(true);
  ui_.pushButtonSetMode->setEnabled(true);
  ui_.pushButtonCalibrateFeet->setEnabled(true);
}

void HighLevelControlManagerPlugin::initRefreshModesClient(
    std::string activeController) {
  getAvailableControllerModesClient_.shutdown();
  getAvailableControllerModesClient_ = nodeHandle_.
      serviceClient<anymal_msgs::GetAvailableControllers>(
      "/" + activeController + "/get_available_modes");
}

/*****************************************************************************/
/** Callbacks                                                               **/
/*****************************************************************************/

void HighLevelControlManagerPlugin::activeControllerCallback(
    const std_msgs::StringConstPtr &msg) {
  std::lock_guard<std::mutex> lock_guard(mutex_);
  emit updateActiveController(msg->data);
}

void HighLevelControlManagerPlugin::controllerManagerStateCallback(
    const rocoma_msgs::ControllerManagerStateConstPtr& msg) {
  std::lock_guard<std::mutex> lock_guard(mutex_);
  emit updateEmergencyState(msg->estop_cleared);
  emit updateManagerState(msg->state);
}

} // namespace
