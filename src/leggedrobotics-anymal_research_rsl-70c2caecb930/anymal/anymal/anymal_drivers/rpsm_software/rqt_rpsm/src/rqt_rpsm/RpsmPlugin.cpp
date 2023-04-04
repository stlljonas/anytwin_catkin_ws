/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Linus Isler <lisler@anybotics.com>                                 *
 ******************************************************************************/

#include "rqt_rpsm/RpsmPlugin.h"

#include <algorithm>

#include <QDialogButtonBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QTableWidgetItem>
#include <QMetaType>

#include <pluginlib/class_list_macros.h>

namespace rqt_rpsm {

/*****************************************************************************/
/** Constructor/Destructor                                                  **/
/*****************************************************************************/

RpsmPlugin::RpsmPlugin()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
    , receivedMainboardInsteadOfLpc_(false)
//    , pw_()
    , shownTemperatureWarning_(false)
{
  setObjectName("RpsmPlugin");
  qRegisterMetaType<rpsm_msgs::ListDirectoryResponse>
        ("rpsm_msgs::ListDirectoryResponse");
  qRegisterMetaType<std_srvs::SetBoolRequest>
      ("std_srvs::SetBoolRequest");
  qRegisterMetaType<QModelIndex>("QModelIndex");
  qRegisterMetaType<QVector<int>>("QVector<int>");
  qRegisterMetaType<std::string>("std::string");
}

/*****************************************************************************/
/** Initialize/Shutdown                                                     **/
/*****************************************************************************/

void RpsmPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  nh_ = getNodeHandle();
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(widget_->windowTitle() +
        " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  confirmPowerShutdownMsgBox_ = new QMessageBox(
        QMessageBox::Warning,
        "Please confirm power shutdown",
        "Make sure that the robot is lying on the ground or is suspended! "
        "Legs will collapse when shutting down power.",
        QMessageBox::Ok | QMessageBox::Cancel,
        widget_);
  confirmABShutdownMsgBox_ = new QMessageBox(
        QMessageBox::Warning,
        "Please confirm leg shutdown",
        "Make sure that the robot is lying on the ground or is suspended! "
        "Legs will collapse when shutting down actuation boards.",
        QMessageBox::Ok | QMessageBox::Cancel,
        widget_);

  // Connect buttons
  connect(ui_.shutdownPushButton, SIGNAL(clicked()),
          this, SLOT(onPushButtonShutdownClicked()));
  connect(ui_.actuationBoardPushButton, SIGNAL(clicked()),
          this, SLOT(onPushButtonABClicked()));
  connect(confirmPowerShutdownMsgBox_, SIGNAL(finished(int)),
          this, SLOT(onPowerWarningFinished(int)));
  connect(confirmABShutdownMsgBox_, SIGNAL(finished(int)),
          this, SLOT(onABWarningFinished(int)));

  // Initialize service client.
  powerShutdownClient_ = getNodeHandle().serviceClient<std_srvs::Empty>(
      "/rpsm_lpc/shutdown_command", false);
  abShutdownClient_ = nh_.serviceClient<std_srvs::SetBool>(
        "/rpsm_lpc/enable_actuationboard_power", false);
  listDirectoryClient_ = getNodeHandle().serviceClient<rpsm_msgs::ListDirectory>
      ("/rpsm_mainboard/ftp_list_directory", false);

  // Action clients.
  downloadFileActionClient_.reset(
      new DownloadFileActionClient(
          nh_, nh_.param<std::string>("clients/download_file/action",
                                      "/rpsm_mainboard/ftp_download_file"),
          false));

  heartbeatTimer_ = new QTimer(this);
  connect(heartbeatTimer_, SIGNAL(timeout()), this, SLOT(checkForHeartbeat()));
  connect(this, SIGNAL(receiveFirstMainboardHeartbeat()),
          this, SLOT(changeToMainboard()));
  connect(this, SIGNAL(receiveFirstLPCHeartbeat()),
          this, SLOT(changeToLpc()));
  heartbeatTimer_->start(1000);
  lastHeartbeat_.stamp = ros::Time::now();
  heartbeatLpcSubscriber_ = nh_.subscribe("/rpsm_lpc/heartbeat",
                                          10,
                                          &RpsmPlugin::heartbeatLpcCallback,
                                          this);
  heartbeatMainboardSubscriber_ =
      nh_.subscribe("/rpsm_mainboard/heartbeat",
                    10,
                    &RpsmPlugin::heartbeatMainboardCallback,
                    this);
  batteryStateSubscriber_ = nh_.subscribe("/rpsm_lpc/battery_state",
                                          10,
                                          &RpsmPlugin::batteryStateCallback,
                                          this);
  bmsStateSubscriber_ = nh_.subscribe("/rpsm_mainboard/bms_state",
                                      10,
                                      &RpsmPlugin::bmsStateCallback,
                                      this);
  mainBoardStateSubscriber_ = nh_.subscribe("/rpsm_lpc/mainboard_state",
                                            10,
                                            &RpsmPlugin::mainBoardStateCallback,
                                            this);
  actuationBoardStatesSubscriber_ =
      nh_.subscribe("/rpsm_lpc/actuationboard_states",
                    10,
                    &RpsmPlugin::actuationBoardStatesCallback,
                    this);

  humidityStateSubscriber_ = nh_.subscribe("/rpsm_lpc/humidity_state",
                                           10,
                                           &RpsmPlugin::humidityStateCallback,
                                           this);

  // Connect callback signals with slots
  connect(this, SIGNAL(receiveHeartbeat()), this, SLOT(colorHeartbeat()));
  connect(this, SIGNAL(receiveBatteryState()), this, SLOT(fillBatteryData()));
  connect(this, SIGNAL(receiveBmsState()), this, SLOT(fillBmsData()));
  connect(this, SIGNAL(receiveMainboardState()),
          this, SLOT(fillMainboardData()));
  connect(this, SIGNAL(receiveActuationBoardStates()),
          this, SLOT(fillActuationBoardData()));
  connect(this, SIGNAL(receiveHumidityState()), this, SLOT(fillHumidityData()));

  QTableWidgetItem* key;
  QTableWidgetItem* value;

  // Set tableWidget for BatteryState
  std::vector<QString> batteryStateMessageStrings
      = {"Voltage [V]",
         "Current [A]",
         "Charge [Ah]",
         "Capacity [Ah]",
         "Design Capacity [Ah]",
         "Percentage",
         "Power Supply Status",
         "Power Supply Health",
         "Power Supply Technology",
         "Present",
         "Location",
         "Serial Number"};
  int numberOfBatteryStateEntries = batteryStateMessageStrings.size();
  ui_.batteryTable->setRowCount(numberOfBatteryStateEntries);
  for (int i = 0; i < numberOfBatteryStateEntries; i++) {
   key = new QTableWidgetItem;
   key->setText(
         batteryStateMessageStrings[i % batteryStateMessageStrings.size()]);
   ui_.batteryTable->setItem(i, 0, key);
   value = new QTableWidgetItem;
   ui_.batteryTable->setItem(i, 1, value);
  }
  ui_.batteryTable->resizeColumnsToContents();

  // Set tableWidget for MainboardState
  int numberOfConverters = 3;
  std::vector<QString> converterMessageStrings
      = {"Name",
         "Voltage [mV]",
         "Current [mA]",
         "Temperature [°C]",
         "Operational",
         "Enabled"};
  int numberOfConverterEntries = converterMessageStrings.size();
  std::vector<QString> mainBoardMessageStrings = {"E-Stop",
                                                  "FSM State",
                                                  "CAN pwr"};
  ui_.mainBoardTable->setRowCount(
        numberOfConverterEntries + mainBoardMessageStrings.size());
  ui_.mainBoardTable->setColumnCount(numberOfConverters * 2);
  for (int row = 0; row < numberOfConverterEntries; row++) {
    for (int column = 0; column < numberOfConverters; column++) {
      key = new QTableWidgetItem;
       key->setText(converterMessageStrings[row % numberOfConverterEntries]);
       ui_.mainBoardTable->setItem(row, column * 2, key);
       value = new QTableWidgetItem;
       ui_.mainBoardTable->setItem(row, column * 2 + 1, value);
    }
  }
  for (int row = numberOfConverterEntries;
       row < numberOfConverterEntries + mainBoardMessageStrings.size();
       row++) {
    key = new QTableWidgetItem;
    key->setText(mainBoardMessageStrings[row - numberOfConverterEntries]);
    ui_.mainBoardTable->setItem(row, 0, key);
    value = new QTableWidgetItem;
    ui_.mainBoardTable->setItem(row, 1, value);
  }
  ui_.mainBoardTable->resizeColumnsToContents();

  // Set tableWidget for ActuationBoardStates
  int numberOfActuationBoards = 2;
  ui_.actuationBoardTable->setColumnCount(numberOfActuationBoards * 2);
  std::vector<QString> actuationBoardStateMessageStrings
      = {"Node ID",
         "Voltage [mV]",
         "Current [mA]",
         "Temperature [°C]",
         "Operational",
         "Enabled",
         "E-Stop"};
  int numberOfActuationBoardStateEntries
      = actuationBoardStateMessageStrings.size();
  ui_.actuationBoardTable->setRowCount(numberOfActuationBoardStateEntries);
  for (int row = 0; row < numberOfActuationBoardStateEntries; row++) {
    for (int column = 0; column < numberOfActuationBoards; column++) {
      key = new QTableWidgetItem;
      key->setText(
            actuationBoardStateMessageStrings[
              row % actuationBoardStateMessageStrings.size()]);
      ui_.actuationBoardTable->setItem(row, column * 2, key);
      value = new QTableWidgetItem;
      ui_.actuationBoardTable->setItem(row, column * 2 + 1, value);
    }
  }
  ui_.actuationBoardTable->resizeColumnsToContents();

  ui_.toolBox->setItemEnabled(5, false);
  ui_.toolBox->setCurrentIndex(0);
}


void RpsmPlugin::shutdownPlugin() {
  heartbeatLpcSubscriber_.shutdown();
  heartbeatMainboardSubscriber_.shutdown();
  batteryStateSubscriber_.shutdown();
  bmsStateSubscriber_.shutdown();
  mainBoardStateSubscriber_.shutdown();
  actuationBoardStatesSubscriber_.shutdown();
  humidityStateSubscriber_.shutdown();
  powerShutdownClient_.shutdown();
  abShutdownClient_.shutdown();
  listDirectoryClient_.shutdown();
  downloadFileActionClient_.reset();
}

/*****************************************************************************/
/** Settings                                                                **/
/*****************************************************************************/

void RpsmPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                              qt_gui_cpp::Settings& instance_settings) const {
  plugin_settings.setValue("temperature_warning_threshold",
                           ui_.tempWarningSpinBox->value());
  plugin_settings.setValue("show_temperature_warning",
                           ui_.tempWarningCheckBox->isChecked());
}


void RpsmPlugin::restoreSettings(
    const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings) {
  ui_.tempWarningCheckBox->setChecked(
        plugin_settings.value("show_temperature_warning", "true").toBool());
  ui_.tempWarningSpinBox->setValue(
        plugin_settings.value("temperature_warning_threshold", "60").toInt());
}


/*****************************************************************************/
/** Slots                                                                   **/
/*****************************************************************************/

void RpsmPlugin::checkForHeartbeat() {
  if (ros::Time::now() - lastHeartbeat_.stamp >= ros::Duration(1.0)) {
    ui_.heartbeatLabel->setStyleSheet("QLabel { color : red; }");
  }
}

void RpsmPlugin::colorHeartbeat() {
  ui_.heartbeatLabel->setStyleSheet("QLabel { color : green; }");
}


void RpsmPlugin::fillBatteryData() {
  int row = 0;
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBatteryState_.voltage));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBatteryState_.current));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBatteryState_.charge));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBatteryState_.capacity));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBatteryState_.design_capacity));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBatteryState_.percentage));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBatteryState_.power_supply_status));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBatteryState_.power_supply_health));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBatteryState_.power_supply_technology));
  ui_.batteryTable->item(row++, 1)->setText(
        QString(lastBatteryState_.present ? "present" : "not present"));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::fromStdString(lastBatteryState_.location));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::fromStdString(lastBatteryState_.serial_number));
  ui_.batteryTable->setRowCount(row + lastBatteryState_.cell_voltage.size());
  for (auto cellVoltage : lastBatteryState_.cell_voltage) {
    if (!(ui_.batteryTable->item(row, 1))) {
      QTableWidgetItem* key = new QTableWidgetItem;
      key->setText("Cell " + QString::number(row - 12) + " Voltage [V]");
      ui_.batteryTable->setItem(row, 0, key);
      QTableWidgetItem* value = new QTableWidgetItem;
      ui_.batteryTable->setItem(row, 1, value);
    }
    ui_.batteryTable->item(row++, 1)->setText(QString::number(cellVoltage));
  }
  ui_.batteryTable->resizeColumnsToContents();
}


void RpsmPlugin::fillBmsData() {
  int row = 0;
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBmsState_.serialnumber));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBmsState_.temperature));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBmsState_.voltage));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBmsState_.current));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBmsState_.stateofcharge));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBmsState_.batterystatus));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBmsState_.safetystatus));
  ui_.batteryTable->item(row++, 1)->setText(
        QString::number(lastBmsState_.operationstatus));
  ui_.batteryTable->setRowCount(row + lastBmsState_.cellvoltage.size());
  int cellCount = 0;
  for (auto cellVoltage : lastBmsState_.cellvoltage) {
    if (!(ui_.batteryTable->item(row, 1))) {
      QTableWidgetItem* key = new QTableWidgetItem;
      key->setText("Cell " + QString::number(cellCount) + " Voltage [mV]");
      ui_.batteryTable->setItem(row, 0, key);
      QTableWidgetItem* value = new QTableWidgetItem;
      ui_.batteryTable->setItem(row, 1, value);
    }
    ui_.batteryTable->item(row++, 1)->setText(QString::number(cellVoltage));
    cellCount++;
  }
  ui_.batteryTable->resizeColumnsToContents();
}


void RpsmPlugin::fillMainboardData() {
  int row = 0;
  for (int converter = 0;
       converter < std::min<unsigned int>(
         lastMainBoardState_.converter_states.size(), 3);
       converter++) {
    row = 0;
    ui_.mainBoardTable->item(row++, converter * 2 + 1)->setText(
          QString::fromStdString(
            lastMainBoardState_.converter_states.at(converter).name));
    ui_.mainBoardTable->item(row++, converter * 2 + 1)->setText(
          QString::number(
            lastMainBoardState_.converter_states.at(converter).voltage));
    ui_.mainBoardTable->item(row++, converter * 2 + 1)->setText(
          QString::number(
            lastMainBoardState_.converter_states.at(converter).current));
    ui_.mainBoardTable->item(row++, converter * 2 + 1)->setText(
          QString::number(
            lastMainBoardState_.converter_states.at(converter).temperature));
    ui_.mainBoardTable->item(row++, converter * 2 + 1)->setText(
          QString(lastMainBoardState_.converter_states.at(converter).operational
                  ? "yes" : "no"));
    ui_.mainBoardTable->item(row++, converter * 2 + 1)->setText(
          QString(lastMainBoardState_.converter_states.at(converter).enabled
                  ? "yes" : "no"));
    ui_.mainBoardTable->resizeColumnToContents(converter * 2 + 1);
  }
  ui_.mainBoardTable->item(row++, 1)->setText(
        QString(lastMainBoardState_.estop_engaged ? "yes" : "no"));
  ui_.mainBoardTable->item(row++, 1)->setText(
        QString::number(lastMainBoardState_.fsm_state));
  ui_.mainBoardTable->item(row++, 1)->setText(
        QString(lastMainBoardState_.can_power_enabled ? "yes" : "no"));
  ui_.mainBoardTable->resizeColumnToContents(1);
}


void RpsmPlugin::fillActuationBoardData() {
  for (int actuationBoard = 0;
       actuationBoard < std::min<unsigned int>(
         lastActuationBoardStates_.states.size(), 2);
       actuationBoard++) {
    int row = 0;
    ui_.actuationBoardTable->item(row++, actuationBoard * 2 + 1)->setText(
          QString::number(lastActuationBoardStates_.states[actuationBoard]
                          .can_node_id));
    ui_.actuationBoardTable->item(row++, actuationBoard * 2 + 1)->setText(
          QString::number(lastActuationBoardStates_.states[actuationBoard]
                          .state.voltage));
    ui_.actuationBoardTable->item(row++, actuationBoard * 2 + 1)->setText(
          QString::number(lastActuationBoardStates_.states[actuationBoard]
                          .state.current));
    ui_.actuationBoardTable->item(row++, actuationBoard * 2 + 1)->setText(
          QString::number(lastActuationBoardStates_.states[actuationBoard]
                          .state.temperature));
    ui_.actuationBoardTable->item(row++, actuationBoard * 2 + 1)->setText(
          QString(lastActuationBoardStates_.states[actuationBoard].state
                  .operational ? "yes" : "no"));
    ui_.actuationBoardTable->item(row++, actuationBoard * 2 + 1)->setText(
          QString(lastActuationBoardStates_.states[actuationBoard].state
                  .enabled ? "yes" : "no"));
    ui_.actuationBoardTable->item(row++, actuationBoard * 2 + 1)->setText(
          QString(lastActuationBoardStates_.states[actuationBoard]
                  .estop_engaged ? "yes" : "no"));
  }
}


void RpsmPlugin::fillHumidityData() {
  ui_.temperatureValueLabel->setText(QString::number(
                                       lastHumidityState_.ambient_temperature)
                                     + "° C");
  ui_.humidityValueLabel->setText(QString::number(
                                    lastHumidityState_.ambient_humidity)
                                  + "%");

  // Display warning if temperature is too high.
  if (lastHumidityState_.ambient_temperature
      >= 0.8 * ui_.tempWarningSpinBox->value()
      && lastHumidityState_.ambient_temperature
      < ui_.tempWarningSpinBox->value())
  {
    ui_.temperatureValueLabel->setStyleSheet("QLabel { color : orange; }");
  }
  else if (lastHumidityState_.ambient_temperature
           >= ui_.tempWarningSpinBox->value())
  {
    ui_.temperatureValueLabel->setStyleSheet("QLabel { color : red; }");
    if (ui_.tempWarningCheckBox->isChecked() && !shownTemperatureWarning_) {
      shownTemperatureWarning_ = true;
      int ret = QMessageBox::warning(widget_,
                                     tr("High Temperature Alert"),
                 "The internal body temperature has reached critical levels of "
                 + QString::number(lastHumidityState_.ambient_temperature)
                                     + "° C");
    }
  } else {
    ui_.temperatureValueLabel->setStyleSheet("QLabel { color : black; }");
    shownTemperatureWarning_ = false;
  }
}


void RpsmPlugin::changeToMainboard() {
  // Set subscribers to rpsm_mainboard topic
  mainBoardStateSubscriber_ = nh_.subscribe(
        "/rpsm_mainboard/mainboard_state",
        10,
        &RpsmPlugin::mainBoardStateCallback,
        this);
  actuationBoardStatesSubscriber_ = nh_.subscribe(
        "/rpsm_mainboard/actuationboard_states",
        10,
        &RpsmPlugin::actuationBoardStatesCallback,
        this);
  humidityStateSubscriber_ = nh_.subscribe(
        "/rpsm_mainboard/humidity_state",
        10,
        &RpsmPlugin::humidityStateCallback,
        this);

  // Set services to rpsm_mainboard topic
  powerShutdownClient_ = getNodeHandle().serviceClient<std_srvs::Empty>(
      "/rpsm_mainboard/shutdown_command", false);
  abShutdownClient_ = nh_.serviceClient<std_srvs::SetBool>(
        "/rpsm_mainboard/enable_actuationboard_power", false);

  // Set tableWidget for BmsState
  QTableWidgetItem* key;
  QTableWidgetItem* value;
  std::vector<QString> bmsStateMessageStrings
      = {"Serial Number",
         "Temperature [0.1°C]",
         "Voltage [mV]",
         "Current [mA]",
         "State of Charge [%]",
         "Battery Status",
         "Safety Status",
         "Operation Status"};
  bmsStateMessageStrings.insert(bmsStateMessageStrings.end(),
                                lastBmsState_.cellvoltage.size(),
                                "Cell Voltage [mV]");
  int numberOfBmsStateEntries = bmsStateMessageStrings.size();
  ui_.batteryTable->setRowCount(numberOfBmsStateEntries);
  for (int row = 0; row < numberOfBmsStateEntries; row++) {
    key = new QTableWidgetItem;
    key->setText(bmsStateMessageStrings[row]);
    ui_.batteryTable->setItem(row, 0, key);
    value = new QTableWidgetItem;
    ui_.batteryTable->setItem(row, 1, value);
  }
  ui_.batteryTable->resizeColumnsToContents();

  // Show toolbox entry: Logs
  ui_.toolBox->setItemEnabled(5, true);
  rpsm_msgs::ListDirectoryRequest request;
  request.absolute_file_path = "/";
  workerThreadListDirectory_ = new WorkerThreadListDirectory;
  connect(workerThreadListDirectory_,
          SIGNAL(result(bool, rpsm_msgs::ListDirectoryResponse)),
          this, SLOT(
            onListDirectoryResult(bool, rpsm_msgs::ListDirectoryResponse)));
  connect(workerThreadListDirectory_, SIGNAL(finished()),
          workerThreadListDirectory_, SLOT(deleteLater()));
  workerThreadListDirectory_->setClient(listDirectoryClient_);
  workerThreadListDirectory_->setRequest(request);
  workerThreadListDirectory_->start();
  connect(ui_.treePushButton, SIGNAL(clicked()),
          this, SLOT(onTreePushButtonClicked()));
  connect(this, SIGNAL(receiveDownloadFileFeedback()),
          this, SLOT(updateDownloadProgress()));
  setLogToolboxState(LogToolboxState::treeView);
}


void RpsmPlugin::changeToLpc() {
  // Set subscribers to rpsm_lpc topic
  mainBoardStateSubscriber_ = nh_.subscribe(
        "/rpsm_lpc/mainboard_state",
        10,
        &RpsmPlugin::mainBoardStateCallback,
        this);
  actuationBoardStatesSubscriber_ = nh_.subscribe(
        "/rpsm_lpc/actuationboard_states",
        10,
        &RpsmPlugin::actuationBoardStatesCallback,
        this);
  humidityStateSubscriber_ = nh_.subscribe(
        "/rpsm_lpc/humidity_state",
        10,
        &RpsmPlugin::humidityStateCallback,
        this);

  // Set services to rpsm_lpc topic
  powerShutdownClient_ = getNodeHandle().serviceClient<std_srvs::Empty>(
      "/rpsm_lpc/shutdown_command", false);
  abShutdownClient_ = nh_.serviceClient<std_srvs::SetBool>(
        "/rpsm_lpc/enable_actuationboard_power", false);

  // Set tableWidget for BatteryState
  QTableWidgetItem* key;
  QTableWidgetItem* value;
  std::vector<QString> batteryStateMessageStrings
      = {"Voltage [V]",
         "Current [A]",
         "Charge [Ah]",
         "Capacity [Ah]",
         "Design Capacity [Ah]",
         "Percentage",
         "Power Supply Status",
         "Power Supply Health",
         "Power Supply Technology",
         "Present",
         "Location",
         "Serial Number"};
  batteryStateMessageStrings.insert(batteryStateMessageStrings.end(),
                                    lastBatteryState_.cell_voltage.size(),
                                    "Cell Voltage [V]");
  int numberOfBatteryStateEntries = batteryStateMessageStrings.size();
  ui_.batteryTable->setRowCount(numberOfBatteryStateEntries);
  for (int row = 0; row < numberOfBatteryStateEntries; row++) {
    key = new QTableWidgetItem;
    key->setText(batteryStateMessageStrings[row]);
    ui_.batteryTable->setItem(row, 0, key);
    value = new QTableWidgetItem;
    ui_.batteryTable->setItem(row, 1, value);
  }
  ui_.batteryTable->resizeColumnsToContents();

  // Hide Logs pannel
  ui_.toolBox->setItemEnabled(5, false);
}


void RpsmPlugin::updateDownloadProgress() {
  ui_.downloadProgressBar->setValue(100.0 * downloadFileFeedback_.progress);
}


void RpsmPlugin::onPushButtonShutdownClicked() {
  confirmPowerShutdownMsgBox_->show();
}


void RpsmPlugin::onPushButtonABClicked() {
  if (ui_.actuationBoardPushButton->isChecked()) {
    confirmABShutdownMsgBox_->setWindowTitle("Please confirm leg shutdown");
    confirmABShutdownMsgBox_->setText(
          "Make sure that the robot is lying on the ground or suspended! "
          "Legs will collaps when shutting down actuation boards.");
    confirmABShutdownMsgBox_->show();
  }
  else {
    confirmABShutdownMsgBox_->setWindowTitle("Please confirm leg power-on");
    confirmABShutdownMsgBox_->setText(
          "Powering on actuation boards. Stand back!");
    confirmABShutdownMsgBox_->show();
  }
}


void RpsmPlugin::onPowerWarningFinished(int rc) {
  if (rc == QDialogButtonBox::Ok) {
    ROS_INFO("[rqt_RPSM] Shutting down RPSM power!");
    workerThreadShutdown_ = new WorkerThreadShutdown;
    connect(workerThreadShutdown_, SIGNAL(result(bool)),
            this, SLOT(onPowerShutdownResult(bool)));
    connect(workerThreadShutdown_, SIGNAL(finished()),
            workerThreadShutdown_, SLOT(deleteLater()));
    workerThreadShutdown_->setClient(powerShutdownClient_);
    workerThreadShutdown_->setRequest(std_srvs::EmptyRequest());
    workerThreadShutdown_->start();
  }
}


void RpsmPlugin::onABWarningFinished(int rc) {
  if (rc == QDialogButtonBox::Ok) {
    workerThreadAB_ = new WorkerThreadAB;
    connect(workerThreadAB_, SIGNAL(result(bool)),
            this, SLOT(onABShutdownResult(bool)));
    connect(workerThreadAB_, SIGNAL(finished()),
            workerThreadAB_, SLOT(deleteLater()));
    workerThreadAB_->setClient(abShutdownClient_);
    if (ui_.actuationBoardPushButton->isChecked()) {
      std_srvs::SetBoolRequest request;
      request.data = false;
      workerThreadAB_->setRequest(request);
      ROS_INFO("[rqt_RPSM] Shutting down Actuation Boards!");
      workerThreadAB_->start();
    }
    else {
      ROS_INFO("[rqt_RPSM] Powering on Actuation Boards!");
      std_srvs::SetBoolRequest request;
      request.data = true;
      workerThreadAB_->setRequest(request);
      workerThreadAB_->start();
    }
  }
  else {
    if (ui_.actuationBoardPushButton->isChecked()) {
      ui_.actuationBoardPushButton->setChecked(false);
    }
    else {
      ui_.actuationBoardPushButton->setChecked(true);
    }
  }
}


void RpsmPlugin::onPowerShutdownResult(bool isOk) {
  if (!isOk) {
    ROS_WARN("[rqt_RPSM] Power shutdown service call failed!");
  }
}


void RpsmPlugin::onABShutdownResult(bool isOk) {
  if (!isOk) {
    ROS_WARN("[rqt_RPSM] Actuation board service call failed!");
    ui_.actuationBoardPushButton->toggle();
  }
}


void RpsmPlugin::onTreeWidgetActivated(QTreeWidgetItem* item, int column) {
  if (static_cast<FileTreeItem*>(item)->isFile()) {
    if (!downloadFileActionClient_->waitForServer(ros::Duration(1.0))) {
      ROS_WARN_STREAM("[rqt_RPSM] onTreeWidgetActivated(): "
                      "downloadFile action server has not been advertised.");
      return;
    }
    rpsm_msgs::DownloadFileGoal goal;
    goal.from_filepath = static_cast<FileTreeItem*>(item)->getFilePath();
    if (specifyDownloadFilePath(&(goal.to_filepath))) {
      downloadFileActionClient_
          ->sendGoal(goal,
                     boost::bind(&RpsmPlugin::downloadFileResultCallback,
                                 this, _1, _2),
                     boost::bind(&RpsmPlugin::downloadFileActiveCallback,
                                 this),
                     boost::bind(&RpsmPlugin::downloadFileFeedbackCallback,
                                 this, _1));
    }
  }
  else {
    rpsm_msgs::ListDirectoryRequest request;
    request.absolute_file_path =
        static_cast<FileTreeItem*>(item)->getFilePath();
    workerThreadListDirectory_ = new WorkerThreadListDirectory;
    connect(workerThreadListDirectory_,
            SIGNAL(result(bool, rpsm_msgs::ListDirectoryResponse)),
            this, SLOT(
              onListDirectoryResult(bool, rpsm_msgs::ListDirectoryResponse)));
    connect(workerThreadListDirectory_, SIGNAL(finished()),
            workerThreadListDirectory_, SLOT(deleteLater()));
    workerThreadListDirectory_->setClient(listDirectoryClient_);
    workerThreadListDirectory_->setRequest(request);
    workerThreadListDirectory_->start();
//    ROS_INFO_STREAM("call ftp_list_directory service.2");
  }
}


void RpsmPlugin::onTreeWidgetExpanded() {
//  if (ui_.treeWidget->columnWidth(0) )
//  ui_.treeWidget->resizeColumnToContents(0);
//  ui_.treeWidget->setColumnWidth(0, ui_.treeWidget->columnWidth(0)+20);
}


void RpsmPlugin::onListDirectoryResult(
    bool isOk, rpsm_msgs::ListDirectoryResponse response)
{
  if (!isOk) {
    ROS_WARN("[rqt_RPSM] List directory service call failed!");
    return;
  }

  if (ui_.treeWidget->currentItem()
      && static_cast<FileTreeItem*>(ui_.treeWidget->currentItem())->isFile())
  {
    ROS_WARN("[rqt_RPSM] Received directory list from unknown origin.");
    return;
  }
  FileTreeItem* currentItem
      = static_cast<FileTreeItem*>(ui_.treeWidget->currentItem());
  if (!currentItem) {
    FileTreeItem* topEntry = new FileTreeItem(ui_.treeWidget, false, "/");
    topEntry->setText(0, "/");
    ui_.treeWidget->setCurrentItem(topEntry);
    currentItem = topEntry;
  }
  currentItem->takeChildren();
  for (int i = 0; i < response.name.size(); i++) {
    if (!response.name[i].empty()) {
      FileTreeItem* entry
          = new FileTreeItem(currentItem,
                             response.size[i] != 0,
                             response.name[i]);
      entry->setText(0, QString::fromStdString(response.name[i]));
      if (response.size[i] != 0) {
        entry->setText(1, QString::number(response.size[i]));
      }
    }
  }
  currentItem->setExpanded(true);
  ui_.treeWidget->resizeColumnToContents(1);
}


void RpsmPlugin::onTreePushButtonClicked() {
  if (ui_.treeWidget->currentItem()) {
    onTreeWidgetActivated(ui_.treeWidget->currentItem(), 0);
  }
}


void RpsmPlugin::onDownloadCancelPushButtonClicked() {
  downloadFileActionClient_->cancelGoal();
  setLogToolboxState(LogToolboxState::treeView);
}


void RpsmPlugin::onDumpBackPushButtonClicked() {
  setLogToolboxState(LogToolboxState::treeView);
}


void RpsmPlugin::onSaveAsCsvPushButtonClicked() {
//    QFileDialog dialog(widget_, "Dump file to ...");
//    dialog.setDirectory(QDir::homePath());
//    dialog.setFileMode(QFileDialog::AnyFile);
//    dialog.setNameFilter(tr("CSV files (*.csv)"));
//    dialog.selectFile("log.csv");
//    dialog.setViewMode(QFileDialog::Detail);
//    if (!dialog.exec()) {
//      setLogToolboxState(LogToolboxState::treeView);
//      return;
//    }
//    std::string filePath = dialog.selectedFiles().at(0).toStdString();

//    getNodeHandle().param<std::string>("password", pw_, "");
//    if (pw_.empty()) {
//      QInputDialog pwDialog(widget_);
//      pwDialog.setInputMode(QInputDialog::TextInput);
//      pwDialog.setTextEchoMode(QLineEdit::EchoMode::Password);
//      if (!pwDialog.exec()) {
//        setLogToolboxState(LogToolboxState::treeView);
//        return;
//      }
//      pw_ = pwDialog.textValue().toStdString();
//    }
//    std::string command = "sshpass -p '";
//    command.append(pw_);
//    command.append("' scp ");
//    command.append(" integration@anymal-beth-lpc:~/rpsm_log/");
//    command.append(filePath);
//    command.append(".yaml");

//    if (std::system(command.c_str()) == 0)
//    {
//      ROS_INFO("File copied.");
//    }
//    else
//    {
//      ROS_ERROR_STREAM("Could not copy file with command: " << command);
//    }

  ROS_WARN("[rqt_RPSM] Not yet implemented.");
}


/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

void RpsmPlugin::heartbeatLpcCallback(const rpsm_msgs::Heartbeat& msg) {
  if (receivedMainboardInsteadOfLpc_) {
    emit receiveFirstLPCHeartbeat();
    receivedMainboardInsteadOfLpc_ = false;
  }
  lastHeartbeat_ = msg;
  emit receiveHeartbeat();
}


void RpsmPlugin::heartbeatMainboardCallback(const rpsm_msgs::Heartbeat& msg) {
  if (!receivedMainboardInsteadOfLpc_) {
    emit receiveFirstMainboardHeartbeat();
    receivedMainboardInsteadOfLpc_ = true;
  }
  lastHeartbeat_ = msg;
  emit receiveHeartbeat();
}


void RpsmPlugin::batteryStateCallback(const sensor_msgs::BatteryState& msg) {
  lastBatteryState_ = msg;
  emit receiveBatteryState();
}


void RpsmPlugin::bmsStateCallback(const rpsm_msgs::BmsState& msg) {
  lastBmsState_ = msg;
  emit receiveBmsState();
}


void RpsmPlugin::mainBoardStateCallback(const rpsm_msgs::MainBoardState &msg) {
  lastMainBoardState_ = msg;
  emit receiveMainboardState();
}


void RpsmPlugin::actuationBoardStatesCallback(
    const rpsm_msgs::ActuationBoardStates &msg)
{
  lastActuationBoardStates_ = msg;
  emit receiveActuationBoardStates();
}


void RpsmPlugin::humidityStateCallback(
    const rpsm_msgs::HumidityState &msg)
{
  lastHumidityState_ = msg;
  emit receiveHumidityState();
}


void RpsmPlugin::downloadFileActiveCallback() {
  ui_.downloadProgressBar->setValue(0);
  setLogToolboxState(LogToolboxState::downloading);
}


void RpsmPlugin::downloadFileFeedbackCallback(
    const rpsm_msgs::DownloadFileFeedbackConstPtr& feedback)
{
  downloadFileFeedback_ = *feedback;
  emit receiveDownloadFileFeedback();
}


void RpsmPlugin::downloadFileResultCallback(
    const actionlib::SimpleClientGoalState& state,
    const rpsm_msgs::DownloadFileResultConstPtr& result)
{
  if (result->success){
    setLogToolboxState(LogToolboxState::dump);
  } else {
    setLogToolboxState(LogToolboxState::treeView);
  }
}


bool RpsmPlugin::specifyDownloadFilePath(std::string* filePath) {
  QInputDialog dialog(widget_);
  dialog.setInputMode(QInputDialog::TextInput);
  dialog.setTextValue("/home/integration/rpsm_logs/log.px4log");

//  QFileDialog dialog(widget_, "Download file to ...");
//  dialog.setDirectory(QString::fromStdString(*filePath));
//  dialog.setFileMode(QFileDialog::AnyFile);
//  dialog.setNameFilter(tr("Log files (*.px4log)"));
//  dialog.selectFile("log.px4log");
//  dialog.setViewMode(QFileDialog::Detail);

  if (!dialog.exec())
    return false;
  *filePath = dialog.textValue().toStdString();
  //  *filePath = dialog.selectedFiles().at(0).toStdString();

  return true;
}


void RpsmPlugin::setLogToolboxState(LogToolboxState state) {
  switch (state)
  {
  case LogToolboxState::treeView:
  {
    ui_.treeWidget->show();
    ui_.treeWidget->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    ui_.treeWidget->header()->setSectionResizeMode(1, QHeaderView::Interactive);

    ui_.treePushButton->show();
    ui_.treeLabel->setText("select file to ");
    ui_.downloadWidget->hide();
    ui_.dumpWidget->hide();
    connect(ui_.treeWidget, SIGNAL(itemActivated(QTreeWidgetItem*,int)),
            this, SLOT(onTreeWidgetActivated(QTreeWidgetItem*, int)));
    connect(ui_.treeWidget, SIGNAL(itemExpanded(QTreeWidgetItem*)),
            this, SLOT(onTreeWidgetExpanded()));
    break;
  }

  case LogToolboxState::downloading:
  {
    ui_.treeWidget->hide();
    ui_.treePushButton->hide();
    ui_.treeLabel->setText("downloading "
                           + ui_.treeWidget->currentItem()->text(0));
    ui_.downloadWidget->show();
    ui_.dumpWidget->hide();
    connect(ui_.downloadPushButton, SIGNAL(clicked(bool)),
            this, SLOT(onDownloadCancelPushButtonClicked()));
    break;
  }

  case LogToolboxState::dump:
  {
    ui_.treeWidget->hide();
    ui_.treePushButton->hide();
    ui_.treeLabel->setText("downloaded " +
                           ui_.treeWidget->currentItem()->text(0) +
                           " to LPC.");
    ui_.downloadWidget->hide();
    ui_.dumpWidget->show();
    connect(ui_.dumpBackPushButton, SIGNAL(clicked(bool)),
            this, SLOT(onDumpBackPushButtonClicked()));
    connect(ui_.dumpSaveAsCsvPushButton, SIGNAL(clicked(bool)),
            this, SLOT(onSaveAsCsvPushButtonClicked()));
    break;
  }
  }
}


} // namespace rqt_rpsm

PLUGINLIB_EXPORT_CLASS(rqt_rpsm::RpsmPlugin, rqt_gui_cpp::Plugin)
