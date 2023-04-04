/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include <pluginlib/class_list_macros.h>
#include "rqt_anymal_state_visualizer/AnymalStateVisualizerPlugin.hpp"

namespace rqt_anymal_state_visualizer {

const int anymalStateWidgetWidth = 239;
const int anymalStateWidgetHeight = 306;

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

AnymalStateVisualizerPlugin::AnymalStateVisualizerPlugin()
    : rqt_gui_cpp::Plugin(),
      widget_(nullptr),
      diagnosticAnymalWidget_(nullptr) {
  // give QObjects reasonable names
  setObjectName("AnymalStateVisualizerPlugin");

  qRegisterMetaType<message_t>("message_t");
  qRegisterMetaType<sensor_msgs::BatteryStateConstPtr>
      ("sensor_msgs::BatteryStateConstPtr");
  qRegisterMetaType<std_srvs::TriggerResponse>
      ("std_srvs::TriggerResponse");
  qRegisterMetaType<QVector<int> >("QVector<int>");
}

AnymalStateVisualizerPlugin::~AnymalStateVisualizerPlugin() {
}

/* ========================================================================== */
/* Initialize                                                                 */
/* ========================================================================== */

void AnymalStateVisualizerPlugin::initPlugin(
    qt_gui_cpp::PluginContext &context) {
  // Access standalone command line arguments.
  QStringList argv = context.argv();

  // Create the main widget, set it up and add it to the user interface.
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  context.addWidget(widget_);

  // get background color
//  QColor color = widget_->palette().color(widget_->backgroundRole());
//  ROS_INFO_STREAM("red: " << color.red());
//  ROS_INFO_STREAM("blue: " << color.blue());
//  ROS_INFO_STREAM("green: " << color.green());

  // add widget to container
  diagnosticAnymalWidget_ = new DiagnosticAnymalWidget(widget_);
  diagnosticAnymalWidget_->setFixedSize(anymalStateWidgetWidth,
                                           anymalStateWidgetHeight);
  ui_.RobotContainer->addWidget(diagnosticAnymalWidget_);

  // setup ros subscribers
  anymalStateSubscriber_ = getNodeHandle().subscribe(
      "/state_estimator/anymal_state_throttle", 10,
      &AnymalStateVisualizerPlugin::anymalStateCallback, this);
  actuatorReadingSubscriber_ = getNodeHandle().subscribe(
      "/anymal_lowlevel_controller/actuator_readings_extended_throttled", 10,
      &AnymalStateVisualizerPlugin::actuatorReadingsCallback, this);
//  subDiagnostic_ = getNodeHandle().subscribe(
//      "/diagnostics_op", 15,
//      &AnymalStateVisualizerPlugin::diagnosticCallback, this);
  // subscribe to imu

  // Check if a RPSM or a PDB is present
  std::string batteryTopicName = "/pdb/battery_state_ros";
  std::vector<std::string> nodeNames;
  ros::master::getNodes(nodeNames);
  if (std::find(nodeNames.cbegin(), nodeNames.cend(), "/rpsm_lpc") != nodeNames.cend()){
    // RPSM exists.
    batteryTopicName = "/rpsm_lpc/battery_state";
  } else if (std::find(nodeNames.cbegin(), nodeNames.cend(), "/pdb_driver") != nodeNames.cend()){
    // PDB exists. And topic name is set by default.
  } else {
    ROS_INFO_STREAM("No hardware node publishing the battery state found. Will try to subscribe to '" << batteryTopicName << "'." );
  }

  batteryStateSubscriber_ = getNodeHandle().subscribe(
      batteryTopicName, 10,
      &AnymalStateVisualizerPlugin::batteryStateCallback, this);

  // setup notification subscriber parameters
  getNodeHandle().setParam(
      "notification/output_devices/robot_visualizer/topic",
      "/notification_robot_state");
  getNodeHandle().setParam(
      "notification/output_devices/robot_visualizer/queue_size", 20);
  // setup notification subscriber
  notificationSubscriber_ = new notification::NotificationSubscriber
      ("robot_visualizer", getNodeHandle(),
       boost::bind(&AnymalStateVisualizerPlugin::notificationCallback,
                   this, _1));

  // connect
  connect(diagnosticAnymalWidget_, SIGNAL(updateNotificationsTableFlag()),
          this, SLOT(updateNotificationsTableFlag()));
  connect(diagnosticAnymalWidget_, SIGNAL(updateSelectedComponents(
      std::vector<DiagnosticComponentBase *>)),
          this, SLOT(updateSelectedComponents(
      std::vector<DiagnosticComponentBase *>)));

  // timer to update notification table
  notificationTableFlag_ = true;
  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(updateNotificationsTable()));
  timer_->start(100); // 10Hz (100ms)

  // install event filter
  ui_.widget_scroll_area->installEventFilter(this);

  // init battery
  //ui_.BatteryWidgetContainer->initWidget(
  //    getNodeHandle(), "/rpsm_lpc/battery_state");
  ui_.BatteryWidgetContainer->hide();
}

void AnymalStateVisualizerPlugin::shutdownPlugin() {
  // unregister all publishers here
  timer_->stop();
  anymalStateSubscriber_.shutdown();
  actuatorReadingSubscriber_.shutdown();
  diagnosticSubscriber_.shutdown();
  batteryStateSubscriber_.shutdown();
  notificationSubscriber_->shutdown();
}

/* ========================================================================== */
/* Settings                                                                   */
/* ========================================================================== */

void AnymalStateVisualizerPlugin::saveSettings(
    qt_gui_cpp::Settings &plugin_settings,
    qt_gui_cpp::Settings &instance_settings) const {
}

void AnymalStateVisualizerPlugin::restoreSettings(
    const qt_gui_cpp::Settings &plugin_settings,
    const qt_gui_cpp::Settings &instance_settings) {
  ROS_INFO_STREAM_NAMED(TAG, TAG << " restoreSettings()");
}

/* ========================================================================== */
/* Callbacks                                                                  */
/* ========================================================================== */

void AnymalStateVisualizerPlugin::anymalStateCallback(
    const anymal_msgs::AnymalState &msg) {
//    std::lock_guard<std::mutex> lock_guard(mutex_);

  if (msg.header.stamp.toSec() != 0) {
    diagnosticAnymalWidget_->setAnymalStateMsg(msg);
  }
}

void AnymalStateVisualizerPlugin::actuatorReadingsCallback(
    const ActuatorReadings &readings) {
  diagnosticAnymalWidget_->setActuatorReadings(readings);
}

void AnymalStateVisualizerPlugin::notificationCallback(
    const notification::Notification &notification) {
  std::lock_guard<std::mutex> lock_guard(mutex_);

  // update notification in diagnosticAnymalWidget
  diagnosticAnymalWidget_->setNotification(notification);
}

void AnymalStateVisualizerPlugin::diagnosticCallback(
    const diagnostic_msgs::DiagnosticArrayConstPtr &status) {
  std::lock_guard<std::mutex> lock_guard(mutex_);

  // update diagnostic in diagnosticAnymalWidget
  for (auto item : status->status) {
    diagnosticAnymalWidget_->setDiagnostic(item);
  }
}

void AnymalStateVisualizerPlugin::imuCallback(
    const sensor_msgs::ImuConstPtr &msg) {
  diagnosticAnymalWidget_->setImu(msg);
}

void AnymalStateVisualizerPlugin::batteryStateCallback(
    const sensor_msgs::BatteryStateConstPtr &msg) {
  diagnosticAnymalWidget_->setBatteryState(msg);
}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */

boost::posix_time::ptime
AnymalStateVisualizerPlugin::local_ptime_from_utc_time_t(
    std::time_t const t) {
  return boost::date_time::c_local_adjustor<
      boost::posix_time::ptime>::utc_to_local(
      boost::posix_time::from_time_t(t));
}

void AnymalStateVisualizerPlugin::remove(QGridLayout *layout, int row,
                                            int column, bool deleteWidgets) {
  // We avoid usage of QGridLayout::itemAtPosition() here
  // to improve performance.
  for (int i = layout->count() - 1; i >= 0; i--) {
    int r, c, rs, cs;
    layout->getItemPosition(i, &r, &c, &rs, &cs);
    if ((r <= row && r + rs - 1 >= row) ||
        (c <= column && c + cs - 1 >= column)) {
      // This layout item is subject to deletion.
      QLayoutItem *item = layout->takeAt(i);
      if (deleteWidgets) {
        deleteChildWidgets(item);
      } else {
        detachChildWidgets(item);
      }
      delete item;
    }
  }
}

void AnymalStateVisualizerPlugin::deleteChildWidgets(QLayoutItem *item) {
  if (item->layout()) {
    // Process all child items recursively.
    for (int i = 0; i < item->layout()->count(); i++) {
      deleteChildWidgets(item->layout()->itemAt(i));
    }
  }
  delete item->widget();
}


void AnymalStateVisualizerPlugin::detachChildWidgets(QLayoutItem *item) {
  if (item->layout()) {
    // Process all child items recursively.
    for (int i = 0; i < item->layout()->count(); i++) {
      detachChildWidgets(item->layout()->itemAt(i));
    }
  } else {
    item->widget()->setParent(0);
  }
}

void AnymalStateVisualizerPlugin::removeRow(QGridLayout *layout,
                                               int row, bool deleteWidgets) {
  remove(layout, row, -1, deleteWidgets);
  layout->setRowMinimumHeight(row, 0);
  layout->setRowStretch(row, 0);
}

void AnymalStateVisualizerPlugin::removeColumn(QGridLayout *layout,
                                                  int column,
                                                  bool deleteWidgets) {
  remove(layout, -1, column, deleteWidgets);
  layout->setColumnMinimumWidth(column, 0);
  layout->setColumnStretch(column, 0);
}

void AnymalStateVisualizerPlugin::cleanGridLayout() {
  // Remove all widgets from grid layout.
  if (ui_.layout_scroll_area->rowCount() > 0) {
    for (int i = 0; i < ui_.layout_scroll_area->rowCount(); ++i) {
      removeRow(ui_.layout_scroll_area, i, false);
    }
    ui_.layout_scroll_area->invalidate();
  }
}

void AnymalStateVisualizerPlugin::generateGridLayout() {
  int width = ui_.widget_scroll_area->size().width();

  // Get components with initialized table model.
  std::vector<DiagnosticComponentBase *> components;
  for (auto item : selectedComponents_) {
    if (item->getTableModel() == NULL) {
      continue;
    }
    components.push_back(item);
  }

  int numberOfComponents = (int)components.size();

  bool isNew = false;
  if (numberOfComponents == componentsLast_.size()) {
    for (int i = 0; i < numberOfComponents; ++i) {
      if (components[i] != componentsLast_[i]) {
        isNew = true;
      }
    }
  } else {
    isNew = true;
  }

  if (components.empty()) {
    cleanGridLayout();
    numberOfColumnsLast_ = 0;
  } else {
    int minWidth = 0;
    int minHeight = 0;
    for (auto item : components) {
      if (item->getDataTable()->minimumWidth() > minWidth) {
        minWidth = item->getDataTable()->minimumWidth();
      }
      if (item->getDataTable()->minimumHeight() > minHeight) {
        minHeight = item->getDataTable()->minimumHeight();
      }
    }

    int numberOfColumns = 1;
    while (true) {
      // TODO 4 ???
      if (numberOfColumns * minWidth +
          (numberOfColumns - 1) * ui_.layout_scroll_area->horizontalSpacing() +
          4 < width) {
        numberOfColumns++;
      } else {
        numberOfColumns--;
        break;
      }
      if (numberOfColumns == numberOfComponents &&
          numberOfColumns * minWidth +
          (numberOfColumns - 1) * ui_.layout_scroll_area->horizontalSpacing() +
          4 < width) {
        break;
      }
    }
    if (numberOfColumns < 1) {
      numberOfColumns = 1;
    }

    // Check if to clean the grid layout.
    if (numberOfColumns != numberOfColumnsLast_ || isNew) {
      cleanGridLayout();
    }

    // If new or the number of columns changed.
    if (numberOfColumns != numberOfColumnsLast_ || isNew) {
      // add widgets to the vertical layout and finally the vertical
      // layout to the grid layout
      int row = 0;
      int counter = 0;
      while (numberOfComponents > 0) {
        for (int i = 0; i < numberOfColumns; ++i) {
          if (numberOfComponents > 0) {
            // add widgets to new vertical layout
            QVBoxLayout *boxLayout = new QVBoxLayout(0);
            boxLayout->addWidget(components[counter]->getTitleLabel());
            boxLayout->addWidget(components[counter]->getDataTable());
            // add vertical layout finally to grid layout
            ui_.layout_scroll_area->addLayout(boxLayout, row, i);
            counter++;
            numberOfComponents--;
          }
        }
        row++;
      }
    }
    // Set current number of columns as last.
    numberOfColumnsLast_ = numberOfColumns;
  }

  componentsLast_ = components;
}

/* ========================================================================== */
/* Slots                                                                      */
/* ========================================================================== */

void AnymalStateVisualizerPlugin::updateNotificationsTable() {
  if (notificationTableFlag_) {
    return;
  }

  std::lock_guard<std::mutex> lock_guard(mutex_);

  if (selectedComponents_.size() == 0) {
    ui_.table_notifications->setRowCount(0);
    ui_.table_notifications->update();
    for (auto item : diagnosticAnymalWidget_->getMessages()) {
      boost::posix_time::ptime ptime = local_ptime_from_utc_time_t(
          item.time.toSec());
      std::string timestamp = boost::posix_time::to_simple_string(ptime);
      int rowCount = ui_.table_notifications->rowCount();
      ui_.table_notifications->insertRow(rowCount);
      ui_.table_notifications->update();
      ui_.table_notifications->setItem(
          rowCount, 0, new QTableWidgetItem(QString::fromStdString(timestamp)));
      switch (item.level) {
        case OK:
          ui_.table_notifications->item(rowCount, 0)->setForeground(
              diagnosticAnymalWidget_->colors_.green);
          break;
        case WARN:
          ui_.table_notifications->item(rowCount, 0)->setForeground(
              diagnosticAnymalWidget_->colors_.warning);
          break;
        case ERROR:
          ui_.table_notifications->item(rowCount, 0)->setForeground(
              diagnosticAnymalWidget_->colors_.error);
          break;
        default:
          break;
      }
      ui_.table_notifications->setItem(rowCount, 1,
                                       new QTableWidgetItem(item.id));
      ui_.table_notifications->setItem(rowCount, 2,
                                       new QTableWidgetItem(item.message));
    }
  } else {
    std::vector<message_t> messages;
    for (auto item : selectedComponents_) {
      for (auto message : item->getMessages()) {
        messages.push_back(message);
      }
//      std::copy(item->getMessages().begin(), item->getMessages().end(),
//                std::back_inserter(messages));
//      messages.insert(messages.end(), item->getMessages().begin(),
//                      item->getMessages().end());
    }
    auto p = sort_permutation(messages, [](
        message_t const &a, message_t const &b) { return a.time > b.time; });
    messages = apply_permutation(messages, p);

    ui_.table_notifications->setRowCount(0);
    ui_.table_notifications->update();
    for (auto item : messages) {
      boost::posix_time::ptime ptime = local_ptime_from_utc_time_t(
          item.time.toSec());
      std::string timestamp = boost::posix_time::to_simple_string(ptime);
      int rowCount = ui_.table_notifications->rowCount();
      ui_.table_notifications->insertRow(rowCount);
      ui_.table_notifications->update();
      ui_.table_notifications->setItem(rowCount, 0, new QTableWidgetItem(
          QString::fromStdString(timestamp)));
      switch (item.level) {
        case OK:
          ui_.table_notifications->item(rowCount, 0)->setForeground(
              diagnosticAnymalWidget_->colors_.green);
          break;
        case WARN:
          ui_.table_notifications->item(rowCount, 0)->setForeground(
              diagnosticAnymalWidget_->colors_.warning);
          break;
        case ERROR:
          ui_.table_notifications->item(rowCount, 0)->setForeground(
              diagnosticAnymalWidget_->colors_.error);
          break;
        default:
          break;
      }
      ui_.table_notifications->setItem(
          rowCount, 1, new QTableWidgetItem(item.id));
      ui_.table_notifications->setItem(
          rowCount, 2, new QTableWidgetItem(item.message));
    }
  }
  ui_.table_notifications->resizeColumnsToContents();
  ui_.table_notifications->resizeRowsToContents();

  notificationTableFlag_ = true;
}

void AnymalStateVisualizerPlugin::updateSelectedComponents(
    std::vector<DiagnosticComponentBase *> components) {
//    std::lock_guard<std::mutex> lock_guard(mutex_);

  selectedComponents_ = components;
  // manage subscribers
  bool hasImu = false;
  for (auto item : components) {
    if (typeid(*item) == typeid(DiagnosticComponentImu)) {
      hasImu = true;
      if (!imuSubscriber_) {
        // subscribe to imu
        imuSubscriber_ = getNodeHandle().subscribe(
            "/sensors/imu_throttle", 10,
            &AnymalStateVisualizerPlugin::imuCallback, this);
        break;
      }
    }
  }
  if (!hasImu && imuSubscriber_) {
    // shutdown imu subscriber
    imuSubscriber_.shutdown();
  }
  // update grid layout
  generateGridLayout();
}

void AnymalStateVisualizerPlugin::updateNotificationsTableFlag() {
  notificationTableFlag_ = false;
}

/* ========================================================================== */
/* Events                                                                     */
/* ========================================================================== */

bool AnymalStateVisualizerPlugin::eventFilter(QObject *object,
                                                 QEvent *event) {
//    std::lock_guard<std::mutex> lock_guard(mutex_);

  if (event->type() == QEvent::Resize) {
    if (object == ui_.widget_scroll_area) {
      generateGridLayout();
    }
  }
  return QObject::eventFilter(object, event);
}

PLUGINLIB_EXPORT_CLASS(AnymalStateVisualizerPlugin, rqt_gui_cpp::Plugin)

} // namespace
