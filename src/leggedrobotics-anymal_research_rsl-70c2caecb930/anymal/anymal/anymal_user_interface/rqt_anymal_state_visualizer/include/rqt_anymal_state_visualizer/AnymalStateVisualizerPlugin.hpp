/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#pragma once

// intern
#include "rqt_anymal_state_visualizer/DiagnosticAnymalWidget.h"
// QT
#include <QWidget>
#include <QStringList>
#include <QGridLayout>
#include <QTimer>
#include <QPushButton>
#include <QTableWidgetItem>
#include <QLayoutItem>
#include <QResizeEvent>
#include <rqt_anymal_state_visualizer/ui_anymal_state_visualizer_plugin.h>
// ros
#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <notification/NotificationSubscriber.hpp>
// msgs
#ifdef ANYDRIVE1X
#include <series_elastic_actuator_msgs/SeActuatorCommands.h>
#include <series_elastic_actuator_msgs/SeActuatorReadingsExtended.h>
#include <series_elastic_actuator_msgs/SeActuatorStates.h>
#else
#include <anydrive_msgs/Commands.h>
#include <anydrive_msgs/ReadingsExtended.h>
#include <anydrive_msgs/States.h>
#endif
#include <anymal_msgs/AnymalState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <std_srvs/Trigger.h>
// std
#include <string>
#include <mutex>
#include <atomic>
// boost
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/local_time_adjustor.hpp>
#include <boost/date_time/c_local_time_adjustor.hpp>

namespace rqt_anymal_state_visualizer {

class AnymalStateVisualizerPlugin : public rqt_gui_cpp::Plugin {
Q_OBJECT
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  AnymalStateVisualizerPlugin();

  virtual ~AnymalStateVisualizerPlugin();

  /* ======================================================================== */
  /* Initialize/Shutdown                                                      */
  /* ======================================================================== */

  virtual void initPlugin(qt_gui_cpp::PluginContext &context);

  virtual void shutdownPlugin();

  /* ======================================================================== */
  /* Settings                                                                 */
  /* ======================================================================== */

  virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                            qt_gui_cpp::Settings &instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                               const qt_gui_cpp::Settings &instance_settings);

private:

  /* ======================================================================== */
  /* Constants                                                                */
  /* ======================================================================== */

  const std::string TAG = "AnymalStateVisualizerPlugin";

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  Ui::AnymalStateVisualizer ui_;
  QWidget *widget_;

  DiagnosticAnymalWidget *diagnosticAnymalWidget_;
  std::vector<DiagnosticComponentBase *> selectedComponents_;
  std::vector<DiagnosticComponentBase *> componentsLast_;
  int numberOfColumnsLast_ = 0;

  // Subscribers
  ros::Subscriber anymalStateSubscriber_;
  ros::Subscriber actuatorReadingSubscriber_;
  ros::Subscriber diagnosticSubscriber_;
  ros::Subscriber imuSubscriber_;
  ros::Subscriber batteryStateSubscriber_;

  // Notifications
  notification::NotificationSubscriber *notificationSubscriber_;

  // Notification table
  QTimer *timer_;
  std::atomic<bool> notificationTableFlag_;

  // Mutex
  std::mutex mutex_;

  /* ======================================================================== */
  /* Callbacks                                                                */
  /* ======================================================================== */

  void anymalStateCallback(const anymal_msgs::AnymalState &msg);

  void actuatorReadingsCallback(
      const ActuatorReadings &readings);

  void notificationCallback(const notification::Notification &notification);

  void diagnosticCallback(
      const diagnostic_msgs::DiagnosticArrayConstPtr &status);

  void imuCallback(const sensor_msgs::ImuConstPtr &msg);

  void batteryStateCallback(const sensor_msgs::BatteryStateConstPtr &msg);

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  /**
   * @brief Convert UTC time to local time.
   * @param t
   * @return
   */
  boost::posix_time::ptime local_ptime_from_utc_time_t(std::time_t const t);

  /**
   * @brief Generate new grid layout.
   */
  void generateGridLayout();

  void cleanGridLayout();

  /**
   * @brief Sort a vector and return the permutation, vector is not sorted.
   * @tparam T
   * @tparam Compare
   * @param vec
   * @param compare
   * @return
   */
  template<typename T, typename Compare>
  std::vector<int> sort_permutation(
      std::vector<T> const &vec,
      Compare compare) {
    std::vector<int> p(vec.size());
    std::iota(p.begin(), p.end(), 0);
    std::sort(p.begin(), p.end(),
              [&](int i, int j) { return compare(vec[i], vec[j]); });
    return p;
  }

  /**
   * @brief Apply a permutation to a vector.
   * @tparam T
   * @param vec
   * @param p
   * @return
   */
  template<typename T>
  std::vector<T> apply_permutation(
      std::vector<T> const &vec,
      std::vector<int> const &p) {
    std::vector<T> sorted_vec(p.size());
    std::transform(p.begin(), p.end(), sorted_vec.begin(),
                   [&](int i) { return vec[i]; });
    return sorted_vec;
  }

  /**
   * @brief Helper function. Removes all layout items within the given @a layout
   * which either span the given @a row or @a column. If @a deleteWidgets
   * is true, all concerned child widgets become not only removed from the
   * layout, but also deleted.
   * http://stackoverflow.com/questions/5395266/removing-widgets-from-qgridlayout/19256990#19256990
   * modified to detach widgets
   * @param layout
   * @param row
   * @param column
   * @param deleteWidgets
   */
  void remove(QGridLayout *layout, int row, int column, bool deleteWidgets);

  /**
   * @brief Helper function. Deletes all child widgets of the given layout
   * @a item.
   * http://stackoverflow.com/questions/5395266/removing-widgets-from-qgridlayout/19256990#19256990
   * @param item
   */
  void deleteChildWidgets(QLayoutItem *item);

  /**
   * @brief Helper function. Detaches all child widgets of the given layout @a
   * item by setting parent to 0.
   * @param item
   */
  void detachChildWidgets(QLayoutItem *item);

  /**
   * @brief Removes all layout items on the given @a row from the given grid
   * @a layout. If @a deleteWidgets is true, all concerned child widgets
   * become not only removed from the layout, but also deleted. Note that
   * this function doesn't actually remove the row itself from the grid
   * layout, as this isn't possible (i.e. the rowCount() and row indices
   * will stay the same after this function has been called).
   * http://stackoverflow.com/questions/5395266/removing-widgets-from-qgridlayout/19256990#19256990
   * @param layout
   * @param row
   * @param deleteWidgets
   */
  void removeRow(QGridLayout *layout, int row, bool deleteWidgets);

  /**
   * @brief Removes all layout items on the given @a column from the given grid
   * @a layout. If @a deleteWidgets is true, all concerned child widgets
   * become not only removed from the layout, but also deleted. Note that
   * this function doesn't actually remove the column itself from the grid
   * layout, as this isn't possible (i.e. the columnCount() and column
   * indices will stay the same after this function has been called).
   * http://stackoverflow.com/questions/5395266/removing-widgets-from-qgridlayout/19256990#19256990
   * @param layout
   * @param column
   * @param deleteWidgets
   */
  void removeColumn(QGridLayout *layout, int column, bool deleteWidgets);

  /* ======================================================================== */
  /* Events                                                                   */
  /* ======================================================================== */

  /**
   * @brief Listen to widget resizing. Regenerate grid layout.
   * @param object
   * @param event
   * @return
   */
  bool eventFilter(QObject *object, QEvent *event);

protected slots:

  /* ======================================================================== */
  /* Slots                                                                    */
  /* ======================================================================== */

  void updateNotificationsTable();

  void updateNotificationsTableFlag();

  void updateSelectedComponents(
      std::vector<DiagnosticComponentBase *> components);
};

} // namespace
