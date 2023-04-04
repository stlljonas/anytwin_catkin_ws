#ifndef icp_plugin_H
#define icp_plugin_H

// rqt gui
#include <rqt_gui_cpp/plugin.h>
#include <ui_icp_plugin.h>

// qt
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QObject>
#include <QSignalMapper>
#include <QTimer>
#include <QWidget>

// ros
#include <ros/ros.h>
#include <ros/service_client.h>

// icp tools node state
#include <slam_common_msgs/NodeState.h>

class IcpPlugin : public rqt_gui_cpp::Plugin {
  Q_OBJECT
 public:
  IcpPlugin();
  void initPlugin(qt_gui_cpp::PluginContext& context) override;
  void shutdownPlugin() override;
  void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
  void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

 protected slots:
  void startLocalization(int idx);
  void stopLocalization(int idx);
  void startPublishPose(int idx);
  void stopPublishPose(int idx);
  void startMapping(int idx);
  void stopMapping(int idx);
  void saveMapToFile(int idx);
  void loadMapFromFile(int idx);
  void clearMap(int idx);
  void publishMap(int idx);
  void readConfigFiles(int idx);

  void refreshMapFile(int idx);

  void updateStateVisualization(int idx);

 signals:
  void nodeStateChanged(int idx);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
 private:
  std::vector<Ui::my_plugin*> uis_;
  QTabWidget* tabwidget_ = nullptr;
  std::vector<QWidget*> widgets_;
  std::vector<QSignalMapper*> signalMappers_;
  std::vector<std::string*> mapOnLaunchFiles_;

  std::vector<ros::ServiceClient> toggleLocalizationClients_;
  std::vector<ros::ServiceClient> togglePublishPoseClients_;
  std::vector<ros::ServiceClient> toggleMappingClients_;
  std::vector<ros::ServiceClient> saveMapToFileClients_;
  std::vector<ros::ServiceClient> loadMapFromFileClients_;
  std::vector<ros::ServiceClient> clearMapClients_;
  std::vector<ros::ServiceClient> publishMapClients_;
  std::vector<ros::ServiceClient> readConfigFilesClients_;

  std::vector<ros::Subscriber> nodeStateSubscribers_;
  std::vector<slam_common_msgs::NodeState> nodeStateMsgs_;

  QPixmap pixmapOk_;
  QPixmap pixmapError_;

  void nodeStateCallback(const slam_common_msgs::NodeState::ConstPtr& stateMsg, int idx);
  bool checkRqtIcpConfigParams(std::string tab_key, XmlRpc::XmlRpcValue values);
};
#endif  // icp_plugin_H
