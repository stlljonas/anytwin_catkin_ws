// pluginlib
#include <pluginlib/class_list_macros.h>

// param io
#include <param_io/get_param.hpp>

// anymal lowlevel controller common
#include <anymal_lowlevel_controller_common/state_machine/StateEnum.hpp>

// rqt anymal lowlevel controller
#include "rqt_anymal_lowlevel_controller/AnymalLowLevelControllerPlugin.hpp"


namespace rqt_anymal_lowlevel_controller {

using namespace anymal_lowlevel_controller_common::state_machine;


AnymalLowLevelControllerPlugin::AnymalLowLevelControllerPlugin()
: rqt_gui_cpp::Plugin()
{
  setObjectName("AnymalLowLevelControllerPlugin");
}

void AnymalLowLevelControllerPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  activeStateSubscriber_ = getNodeHandle().subscribe(
      param_io::param<std::string>(getNodeHandle(), "subscribers/active_state/topic", "/default"),
      param_io::param<int>(getNodeHandle(), "subscribers/active_state/queue_size", 1),
      &AnymalLowLevelControllerPlugin::activeStateCb, this);

  goToStateClient_ = getNodeHandle().serviceClient<anymal_msgs::AnymalLowLevelControllerGoToState>(
      param_io::param<std::string>(getNodeHandle(), "clients/go_to_state/service", "/default"),
      param_io::param<bool>(getNodeHandle(), "clients/go_to_state/persistent", 1));

  connect(this, SIGNAL(activeStateChanged()), this, SLOT(updateActiveState()));

  signalMapper_ = new QSignalMapper(this);
  connect(ui_.pushButtonEnable, SIGNAL(clicked()), signalMapper_, SLOT(map()));
  connect(ui_.pushButtonDisable, SIGNAL(clicked()), signalMapper_, SLOT(map()));
  connect(ui_.pushButtonClearErrors, SIGNAL(clicked()), signalMapper_, SLOT(map()));
  connect(ui_.pushButtonWarmReset, SIGNAL(clicked()), signalMapper_, SLOT(map()));
  connect(ui_.pushButtonIdle, SIGNAL(clicked()), signalMapper_, SLOT(map()));
  connect(ui_.pushButtonOperational, SIGNAL(clicked()), signalMapper_, SLOT(map()));
  connect(ui_.pushButtonFatal, SIGNAL(clicked()), signalMapper_, SLOT(map()));
  connect(ui_.pushButtonZeroTorque, SIGNAL(clicked()), signalMapper_, SLOT(map()));
  signalMapper_->setMapping(ui_.pushButtonEnable, static_cast<int>(StateEnum::ActionActuatorsEnable));
  signalMapper_->setMapping(ui_.pushButtonDisable, static_cast<int>(StateEnum::ActionActuatorsDisable));
  signalMapper_->setMapping(ui_.pushButtonClearErrors, static_cast<int>(StateEnum::ActionActuatorsClearErrors));
  signalMapper_->setMapping(ui_.pushButtonWarmReset, static_cast<int>(StateEnum::ActionActuatorsWarmReset));
  signalMapper_->setMapping(ui_.pushButtonIdle, static_cast<int>(StateEnum::StateIdle));
  signalMapper_->setMapping(ui_.pushButtonOperational, static_cast<int>(StateEnum::StateOperational));
  signalMapper_->setMapping(ui_.pushButtonFatal, static_cast<int>(StateEnum::StateFatal));
  signalMapper_->setMapping(ui_.pushButtonZeroTorque, static_cast<int>(StateEnum::StateZeroJointTorque));
  connect(signalMapper_, SIGNAL(mapped(int)), this, SLOT(goToStateCb(int)));
}

void AnymalLowLevelControllerPlugin::shutdownPlugin()
{
  activeStateSubscriber_.shutdown();
  goToStateClient_.shutdown();
}

void AnymalLowLevelControllerPlugin::saveSettings(
    qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{

}

void AnymalLowLevelControllerPlugin::restoreSettings(
    const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{

}

void AnymalLowLevelControllerPlugin::activeStateCb(const anymal_msgs::AnymalLowLevelControllerStateConstPtr& state)
{
  std::lock_guard<std::mutex> lock(activeStateMutex_);
  activeState_ = *state;
  emit activeStateChanged();
}

void AnymalLowLevelControllerPlugin::updateActiveState()
{
  std::lock_guard<std::mutex> lock(activeStateMutex_);
  ui_.activeStateLabel->setText(QString::fromStdString(stateMsgToName(activeState_)));
}

void AnymalLowLevelControllerPlugin::goToStateCb(int stateId)
{
  anymal_msgs::AnymalLowLevelControllerGoToState goToState;
  goToState.request.state = stateEnumToMsg(static_cast<StateEnum>(stateId));
  if (!goToStateClient_.exists())
  {
    ROS_ERROR_STREAM("Service server '" << goToStateClient_.getService() << "' does not exist.");
    return;
  }
  if (!goToStateClient_.call(goToState))
  {
    ROS_ERROR_STREAM("Service server '" << goToStateClient_.getService() << "' returned false.");
    return;
  }
}


} // rqt_anymal_lowlevel_controller

PLUGINLIB_EXPORT_CLASS(rqt_anymal_lowlevel_controller::AnymalLowLevelControllerPlugin, rqt_gui_cpp::Plugin)
