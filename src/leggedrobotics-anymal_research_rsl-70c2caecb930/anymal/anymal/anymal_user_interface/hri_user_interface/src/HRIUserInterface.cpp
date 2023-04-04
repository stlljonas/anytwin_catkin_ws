/*
 * HRIUserInterface.cpp
 *
 *  Created on: 21 May 2015
 *      Author: markuszahner
 */

#include "hri_user_interface/HRIUserInterface.h"

#include "hri_user_interface/AnymalMotionControlForm.h"
#include "hri_user_interface/ServiceForm.h"
#include "hri_user_interface/EmergencyForm.h"

namespace hri_user_interface {

HRIUserInterface::HRIUserInterface(any_node::Node::NodeHandlePtr nh) :
  any_node::Node(nh),
  sliderLeft_(0.0),
  sliderRight_(0.0),
  firstRemoteConnection_(true),
  isReceivingJoyMessages_(false),
  hardEmergencyStop_(false),
  notificationLevel_(0),
  notificationVibration_(0) {
  // set up JoystickLines, could be customized
  forms_.push_back(FormPtr(new AnymalMotionControlForm(this)));
  forms_.push_back(FormPtr(new ServiceForm(this)));
  emergencyForm_ = FormPtr(new EmergencyForm(this));
  lastFormIt_ = forms_.begin();
  currentForm_ = *lastFormIt_;
}

bool HRIUserInterface::init() {
  bool success = true;
  // set Button callback function
  buttons_.init(boost::bind(&HRIUserInterface::handleJoystickButtonPressed, this, _1,_2));

  // notification
  notificationSubscriberPtr_.reset(new notification::NotificationSubscriber("joystick_screen",getNodeHandle(),0,20,boost::bind(&HRIUserInterface::notificationCallback,this,_1)));
  statusLineTimer_ = getNodeHandle().createTimer(ros::Duration(1.0/kStatusLineUpdateFrequency), &HRIUserInterface::updateStatusLine, this);


  // service clients
  joySetTextOutputClient_ =  any_node::serviceClient<hri_safety_sense::KeyString>(getNodeHandle(), "hri_key_string", "/hri_safety_sense/key_string");
  joySetKeyValueClient_ = any_node::serviceClient<hri_safety_sense::KeyValue>(getNodeHandle(), "hri_key_value", "/hri_safety_sense/key_value");

  // subscribers
  hriJoySubscriber_ = any_node::subscribe(getNodeHandle(), "hri_joy", "/joy/onboard", 10, &HRIUserInterface::hriJoystickCallback, this);
  hriEmergencyStopSubscriber_ = any_node::subscribe(getNodeHandle(), "hri_emergency_stop","/hri_safety_sense/emergency_stop",10,&HRIUserInterface::hriEmergencyStopCallback, this);
  controllerManagerStateSubscriber_ = any_node::subscribe(getNodeHandle(), "controller_manager_state", "/anymal_highlevel_controller/notify_controller_manager_state", 10, &HRIUserInterface::controllerManagerStateCallback, this);
  anydriveReadingsSubscriber_ = any_node::subscribe(getNodeHandle(), "anydrive_readings", "/anymal_lowlevel_controller/actuator_readings_extended_throttled", 10, &HRIUserInterface::anydriveCallback, this);
  batteryStateSubscriber_ = any_node::subscribe(getNodeHandle(), "battery_state", "/rpsm_lpc/battery_state", 10, &HRIUserInterface::batteryCallback, this);
  anymalStateSubscriber_ = any_node::subscribe(getNodeHandle(), "anymal_state", "/state_estimator/anymal_state_throttle", 10, &HRIUserInterface::anymalStateCallback, this);

  // publishers
  anyJoyPublisher_ = any_node::advertise<joy_manager_msgs::AnyJoy>(getNodeHandle(), "anyJoy", "/anyjoy/onboard",10,false);

  // set up the joystick menu top line
  success = success && forms_[0]->init(getNodeHandle());
  success = success && forms_[1]->init(getNodeHandle());
  success = success && emergencyForm_->init(getNodeHandle());

  success = success && loadParameters();

  anyJoyMsg_.joy.axes.resize(kNumberOfAxes, 0.0);
  anyJoyMsg_.joy.buttons.resize(kNumberOfButtons, 0.0);
  anyJoyMsg_.joy.header.stamp = ros::Time::now();
  hriJoy_ = anyJoyMsg_.joy;
  currentCommands_.clear();

  double timeOutDouble, spinrate;
  success = success && getNodeHandle().param<double>("timeout", timeOutDouble, 5.0);
  success = success && getNodeHandle().param<double>("spinrate", spinrate, 50.0);
  timeOut_ = ros::Duration(timeOutDouble);

  any_worker::WorkerOptions workerOptions;
  workerOptions.name_ = "publish_anyJoy";
  workerOptions.callback_ = boost::bind(&HRIUserInterface::publishJoystick, this, _1);
  workerOptions.timeStep_ = 1.0/spinrate;
  workerOptions.defaultPriority_ = 1;
  if (!addWorker(workerOptions)) {
    ROS_ERROR_STREAM("Could not add worker: " << workerOptions.name_);
    success = false;
  }

  return success;
}


void HRIUserInterface::cleanup(){
  hriJoySubscriber_.shutdown();
  hriEmergencyStopSubscriber_.shutdown();
  controllerManagerStateSubscriber_.shutdown();
  anydriveReadingsSubscriber_.shutdown();
  batteryStateSubscriber_.shutdown();
  anymalStateSubscriber_.shutdown();
  notificationSubscriberPtr_->shutdown();
  joySetTextOutputClient_.shutdown();
  joySetKeyValueClient_.shutdown();
  anyJoyPublisher_.shutdown();
}


void HRIUserInterface::notificationCallback(const notification::Notification & msg) {
  ROS_INFO_STREAM("Notification: " << msg.name_);
  if (msg.getLevelAsUnsignedChar() >= notificationLevel_) {
    // our callback
    notification_ = msg.name_;
    notificationAge_ = ros::Duration(0.0);
  }
  if (msg.getLevelAsUnsignedChar()>=notificationVibration_) {
    // severe failure
    setRemoteKey(VSC_USER_BOTH_MOTOR_INTENSITY,MOTOR_CONTROL_INTENSITY_HIGH);
  }
}


void HRIUserInterface::hriJoystickCallback(const sensor_msgs::Joy::ConstPtr & msg) {
  if ((int)msg->axes.size() < kNumberOfAxes || (int)msg->buttons.size() < kNumberOfButtons) {
    ROS_ERROR("Nonstandard number of joystick axes or buttons!");
    ros::shutdown();
  }
  {
    std::lock_guard<std::mutex> connectionLock(connectionMutex_);
    isReceivingJoyMessages_ = true;
    if (firstRemoteConnection_) {
      firstRemoteConnection_ = false;

      // disable pause features
      setRemoteKey(VSC_USER_ORIENTATION_PAUSE_ENABLE, 0);
      setRemoteKey(VSC_USER_FREEFALL_PAUSE_ENABLE, 0);
      setRemoteKey(VSC_USER_INACTIVITY_PAUSE_ENABLE, 0);

      setRemoteKey(VSC_USER_DISPLAY_MODE,DISPLAY_MODE_CUSTOM_TEXT);
      setRemoteString(VSC_USER_DISPLAY_ROW_1,"");
      setRemoteString(VSC_USER_DISPLAY_ROW_2,"");
      setRemoteString(VSC_USER_DISPLAY_ROW_3,"");
      setRemoteString(VSC_USER_DISPLAY_ROW_4,"");
      updateJoystickScreen();

      hriJoy_.header.stamp = ros::Time::now();
      sleep(1); // give the joystick time to update
    }
  }
  // create bits from buttons
  uint pushbuttons=0;
  int j;
  for (j = 0;j<std::min(32,(int)msg->buttons.size());j++) {
    pushbuttons |= (msg->buttons[j])<<j;
  }
  pushbuttons |= (msg->axes[5]!=0)<<j; // this is entry 9 (used to set the parameter values)
  sliderLeft_ = msg->axes[2];
  sliderRight_ = msg->axes[5];
  // feed the buttons, and its callback
  buttons_.update(pushbuttons);
  // forward joystick values to the topic
  {
    boost::unique_lock<boost::shared_mutex> lock(hriJoyMutex_);
    hriJoy_.header.stamp = ros::Time::now();
    hriJoy_.axes = msg->axes;
    hriJoy_.axes[0] = -hriJoy_.axes[0];
    hriJoy_.axes[3] = -hriJoy_.axes[3];
    hriJoy_.axes[2] = 0.0;
    hriJoy_.axes[5] = 0.0;

    hriJoy_.buttons[0] = msg->buttons[4]; // stand (A)
    hriJoy_.buttons[1] = msg->buttons[5]; // walk (B)
    hriJoy_.buttons[2] = msg->buttons[7]; // trot (X)
    hriJoy_.buttons[3] = msg->buttons[6]; // external control (Y)

    for (size_t i=0;i<hriJoy_.axes.size();i++) {
      hriJoy_.axes[i] /= 1024; // rescale between -1,1
    }
  } // lock
}


bool HRIUserInterface::publishJoystick(const any_worker::WorkerEvent& timerEvent) {
  if (anyJoyPublisher_.getNumSubscribers() > 0u) {
    {
      boost::shared_lock<boost::shared_mutex> lockOnboard(hriJoyMutex_);
      anyJoyMsg_.joy = hriJoy_;
    } // lock
    {
      std::lock_guard<std::mutex> lock(currentCommandsMutex_);
      anyJoyMsg_.commands.clear();
      anyJoyMsg_.modules.clear();
      for (std::pair<std::string, std::string> command : currentCommands_) {
        anyJoyMsg_.commands.push_back(command.first);
        anyJoyMsg_.modules.push_back(command.second);
      }
      currentCommands_.clear();
    } // lock
    anyJoyMsg_.header.stamp = ros::Time::now();
    std::lock_guard<std::mutex> connectionLock(connectionMutex_);
    joy_manager_msgs::AnyJoyConstPtr anyJoyMsg(new joy_manager_msgs::AnyJoy(anyJoyMsg_));
    anyJoyPublisher_.publish(anyJoyMsg);
  }
  return true;
}


void HRIUserInterface::setCommand(const std::string& module, const std::string& command) {
  std::lock_guard<std::mutex> lock(currentCommandsMutex_);
  if (currentCommands_.find(command) == currentCommands_.end())
    ROS_DEBUG_STREAM("[HRI] " << command << " --> " << module);
    currentCommands_.insert(std::pair<std::string, std::string>(command, module));
}


void HRIUserInterface::anydriveCallback(const anydrive_msgs::ReadingsExtended::ConstPtr & msg) {
  double maxTemperature = std::numeric_limits<double>::min();
  for (const auto& reading : msg->readings) {
    if (maxTemperature < reading.state.temperature) {
      maxTemperature = reading.state.temperature;
    }
  }
  maxDriveTemperature_ = maxTemperature;
}


void HRIUserInterface::batteryCallback(const sensor_msgs::BatteryState::ConstPtr & msg) {
  batteryVoltage_ = msg->voltage;
  batteryChargingState_ = msg->power_supply_status;
}


void HRIUserInterface::anymalStateCallback(const anymal_msgs::AnymalState::ConstPtr & msg) {
  stateEstimatorState_ = msg->state;
}


void HRIUserInterface::hriEmergencyStopCallback(const hri_safety_sense::EstopStatus::ConstPtr & msg) {
  if (msg->EstopStatus != hri_safety_sense::EstopStatus::ESTOP_REASON_CLEAR) {
    if (!hardEmergencyStop_) {
      ROS_WARN("[HRIUserInterface::hriEmergencyStopCallback] Hard emergency stop pressed.");
      hardEmergencyStop_ = true;
    }
/*
    // Forward the hard emergency stop to the system. TODO: Enable again when false positives have been fixed!
    anyJoyMsg_.header.stamp = ros::Time::now();
    anyJoyMsg_.commands = {"hard_emcy_stop"};
    joy_manager_msgs::AnyJoyConstPtr anyJoyMsg(new joy_manager_msgs::AnyJoy(anyJoyMsg_));
    anyJoyPublisher_.publish(anyJoyMsg);
*/
  }
  else if (hardEmergencyStop_) {
      ROS_INFO("[HRIUserInterface::hriEmergencyStopCallback] Hard emergency stop released.");
      hardEmergencyStop_ = false;
  }
}


void HRIUserInterface::controllerManagerStateCallback(const rocoma_msgs::ControllerManagerStateConstPtr& msg) {
  if (msg->estop_cleared && currentForm_ == emergencyForm_) {
    currentForm_ = *lastFormIt_;
  }
  else if (!msg->estop_cleared && currentForm_ != emergencyForm_) {
    lastFormIt_ = std::find(forms_.begin(), forms_.end(), currentForm_);
    if (lastFormIt_ == forms_.end()) {
      ROS_WARN_STREAM("[HRIUserInterface::controllerManagerStateCallback] Form out of bound!");
      currentForm_ = *lastFormIt_;
    }
    currentForm_ = emergencyForm_;
  }
  updateJoystickScreen();
}


void HRIUserInterface::setRemoteKey(int key,int value) {
  hri_safety_sense::KeyValue key_msg;// set remote screen to mode

  key_msg.request.Key = key;
  key_msg.request.Value = value;

  if (joySetKeyValueClient_.call(key_msg)) {
//     ROS_INFO("Set Remote key: %i to mode: %i\n",key,value);
  }
}


void HRIUserInterface::setRemoteString(int key,std::string str) {
  hri_safety_sense::KeyString key_msg;// set remote screen to mode

  key_msg.request.Key = key;
  key_msg.request.Value = str;

  if (joySetTextOutputClient_.call(key_msg)) {
    //ROS_INFO("Set Remote key:%i to %s \n",key,str.c_str());
  }
}


void HRIUserInterface::updateJoystickScreen() {
  // if possible, keep 1 line above and below cursor
  std::vector<std::string> screenLines = currentForm_->print(kNumberScreenLines);

  for (int i = 0; i < kNumberScreenLines; i++) {
    setRemoteString(VSC_USER_DISPLAY_ROW_2+i, screenLines[i]);
  }
}


void HRIUserInterface::handleJoystickButtonPressed(int button, int action) {
  // Handle buttons regardless of the currently active form.
  if (button == BUTTON_3) {
    if (currentCommands_.find("soft_emcy_stop") == currentCommands_.end()) {
      currentCommands_.insert(std::pair<std::string, std::string>("soft_emcy_stop", ""));
    }
    return;
  }

  int factor = 10;
  if (sliderLeft_ > 200) {
    factor = 100;
  }
  else if (sliderLeft_<-200) {
    factor = 1;
  }
  int value = sliderRight_/100;

  // Handle buttons in currently active form.
  int deltaFormIndex = currentForm_->handleButtons(button, value * factor);
  // If emergencyForm is the current form we do not change forms.
  if (currentForm_ != emergencyForm_) {
    // Search for the currently active form.
    auto currentFormIt = std::find(forms_.begin(), forms_.end(), currentForm_);
    // If we reach boundaries of the forms_ vector then stay at the current form.
    if ((currentFormIt == forms_.begin() && deltaFormIndex == -1) ||
        (currentFormIt == forms_.end()-1 && deltaFormIndex == 1)) {
      currentForm_ = *currentFormIt;
    }
    else if (currentFormIt == forms_.end()) {
      ROS_WARN_STREAM("[HRIUserInterface::handleJoystickButtonPressed] Form out of bound!");
      currentForm_ = *lastFormIt_;
    }
    else {
      currentForm_ = *(currentFormIt + deltaFormIndex);
    }
  }
  updateJoystickScreen();

}


bool HRIUserInterface::loadParameters() {
  bool success = getNodeHandle().param<int>("notification/level/screen", notificationLevel_, 1);
  success = success && getNodeHandle().param<int>("notification/level/vibration", notificationVibration_, 3);
  return success;
}

void HRIUserInterface::updateStatusLine(const ros::TimerEvent& event) {
  if (notification_.empty()) {
    std::stringstream lineStream;
    lineStream << std::setprecision(1) << std::fixed;
    lineStream << "SE " << (stateEstimatorState_ < 0 ? ":(  " : ":)  ");
    lineStream << maxDriveTemperature_ << "C  ";
    lineStream << batteryVoltage_ << "V" << (batteryChargingState_ == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING ? "+" : "-");
    statusLine_ = lineStream.str();
  }
  else {
    statusLine_ = notification_;
    if (notificationAge_ <= ros::Duration(kNotificationLifespan)) {
      notificationAge_ += ros::Duration(1.0 / kStatusLineUpdateFrequency);
    } else {
      notification_.clear();
    }
  }
  setRemoteKey(VSC_USER_ORIENTATION_PAUSE_ENABLE, 0);
  setRemoteKey(VSC_USER_FREEFALL_PAUSE_ENABLE, 0);
  setRemoteKey(VSC_USER_INACTIVITY_PAUSE_ENABLE, 0);
  setRemoteKey(VSC_USER_DISPLAY_MODE,DISPLAY_MODE_CUSTOM_TEXT);
  setRemoteString(VSC_USER_DISPLAY_ROW_1, statusLine_);
}


} // namespace hri_user_interface
