#include <gazebo_play_tool/gazebo_play_tool.h>

#include <ros/console.h>

#include <rviz/display_context.h>
#include <rviz/load_resource.h>
#include <rviz/tool_manager.h>

#include <std_srvs/Empty.h>

    namespace gazebo_play_tool {

  GazeboPlayTool::GazeboPlayTool() {
    // In case this is unclear, the shortcut is the spacebar.
    shortcut_key_ = ' ';
  }

  GazeboPlayTool::~GazeboPlayTool() {}

  void GazeboPlayTool::onInitialize() {
    setName("Gazebo");

    ros::NodeHandle nh_;

    gazeboSrvPause_ =
        nh_.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    gazeboSrvPlay_ =
        nh_.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    play_icon_ =
        rviz::loadPixmap("package://gazebo_play_tool/icons/classes/Play.png");
    pause_icon_ =
        rviz::loadPixmap("package://gazebo_play_tool/icons/classes/Pause.png");
    broken_play_icon_ = rviz::loadPixmap(
        "package://gazebo_play_tool/icons/classes/BrokenPlay.png");
    broken_pause_icon_ = rviz::loadPixmap(
        "package://gazebo_play_tool/icons/classes/BrokenPause.png");

    button_state_ = kPause;
    setIcon(play_icon_);
    setIcon(pause_icon_);
  }

  void GazeboPlayTool::activate() {
    setIcon(play_icon_);

    switch (button_state_) {
      case ButtonState::kPause:
        tryPause();
        break;

      case ButtonState::kPlay:
        tryPlay();
        break;

      case ButtonState::kBrokenPause:
        tryPause();
        break;

      case ButtonState::kBrokenPlay:
        tryPlay();
        break;

      default:
        break;
    }

  // This may be a hack, but it changes the selected tool to the default tool.
    context_->getToolManager()->setCurrentTool(
        context_->getToolManager()->getDefaultTool());
  }

  void GazeboPlayTool::tryPause() {
    std_srvs::Empty emptyService;

    if (gazeboSrvPause_.call(emptyService)) {
      button_state_ = ButtonState::kPlay;
      setIcon(play_icon_);
    } else {
      button_state_ = ButtonState::kBrokenPause;
      setIcon(broken_pause_icon_);
      ROS_ERROR("Failed to call gazebo unpause_physics");
    }
    context_->getToolManager()->refreshTool(this);
  }

  void GazeboPlayTool::tryPlay() {
    std_srvs::Empty emptyService;

    if (gazeboSrvPlay_.call(emptyService)) {
      button_state_ = ButtonState::kPause;
      setIcon(pause_icon_);
    } else {
      button_state_ = ButtonState::kBrokenPlay;
      setIcon(broken_play_icon_);
      ROS_ERROR("Failed to call gazebo pause_physics");
    }
    context_->getToolManager()->refreshTool(this);
  }

  void GazeboPlayTool::deactivate() {}

}  // gazebo_play_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gazebo_play_tool::GazeboPlayTool, rviz::Tool)