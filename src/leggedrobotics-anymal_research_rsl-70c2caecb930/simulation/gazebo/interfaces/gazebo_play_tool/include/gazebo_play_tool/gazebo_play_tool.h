#ifndef GAZEBO_PLAY_TOOL_H
#define GAZEBO_PLAY_TOOL_H

#include <rviz/tool.h>

#include <rviz/properties/tf_frame_property.h>

#include <ros/ros.h>

namespace gazebo_play_tool
{

// CenterFrameTool exposes a button that centers the camera on a specifed frame
// via the view_manager Focal Point property. 

class GazeboPlayTool: public rviz::Tool
{

public:
  // Constructor
  GazeboPlayTool();
  virtual ~GazeboPlayTool();

protected:
  //This is called when the Tool is created, ie when added to the Tool bar
  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

private:

  ros::ServiceClient gazeboSrvPause_;
  ros::ServiceClient gazeboSrvPlay_;

  enum ButtonState{
    kPause,
    kPlay,
    kBrokenPause,
    kBrokenPlay
  } button_state_;

  QIcon pause_icon_;
  QIcon play_icon_;
  QIcon broken_pause_icon_;
  QIcon broken_play_icon_;

  void tryPlay();
  void tryPause();

};
// END_TUTORIAL

} //gazebo_play_tool

#endif // PLANT_FLAG_TOOL_H