#ifndef CENTER_FRAME_TOOL_H
#define CENTER_FRAME_TOOL_H

#include <rviz/tool.h>

#include <rviz/properties/tf_frame_property.h>


namespace rviz
{
class ViewportMouseEvent;
} //rviz

namespace center_frame_tool
{

// Here we declare a subclass of rviz::Tool.
// These can be added in rviz via the Tools bar
//
// CenterFrameTool exposes a button that centers the camera on a specifed frame
// via the view_manager Focal Point property. 

class CenterFrameTool: public rviz::Tool
{

public:
  // Constructor
  CenterFrameTool();
  virtual ~CenterFrameTool();

protected:
  //This is called when the Tool is created, ie when added to the Tool bar
  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

private:

  //The desired frame to center on, in rviz Propery form
  rviz::TfFrameProperty* frame_property_;

};
// END_TUTORIAL

} //center_frame_tool

#endif // PLANT_FLAG_TOOL_H