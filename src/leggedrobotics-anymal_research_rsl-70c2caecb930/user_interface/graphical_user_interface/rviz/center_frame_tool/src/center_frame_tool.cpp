#include <center_frame_tool/center_frame_tool.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/tool_manager.h>
#include <rviz/display_context.h>
#include <rviz/view_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/vector_property.h>

namespace center_frame_tool
{

CenterFrameTool::CenterFrameTool()
{
  shortcut_key_ = 'p';
}


CenterFrameTool::~CenterFrameTool()
{

}

void CenterFrameTool::onInitialize()
{

  setName( "Center Frame" );

  frame_property_ = new rviz::TfFrameProperty( "Frame", "base_link",
                      "The frame to center the camera at",
                      getPropertyContainer(),
                      context_->getFrameManager());
}

void CenterFrameTool::activate()
{
  //In the current build of rviz 1.11.15 Indigo, the Current View is always the first property, ie childAt(0)
  rviz::Property* current_view = context_->getViewManager()->getPropertyModel()->getRoot()->childAt(0);

  rviz::VectorProperty* focal_point = 0;
  rviz::StringProperty* target_frame = 0;

  // We need to find pointers to the Target Frame and Focal Point properties
  // Both are members of the current view controller, which is itself a property
  for (int index = 0 ; index < current_view->numChildren(); index++){

    rviz::Property* cur_child = current_view->childAt(index);

    if (cur_child->getNameStd() == "Target Frame"){
      target_frame = dynamic_cast<rviz::StringProperty*>(cur_child);
    }

    if (cur_child->getNameStd() == "Focal Point"){
      focal_point = dynamic_cast<rviz::VectorProperty*>(cur_child);
    }
  }


  if (focal_point && target_frame){

    rviz::FrameManager* fm = context_->getFrameManager();

    std::string target_frame_str = target_frame->getStdString();

    // A check to see if the target frame is <Fixed Frame>, which is not a real frame
    if (target_frame_str == "<Fixed Frame>"){
    	target_frame_str = fm->getFixedFrame();
    }


    Ogre::Vector3 desired_frame_position;
    Ogre::Quaternion desired_frame_orientation;
    Ogre::Vector3 target_frame_position;
    Ogre::Quaternion target_frame_orientation;

    if( fm->getTransform( frame_property_->getStdString(), ros::Time(),
                                              desired_frame_position, desired_frame_orientation )
      &&
        fm->getTransform( target_frame_str, ros::Time(),
                                              target_frame_position, target_frame_orientation ) )
    {

      // In rviz view controller, the focal point is in the target frames
      // fm->getTransform gives you points in the fixed frame
      // Thus, we need to get the difference between the target frame and the desired frame in the fixed frame

      Ogre::Vector3 new_ref_in_target_frame = desired_frame_position - target_frame_position;

      focal_point->setVector(new_ref_in_target_frame);

    }
    else
    {
      ROS_ERROR("Failed to find Ogre vectors between desired center frame and current view target frame");
    }

  }
  else
  {
    ROS_ERROR("Unable to find neccessary properties (Focal Point, Target Frame)");
  }


  context_->getToolManager()->setCurrentTool( context_->getToolManager()->getDefaultTool() );

}


void CenterFrameTool::deactivate()
{

}


} //center_frame_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(center_frame_tool::CenterFrameTool, rviz::Tool )