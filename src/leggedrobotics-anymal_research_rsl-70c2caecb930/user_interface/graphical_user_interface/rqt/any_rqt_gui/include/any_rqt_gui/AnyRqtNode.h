#pragma once

// any_rqt_gui
#include "any_rqt_gui/AnyRqtGuiBase.h"

// rqt_utils
#include "rqt_utils/common/QSignalHandler.h"

//Qt
#include <QApplication>

// Ros
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

// STL
#include <atomic>
#include <thread>

namespace any_rqt_gui {

class AnyRqtNode {

public:
    //! @brief Constructor
    AnyRqtNode(std::unique_ptr<QApplication> && app,
               std::unique_ptr<AnyRqtGuiBase> && gui,
               const unsigned int rosUpdateFrequency);

    //! @brief Destructor
    ~AnyRqtNode();

    //! @brief initializes the app, sets window properties and starts ros spinner thread
    virtual void init();

    //! @brief executes the app
    virtual void exec();

    //! @brief Service to toggle the gui
    bool toggleGui(std_srvs::Trigger::Request  &req,
                   std_srvs::Trigger::Response &res);

protected:
  //! @brief ros spinner function (executed in a seperate thread)
  void spinRos();

private:
    //! Qt application
    std::unique_ptr<QApplication> app_;
    //! Any Rqt Gui
    std::unique_ptr<AnyRqtGuiBase> gui_;
    //! Signal handler
    rqt_utils::QSignalHandler signalHandler_;
    //! Ros update frequency
    const unsigned int rosUpdateFrequency_;
    //! Nodehandle
    ros::NodeHandle nh_;
    //! Ros spinner thread
    std::thread rosSpinner_;
    std::atomic_bool isRokOk_;
    //! Ros services
    ros::ServiceServer showGUIServer_;
};

}
