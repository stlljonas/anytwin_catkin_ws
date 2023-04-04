// any_rqt_gui
#include "any_rqt_gui/AnyRqtNode.h"

// rqt_utils
#include "rqt_utils/common/helper_methods.h"

#include <iostream>
namespace any_rqt_gui {

AnyRqtNode::AnyRqtNode(std::unique_ptr<QApplication> && app,
                       std::unique_ptr<AnyRqtGuiBase> && gui,
                       const unsigned int rosUpdateFrequency):
 app_( std::move(app) ),
 gui_( std::move(gui) ),
 signalHandler_(),
 rosUpdateFrequency_(rosUpdateFrequency),
 nh_(),
 rosSpinner_(),
 isRokOk_(true),
 showGUIServer_()
{
    // connect signals to slots
    QObject::connect(&signalHandler_, SIGNAL(sigINT()),       gui_.get(), SLOT(shutdown()));
    QObject::connect(&signalHandler_, SIGNAL(sigTERM()),      gui_.get(), SLOT(shutdown()));
    QObject::connect(app_.get(),      SIGNAL(aboutToQuit()),  gui_.get(), SLOT(shutdown()));
    QObject::connect(gui_.get(),      SIGNAL(exitApp()),      app_.get(), SLOT(quit()));
    showGUIServer_ = nh_.advertiseService("toggle_gui", &AnyRqtNode::toggleGui, this);
}

AnyRqtNode::~AnyRqtNode() {

}

void AnyRqtNode::init() {
    // Start ros spinner
    rosSpinner_ = std::thread(&AnyRqtNode::spinRos, this);

    // GUI is always on top
    Qt::WindowFlags eFlags = gui_->windowFlags();
    eFlags |= Qt::WindowStaysOnTopHint;
    gui_->setWindowFlags(eFlags);

    // Show gui on fullscreen
    gui_->show();
    gui_->setWindowState(Qt::WindowFullScreen);
    gui_->raise();
    gui_->activateWindow();

    // Load gui from param server
    if(nh_.hasParam("any_rqt_gui_plugins")) {
      std::vector<std::string> plugins;
      nh_.getParam("any_rqt_gui_plugins", plugins);
      gui_->setPluginNames(plugins);
    }

    // Init gui
    gui_->init();

}

void AnyRqtNode::exec() {
    // Execute app
    app_->exec();
    // Join ros spinner thread when app stopped running
    isRokOk_.store(false);
    rosSpinner_.join();
}

void AnyRqtNode::spinRos() {
    ros::Rate r(rosUpdateFrequency_);
    while (isRokOk_)
    {
      ros::spinOnce();
      r.sleep();
    }
}

bool AnyRqtNode::toggleGui(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res) {
  rqt_utils::postToThread( [=]{ gui_->isHidden() ? gui_->show() : gui_->hide(); }, gui_.get());
  res.success = true;
  return true;
}


}
