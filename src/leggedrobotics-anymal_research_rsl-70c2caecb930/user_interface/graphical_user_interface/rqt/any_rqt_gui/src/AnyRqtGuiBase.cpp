// any_rqt_gui
#include "any_rqt_gui/AnyRqtGuiBase.h"

// Qt
#include <QLayout>
#include <QLayoutItem>
#include <QStringList>
#include <QEvent>
#include <QKeyEvent>

namespace any_rqt_gui {

AnyRqtGuiBase::AnyRqtGuiBase( const std::string & name,
                              const std::vector< std::string > & pluginNames)
    : QWidget()
    , pluginNames_(pluginNames)
    , classLoader_("rqt_gui", "rqt_gui_cpp::Plugin")
{
  setObjectName( QString::fromStdString(name) );
  this->installEventFilter(this);
}

bool AnyRqtGuiBase::eventFilter(QObject* obj, QEvent* event) {
  if (event->type()==QEvent::KeyPress) {
      QKeyEvent* key = static_cast<QKeyEvent*>(event);
      if (key->key()==Qt::Key_F11) {
          this->setWindowState(Qt::WindowFullScreen);
          this->raise();
          this->activateWindow();
      } else if(key->key()==Qt::Key_Escape) {
          this->setWindowState(Qt::WindowNoState);
          this->raise();
          this->activateWindow();
      } else {
          return QObject::eventFilter(obj, event);
      }
      return true;
  } else {
      return QObject::eventFilter(obj, event);
  }
}

void AnyRqtGuiBase::init() {
  try
  {
      for(auto pluginName : pluginNames_ ) {
          plugins_.push_back(classLoader_.createInstance(pluginName));
      }
  }
  catch (const pluginlib::PluginlibException& ex)
  {
      ROS_ERROR("Error: %s", ex.what());
  }

  qt_gui_cpp::PluginContext context(this, 1, QStringList());
  for(auto& plugin : plugins_ ) { plugin->initPlugin(context); }
}

void AnyRqtGuiBase::shutdown() {
  for(auto& plugin : plugins_ ) { plugin->shutdownPlugin(); }

  // Delete all widgets
  if ( this->layout() != NULL )
  {
      QLayoutItem* item;
      while ( ( item = this->layout()->takeAt( 0 ) ) != NULL )
      {
          delete item->widget();
          delete item;
      }
      delete this->layout();
  }

  // Force application quit
  emit exitApp();
}

void AnyRqtGuiBase::add_widget(QWidget* widget) {

  QWidget* placeholder = this->findChild<QWidget*>(widget->objectName());
  if(placeholder != nullptr){
    if(placeholder->layout() == nullptr) {
      placeholder->setLayout(new QHBoxLayout);
    }
    placeholder->layout()->addWidget(widget);
    widget->setParent(placeholder);
    widget->resize(widget->size());
    show();
  } else {
    std::cout << "ERROR: No placeholder for widget " << widget->objectName().toStdString() << std::endl;
  }
}



} // any_rqt_gui
