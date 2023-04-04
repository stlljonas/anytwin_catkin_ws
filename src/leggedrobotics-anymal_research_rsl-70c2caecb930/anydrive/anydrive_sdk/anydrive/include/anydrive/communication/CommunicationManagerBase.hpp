#pragma once

#include <memory>

#include "anydrive/setup/Setup.hpp"

namespace anydrive {

//! AnydriveManager forward declaration.
class AnydriveManager;

namespace communication {

class CommunicationManagerBase {
 protected:
  CommunicationManagerBase() = default;
  virtual ~CommunicationManagerBase() = default;

 public:
  virtual bool loadSetup(const setup::SetupPtr setup, AnydriveManager* anydriveManager) = 0;

  virtual bool startup() = 0;
  virtual void updateRead() = 0;
  virtual void updateWrite() = 0;
  virtual void shutdown() = 0;
  virtual bool isCommunicationOk() = 0;
  virtual unsigned int getWorkingCounterTooLowCount() = 0;
};

using CommunicationManagerBasePtr = std::shared_ptr<CommunicationManagerBase>;

}  // namespace communication
}  // namespace anydrive
