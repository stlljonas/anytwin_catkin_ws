#pragma once

#include <xmlrpcpp/XmlRpc.h>

#include <anydrive/setup/Anydrive.hpp>
#include <anydrive/setup/Setup.hpp>

namespace anydrive_ethercat_ros {

bool readCommunicationParameters(XmlRpc::XmlRpcValue& params, anydrive::setup::AnydrivePtr& anydrivePtr);

bool readSetupParameters(XmlRpc::XmlRpcValue& params, anydrive::setup::SetupPtr& setupPtr);

}  // namespace anydrive_ethercat_ros
