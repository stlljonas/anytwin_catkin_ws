
// C++ standard library
#include <locale>

// point cloud processor
#include "point_cloud_processor/PointCloudProcessor.hpp"

ros::console::levels::Level getLogLevel(const ros::NodeHandle& nodeHandle) {
  std::string logLevelParam;

  if (!nodeHandle.getParam("log_level", logLevelParam)) {
    MELO_WARN_STREAM("Log level not set. Using default \"WARN\"");
    logLevelParam = "WARN";
  }

  std::transform(logLevelParam.begin(), logLevelParam.end(), logLevelParam.begin(),
                 [](const char c) { return std::use_facet<std::ctype<char>>(std::locale()).toupper(c); });

  ros::console::levels::Level logLevel = ros::console::levels::Warn;

  if (logLevelParam == "DEBUG") {
    logLevel = ros::console::levels::Debug;
  } else if (logLevelParam == "INFO") {
    logLevel = ros::console::levels::Info;
  } else if (logLevelParam == "WARN") {
    logLevel = ros::console::levels::Warn;
  } else if (logLevelParam == "ERROR") {
    logLevel = ros::console::levels::Error;
  } else if (logLevelParam == "FATAL") {
    logLevel = ros::console::levels::Fatal;
  } else if (logLevelParam == "COUNT") {
    logLevel = ros::console::levels::Count;
  } else {
    MELO_WARN_STREAM("Unknown log level in configuration file. Using default \"WARN\"");
  }

  return logLevel;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_processor");
  ros::NodeHandle privateNodeHandle("~");

  const auto logLevel = getLogLevel(privateNodeHandle);

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, logLevel)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  point_cloud_processor::PointCloudProcessor pointCloudProcessor(privateNodeHandle);

  ros::MultiThreadedSpinner spinner(5);
  spinner.spin();

  return EXIT_SUCCESS;
}
