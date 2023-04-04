#include "anydrive/setup/Anydrive.hpp"
#include "anydrive/Exception.hpp"

namespace anydrive {
namespace setup {

void Anydrive::fromFile(const yaml_tools::YamlNode& yamlNode, const std::string& setupFile) {
  if (yamlNode.hasKey("name")) {
    name_ = yamlNode["name"].as<std::string>();
  }
  if (yamlNode.hasKey("configuration_file")) {
    std::string configurationFile = yamlNode["configuration_file"].as<std::string>();
    if (configurationFile.empty()) {
      throw Exception("The path to the configuration file is empty.");
    } else if (configurationFile.front() == '/') {
      // Path to the configuration file is absolute, we can use it as is.
    } else if (configurationFile.front() == '~') {
      // Path to the configuration file is absolute, we need to replace '~' with the home directory.
      const char* homeDirectory = getenv("HOME");
      if (homeDirectory == nullptr) {
        throw Exception("Environment variable 'HOME' could not be evaluated.");
      }
      configurationFile.erase(configurationFile.begin());
      configurationFile = homeDirectory + configurationFile;
    } else {
      // Path to the configuration file is relative, we need to append it to the path of the setup file.
      configurationFile = setupFile.substr(0, setupFile.find_last_of("/") + 1) + configurationFile;
    }
    configuration_.fromFile(configurationFile);
  }
}

}  // namespace setup
}  // namespace anydrive
