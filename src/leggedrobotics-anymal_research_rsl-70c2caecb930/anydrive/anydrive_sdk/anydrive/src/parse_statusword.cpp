#include <cstdlib>

#include "anydrive/Statusword.hpp"

int main(int argc, char** argv) {
  // Check for number of arguments.
  if (argc != 2) {
    std::cout << "Usage: ./parse_statusword 1234" << std::endl;
    return -1;
  }

  // Check for valid input.
  long long int input = std::atoll(argv[1]);  // NOLINT
  using Data = uint32_t;
  const Data min = std::numeric_limits<Data>::min();
  const Data max = std::numeric_limits<Data>::max();
  if (input < min || input > max) {
    std::cout << "Statusword " << input << " is out of range [" << min << ", " << max << "]." << std::endl;
    return -1;
  }

  // Parse statusword.
  const Data data = static_cast<Data>(input);
  std::cout << "Read Statusword: " << data << std::endl;
  anydrive::Statusword statusword(data);
  std::cout << "FSM State: " << anydrive::fsm::stateEnumToName(statusword.getStateEnum()) << std::endl;
  std::cout << "Mode of Operation: " << anydrive::mode::modeEnumToName(statusword.getModeEnum()) << std::endl;
  std::vector<std::string> infos;
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
  std::vector<std::string> fatals;
  statusword.getMessages(infos, warnings, errors, fatals);
  for (const std::string& info : infos) {
    std::cout << info << std::endl;
  }
  for (const std::string& warning : warnings) {
    std::cout << warning << std::endl;
  }
  for (const std::string& error : errors) {
    std::cout << error << std::endl;
  }
  for (const std::string& fatal : fatals) {
    std::cout << fatal << std::endl;
  }
  return 0;
}
