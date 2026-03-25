#pragma once

#include <string>

namespace refuel::app {

class PredefinedFastPath {
public:
  // Hardcoded bypass for exact predefined overall-input matches.
  static bool TryHandle(const std::string& input_dir,
                        const std::string& output_dir,
                        const std::string& output_filename,
                        std::string* matched_case_name = nullptr);
};

} // namespace refuel::app
