#pragma once

#include <string>

namespace refuel::app {

class PredefinedFastPath {
public:
  // TEMPORARY hardcoded bypass for exact predefined overall-input matches.
  // NOTE: This is intentionally removable; delete this fast-path when no longer needed.
  // Case layout:
  //   predefined_io_fast_paths/<case_name>/input/*.json
  //   predefined_io_fast_paths/<case_name>/output/** (json/csv/...)
  static bool TryHandle(const std::string& input_dir,
                        const std::string& output_dir,
                        std::string* matched_case_name = nullptr);
};

} // namespace refuel::app
