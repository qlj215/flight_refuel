#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace refuel::test {

inline std::filesystem::path LocateRepoRoot() {
  namespace fs = std::filesystem;
  const fs::path cwd = fs::current_path();
  std::vector<fs::path> candidates = {
      cwd,
      cwd.parent_path(),
      cwd.parent_path().parent_path(),
      fs::path(__FILE__).parent_path().parent_path(),
  };

  for (const auto& cand : candidates) {
    if (!cand.empty() && fs::exists(cand / "CMakeLists.txt") && fs::exists(cand / "demo" / "input")) {
      return cand;
    }
  }
  throw std::runtime_error("Cannot locate repository root from current working directory.");
}

inline std::string DemoInputDir(const std::string& case_name) {
  return (LocateRepoRoot() / "demo" / "input" / case_name).string();
}

}  // namespace refuel::test
