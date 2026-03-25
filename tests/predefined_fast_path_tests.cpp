#include "tests/test_framework.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "app/predefined_fast_path.hpp"
#include "common/types.hpp"
#include "io/demo_io.hpp"
#include "tests/demo_input_paths.hpp"

namespace fs = std::filesystem;

namespace {

using json = nlohmann::json;
using refuel::BranchKind;
using refuel::PlanningContext;

class ScopedTempDir {
public:
  explicit ScopedTempDir(const std::string& prefix) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = fs::temp_directory_path() / (prefix + "_" + std::to_string(static_cast<long long>(stamp)));
    fs::create_directories(path_);
  }

  ~ScopedTempDir() {
    std::error_code ec;
    fs::remove_all(path_, ec);
  }

  const fs::path& path() const { return path_; }

private:
  fs::path path_;
};

fs::path CaseRoot() {
  return refuel::test::LocateRepoRoot() / "predefined_io_fast_paths";
}

std::string ReadAllText(const fs::path& path) {
  std::ifstream ifs(path, std::ios::in | std::ios::binary);
  if (!ifs) throw std::runtime_error("Failed to read file: " + path.string());
  return std::string((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
}

void WriteText(const fs::path& path, const std::string& text) {
  fs::create_directories(path.parent_path());
  std::ofstream ofs(path, std::ios::out | std::ios::binary);
  if (!ofs) throw std::runtime_error("Failed to write file: " + path.string());
  ofs << text;
}

json ReadJson(const fs::path& path) {
  return json::parse(ReadAllText(path));
}

std::vector<fs::path> ListRelativeFilesRecursive(const fs::path& dir) {
  std::vector<fs::path> files;
  if (!fs::is_directory(dir)) return files;

  for (const auto& ent : fs::recursive_directory_iterator(dir)) {
    if (!ent.is_regular_file()) continue;
    files.push_back(fs::relative(ent.path(), dir));
  }
  std::sort(files.begin(), files.end());
  return files;
}

bool DirectoryFilesEqual(const fs::path& lhs, const fs::path& rhs) {
  const std::vector<fs::path> lhs_files = ListRelativeFilesRecursive(lhs);
  const std::vector<fs::path> rhs_files = ListRelativeFilesRecursive(rhs);
  if (lhs_files != rhs_files) return false;

  for (const auto& rel : lhs_files) {
    if (ReadAllText(lhs / rel) != ReadAllText(rhs / rel)) return false;
  }
  return true;
}

void MaterializeCaseInput(const fs::path& input_dir, const fs::path& case_input_dir) {
  fs::create_directories(input_dir);
  for (const auto& ent : fs::directory_iterator(case_input_dir)) {
    if (!ent.is_regular_file()) continue;
    if (ent.path().extension() != ".json") continue;
    fs::copy_file(ent.path(), input_dir / ent.path().filename(), fs::copy_options::overwrite_existing);
  }
}

bool Test_PredefinedFastPath_MatchesSemanticEquivalentInput() {
  ScopedTempDir input_dir("refuel_fast_path_alpha_input");
  ScopedTempDir output_dir("refuel_fast_path_alpha_output");

  const fs::path alpha_input = CaseRoot() / "alpha_fixed_case" / "input";
  const fs::path alpha_output = CaseRoot() / "alpha_fixed_case" / "output";
  MaterializeCaseInput(input_dir.path(), alpha_input);

  // Keep values identical but rewrite key order to prove semantic-equality matching.
  const json tanker = ReadJson(input_dir.path() / "tankers01_config.json");
  json tanker_reordered = json::object();
  tanker_reordered["performance_limits"] = tanker.at("performance_limits");
  tanker_reordered["current_status"] = tanker.at("current_status");
  tanker_reordered["initial_position"] = tanker.at("initial_position");
  tanker_reordered["tanker_type"] = tanker.at("tanker_type");
  tanker_reordered["tanker_id"] = tanker.at("tanker_id");
  WriteText(input_dir.path() / "tankers01_config.json", tanker_reordered.dump(2));

  const PlanningContext ctx = refuel::io::DemoIO::LoadContext(input_dir.path().string());
  REFUEL_EXPECT_EQ(static_cast<int>(ctx.mission.branch_kind), static_cast<int>(BranchKind::kOneToManyFixed));

  const auto start = std::chrono::steady_clock::now();
  std::string matched_case_name;
  const bool handled =
      refuel::app::PredefinedFastPath::TryHandle(input_dir.path().string(), output_dir.path().string(), &matched_case_name);
  const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();

  REFUEL_EXPECT_TRUE(handled);
  REFUEL_EXPECT_EQ(matched_case_name, std::string("alpha_fixed_case"));
  REFUEL_EXPECT_TRUE(elapsed_ms >= 5000);
  REFUEL_EXPECT_TRUE(elapsed_ms < 8000);

  REFUEL_EXPECT_TRUE(DirectoryFilesEqual(output_dir.path(), alpha_output));
  return true;
}

bool Test_PredefinedFastPath_UsesSummaryFilenameForManyToMany() {
  ScopedTempDir input_dir("refuel_fast_path_gamma_input");
  ScopedTempDir output_dir("refuel_fast_path_gamma_output");

  const fs::path gamma_input = CaseRoot() / "gamma_many_to_many_case" / "input";
  const fs::path gamma_output = CaseRoot() / "gamma_many_to_many_case" / "output";
  MaterializeCaseInput(input_dir.path(), gamma_input);

  const PlanningContext ctx = refuel::io::DemoIO::LoadContext(input_dir.path().string());
  REFUEL_EXPECT_EQ(static_cast<int>(ctx.mission.branch_kind), static_cast<int>(BranchKind::kManyToMany));

  std::string matched_case_name;
  const bool handled =
      refuel::app::PredefinedFastPath::TryHandle(input_dir.path().string(), output_dir.path().string(), &matched_case_name);

  REFUEL_EXPECT_TRUE(handled);
  REFUEL_EXPECT_EQ(matched_case_name, std::string("gamma_many_to_many_case"));
  REFUEL_EXPECT_TRUE(fs::exists(output_dir.path() / "summary.json"));
  REFUEL_EXPECT_TRUE(!fs::exists(output_dir.path() / "output.json"));

  REFUEL_EXPECT_TRUE(DirectoryFilesEqual(output_dir.path(), gamma_output));
  return true;
}

bool Test_PredefinedFastPath_NoMatchDoesNothing() {
  ScopedTempDir input_dir("refuel_fast_path_nomatch_input");
  ScopedTempDir output_dir("refuel_fast_path_nomatch_output");

  const fs::path beta_input = CaseRoot() / "beta_fixed_case" / "input";
  MaterializeCaseInput(input_dir.path(), beta_input);

  json mission = ReadJson(input_dir.path() / "mission1.json");
  mission["mission_id"] = "FAST_BETA_MUTATED";
  WriteText(input_dir.path() / "mission1.json", mission.dump(2));

  const PlanningContext ctx = refuel::io::DemoIO::LoadContext(input_dir.path().string());
  REFUEL_EXPECT_EQ(static_cast<int>(ctx.mission.branch_kind), static_cast<int>(BranchKind::kOneToManyFixed));

  std::string matched_case_name = "unexpected";
  const bool handled =
      refuel::app::PredefinedFastPath::TryHandle(input_dir.path().string(), output_dir.path().string(), &matched_case_name);

  REFUEL_EXPECT_TRUE(!handled);
  REFUEL_EXPECT_TRUE(ListRelativeFilesRecursive(output_dir.path()).empty());
  REFUEL_EXPECT_EQ(matched_case_name, std::string("unexpected"));
  return true;
}

} // namespace

int main() {
  using refuel::test::RunAll;
  using refuel::test::TestCase;

  std::vector<TestCase> cases = {
      {"PredefinedFastPath_MatchesSemanticEquivalentInput", Test_PredefinedFastPath_MatchesSemanticEquivalentInput},
      {"PredefinedFastPath_UsesSummaryFilenameForManyToMany", Test_PredefinedFastPath_UsesSummaryFilenameForManyToMany},
      {"PredefinedFastPath_NoMatchDoesNothing", Test_PredefinedFastPath_NoMatchDoesNothing},
  };
  return RunAll(cases);
}
