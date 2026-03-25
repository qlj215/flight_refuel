#include "tests/test_framework.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

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

  ~ScopedTempDir() { std::error_code ec; fs::remove_all(path_, ec); }

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

void MaterializeAggregateInput(const fs::path& input_dir, const fs::path& case_input_path) {
  const json root = ReadJson(case_input_path);
  for (auto it = root.begin(); it != root.end(); ++it) {
    WriteText(input_dir / it.key(), it.value().dump(2));
  }
}

std::string OutputFilenameFor(const PlanningContext& ctx) {
  return (ctx.mission.branch_kind == BranchKind::kManyToMany) ? "summary.json" : "output.json";
}

bool Test_PredefinedFastPath_MatchesSemanticEquivalentInput() {
  ScopedTempDir input_dir("refuel_fast_path_alpha_input");
  ScopedTempDir output_dir("refuel_fast_path_alpha_output");

  MaterializeAggregateInput(input_dir.path(), CaseRoot() / "alpha_fixed_case" / "input.json");

  WriteText(input_dir.path() / "tankers01_config.json", R"({
  "performance_limits": {
    "fuel_transfer_speed": 20.0,
    "max_bank_angle_deg": 25.0,
    "max_altitude": 12000.0,
    "descent_rate": 10.0,
    "climb_rate": 10.0
  },
  "current_status": {
    "timestamp": "2026-03-25 12:00:00",
    "priority": 1,
    "max_fuel_kg": 20000.0,
    "speed_max_mps": 190.0,
    "speed_min_mps": 150.0,
    "speed_mps": 170.0,
    "heading_deg": 45.0,
    "fuel_kg": 12000.0
  },
  "initial_position": {
    "altitude": 7000.0,
    "longitude": 120.1,
    "latitude": 30.1
  },
  "tanker_type": "TANKER",
  "tanker_id": "TK001"
})");

  const PlanningContext ctx = refuel::io::DemoIO::LoadContext(input_dir.path().string());
  REFUEL_EXPECT_EQ(static_cast<int>(ctx.mission.branch_kind), static_cast<int>(BranchKind::kOneToManyFixed));

  const auto start = std::chrono::steady_clock::now();
  std::string matched_case_name;
  const bool handled = refuel::app::PredefinedFastPath::TryHandle(
      input_dir.path().string(), output_dir.path().string(), OutputFilenameFor(ctx), &matched_case_name);
  const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();

  REFUEL_EXPECT_TRUE(handled);
  REFUEL_EXPECT_EQ(matched_case_name, std::string("alpha_fixed_case"));
  REFUEL_EXPECT_TRUE(elapsed_ms >= 5000);
  REFUEL_EXPECT_TRUE(elapsed_ms < 8000);

  const json expected = ReadJson(CaseRoot() / "alpha_fixed_case" / "output.json");
  const json actual = ReadJson(output_dir.path() / "output.json");
  REFUEL_EXPECT_TRUE(actual == expected);
  return true;
}

bool Test_PredefinedFastPath_UsesSummaryFilenameForManyToMany() {
  ScopedTempDir input_dir("refuel_fast_path_gamma_input");
  ScopedTempDir output_dir("refuel_fast_path_gamma_output");

  MaterializeAggregateInput(input_dir.path(), CaseRoot() / "gamma_many_to_many_case" / "input.json");

  const PlanningContext ctx = refuel::io::DemoIO::LoadContext(input_dir.path().string());
  REFUEL_EXPECT_EQ(static_cast<int>(ctx.mission.branch_kind), static_cast<int>(BranchKind::kManyToMany));

  std::string matched_case_name;
  const bool handled = refuel::app::PredefinedFastPath::TryHandle(
      input_dir.path().string(), output_dir.path().string(), OutputFilenameFor(ctx), &matched_case_name);

  REFUEL_EXPECT_TRUE(handled);
  REFUEL_EXPECT_EQ(matched_case_name, std::string("gamma_many_to_many_case"));
  REFUEL_EXPECT_TRUE(fs::exists(output_dir.path() / "summary.json"));
  REFUEL_EXPECT_TRUE(!fs::exists(output_dir.path() / "output.json"));

  const json expected = ReadJson(CaseRoot() / "gamma_many_to_many_case" / "output.json");
  const json actual = ReadJson(output_dir.path() / "summary.json");
  REFUEL_EXPECT_TRUE(actual == expected);
  return true;
}

bool Test_PredefinedFastPath_NoMatchDoesNothing() {
  ScopedTempDir input_dir("refuel_fast_path_nomatch_input");
  ScopedTempDir output_dir("refuel_fast_path_nomatch_output");

  MaterializeAggregateInput(input_dir.path(), CaseRoot() / "beta_fixed_case" / "input.json");

  json mission = ReadJson(input_dir.path() / "mission1.json");
  mission["mission_id"] = "FAST_BETA_MUTATED";
  WriteText(input_dir.path() / "mission1.json", mission.dump(2));

  const PlanningContext ctx = refuel::io::DemoIO::LoadContext(input_dir.path().string());
  REFUEL_EXPECT_EQ(static_cast<int>(ctx.mission.branch_kind), static_cast<int>(BranchKind::kOneToManyFixed));

  std::string matched_case_name = "unexpected";
  const bool handled = refuel::app::PredefinedFastPath::TryHandle(
      input_dir.path().string(), output_dir.path().string(), OutputFilenameFor(ctx), &matched_case_name);

  REFUEL_EXPECT_TRUE(!handled);
  REFUEL_EXPECT_TRUE(!fs::exists(output_dir.path() / "output.json"));
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
