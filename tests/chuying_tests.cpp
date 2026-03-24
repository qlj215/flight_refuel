#include "tests/test_framework.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

#include "branches/chuying_branch.hpp"
#include "io/demo_io.hpp"
#include "tests/demo_input_paths.hpp"

namespace {

using refuel::BranchKind;
using refuel::PlanningContext;
using refuel::test::DemoInputDir;

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kPi = 3.14159265358979323846;

refuel::LLA XYToLLA(double x, double y, double alt_m) {
  const double lon = x / kEarthRadiusM;
  const double lat = 2.0 * std::atan(std::exp(y / kEarthRadiusM)) - kPi / 2.0;
  return refuel::LLA{lat * 180.0 / kPi, lon * 180.0 / kPi, alt_m};
}

bool Test_Chuying_SingleReceiver_OutputSchema() {
  PlanningContext ctx = refuel::io::DemoIO::LoadContext(DemoInputDir("0210"));

  // 仅保留一机一受油机，构造可行出迎样例（与 0324 模型案例一致）
  REFUEL_EXPECT_TRUE(!ctx.tankers.empty());
  REFUEL_EXPECT_TRUE(!ctx.receivers.empty());

  ctx.tankers.resize(1);
  ctx.tanker = ctx.tankers.front();
  ctx.mission.tanker_ids = {ctx.tanker.id};
  ctx.mission.tanker_id = ctx.tanker.id;

  ctx.receivers.resize(1);
  auto& r = ctx.receivers.front();
  ctx.mission.receiver_ids = {r.id};

  ctx.mission.branch_kind = BranchKind::kIntercept;
  ctx.mission.refuel_mode.sub_mode = "chuying";
  ctx.mission.prefs.weight_factors = {0.5, 0.2, 0.3};

  // tanker
  ctx.tanker.initial_position_lla = XYToLLA(12917500.0, 3144000.0, 8000.0);
  ctx.tanker.current_status.heading_deg = 90.0;
  ctx.tanker.current_status.speed_mps = 150.0;
  ctx.tanker.current_status.fuel_kg = 8000.0;

  // receiver
  r.initial_position_lla = XYToLLA(13330000.0, 3246000.0, 7500.0);
  r.current_status.heading_deg = 180.0;
  r.current_status.speed_mps = 150.0;
  r.current_status.fuel_kg = 3500.0;
  r.priority = 1;  // damage_state=1

  const auto out_dir = std::filesystem::temp_directory_path() / "refuel_chuying_test_out";
  std::filesystem::create_directories(out_dir);

  std::string err;
  const bool ok = refuel::branch::RunChuyingBranch(ctx, out_dir.string(), &err);
  REFUEL_EXPECT_TRUE(ok);
  REFUEL_EXPECT_TRUE(err.empty());

  const auto out_json = out_dir / "output.json";
  REFUEL_EXPECT_TRUE(std::filesystem::exists(out_json));

  nlohmann::json j;
  {
    std::ifstream ifs(out_json);
    REFUEL_EXPECT_TRUE(static_cast<bool>(ifs));
    ifs >> j;
  }

  REFUEL_EXPECT_TRUE(j.contains("meeting_point"));
  REFUEL_EXPECT_TRUE(j.contains("meeting_method"));
  REFUEL_EXPECT_TRUE(j.contains("tanker_meeting_params"));
  REFUEL_EXPECT_TRUE(j.contains("meeting_time"));
  REFUEL_EXPECT_TRUE(j.contains("fuel_consumption"));
  REFUEL_EXPECT_TRUE(j.contains("fueling_params"));
  REFUEL_EXPECT_TRUE(j.contains("phase"));

  REFUEL_EXPECT_TRUE(j["meeting_time"].is_number());
  REFUEL_EXPECT_TRUE(j["meeting_time"].get<double>() > 0.0);

  REFUEL_EXPECT_TRUE(j["tanker_meeting_params"].contains("speed"));
  REFUEL_EXPECT_TRUE(j["fueling_params"].contains("speed"));
  REFUEL_EXPECT_TRUE(j["fuel_consumption"].contains("tanker"));
  REFUEL_EXPECT_TRUE(j["fuel_consumption"].contains("receiver"));

  REFUEL_EXPECT_TRUE(j["phase"].contains("tanker_midpoint"));
  REFUEL_EXPECT_TRUE(j["phase"].contains("receiver_midpoint"));

  REFUEL_EXPECT_TRUE(std::filesystem::exists(out_dir / "tanker01.csv"));
  REFUEL_EXPECT_TRUE(std::filesystem::exists(out_dir / "receiver01.csv"));

  return true;
}

}  // namespace

int main() {
  using refuel::test::RunAll;
  using refuel::test::TestCase;

  std::vector<TestCase> cases = {
      {"Chuying_SingleReceiver_OutputSchema", Test_Chuying_SingleReceiver_OutputSchema},
  };
  return RunAll(cases);
}
