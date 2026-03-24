#include "tests/test_framework.hpp"

#include <filesystem>
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

#include "branches/suidui_branch.hpp"
#include "io/demo_io.hpp"
#include "tests/demo_input_paths.hpp"

namespace {

using refuel::BranchKind;
using refuel::PlanningContext;
using refuel::test::DemoInputDir;

bool Test_Suidui_SingleReceiver_OutputSchema() {
  PlanningContext ctx = refuel::io::DemoIO::LoadContext(DemoInputDir("0210"));

  REFUEL_EXPECT_TRUE(!ctx.tankers.empty());
  REFUEL_EXPECT_TRUE(!ctx.receivers.empty());

  ctx.tankers.resize(1);
  ctx.tanker = ctx.tankers.front();
  ctx.mission.tanker_ids = {ctx.tanker.id};
  ctx.mission.tanker_id = ctx.tanker.id;

  ctx.receivers.resize(1);
  auto& r = ctx.receivers.front();
  ctx.mission.receiver_ids = {r.id};

  ctx.mission.branch_kind = BranchKind::kFollow;
  ctx.mission.refuel_mode.sub_mode = "suidui";

  // 构造一组稳定可行参数
  ctx.tanker.current_status.heading_deg = 30.0;
  ctx.tanker.current_status.speed_mps = 160.0;
  ctx.tanker.current_status.speed_min_mps = 120.0;
  ctx.tanker.current_status.speed_max_mps = 200.0;

  r.current_status.heading_deg = 80.0;
  r.current_status.speed_mps = 145.0;
  r.current_status.speed_min_mps = 120.0;
  r.current_status.speed_max_mps = 200.0;

  if (r.current_status.fuel_kg <= 0.0) r.current_status.fuel_kg = 4000.0;
  if (r.max_fuel_kg <= 0.0) r.max_fuel_kg = 7000.0;

  const auto out_dir = std::filesystem::temp_directory_path() / "refuel_suidui_test_out";
  std::filesystem::create_directories(out_dir);

  std::string err;
  const bool ok = refuel::branch::RunSuiduiBranch(ctx, out_dir.string(), &err);
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
  REFUEL_EXPECT_TRUE(j.contains("tanker_meeting_params"));
  REFUEL_EXPECT_TRUE(j.contains("receiver_meeting_params"));
  REFUEL_EXPECT_TRUE(j.contains("team_fly_params"));
  REFUEL_EXPECT_TRUE(j.contains("fueling_time"));
  REFUEL_EXPECT_TRUE(j.contains("fuel_consumption"));
  REFUEL_EXPECT_TRUE(j.contains("fueling_params"));

  REFUEL_EXPECT_TRUE(j["meeting_point"].is_array());
  REFUEL_EXPECT_TRUE(j["meeting_point"].size() == 2);
  REFUEL_EXPECT_TRUE(j["fueling_time"].is_number());
  REFUEL_EXPECT_TRUE(j["fueling_time"].get<double>() > 0.0);

  REFUEL_EXPECT_TRUE(std::filesystem::exists(out_dir / "tanker01.csv"));
  REFUEL_EXPECT_TRUE(std::filesystem::exists(out_dir / "receiver01.csv"));

  return true;
}

} // namespace

int main() {
  using refuel::test::RunAll;
  using refuel::test::TestCase;

  std::vector<TestCase> cases = {
      {"Suidui_SingleReceiver_OutputSchema", Test_Suidui_SingleReceiver_OutputSchema},
  };
  return RunAll(cases);
}
