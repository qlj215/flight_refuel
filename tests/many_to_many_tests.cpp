#include "tests/test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "io/demo_io.hpp"
#include "pipeline/pipeline.hpp"
#include "tests/demo_input_paths.hpp"
#include "tests/many_to_many_test_utils.hpp"

namespace {

using refuel::BranchKind;
using refuel::PlanningContext;
using refuel::test::DemoInputDir;
using refuel::test::MakeOneToManySubContext;

bool IsFinite(double v) { return std::isfinite(v); }

const refuel::AircraftConfig* FindReceiver(const PlanningContext& ctx, const std::string& id) {
  auto it = std::find_if(ctx.receivers.begin(), ctx.receivers.end(), [&](const refuel::AircraftConfig& r) {
    return r.id == id;
  });
  return (it == ctx.receivers.end()) ? nullptr : &(*it);
}

bool Test_ManyToMany_ParseExtendedInputs() {
  const PlanningContext ctx = refuel::io::DemoIO::LoadContext(DemoInputDir("0306_many_to_many"));

  REFUEL_EXPECT_EQ(static_cast<int>(ctx.mission.branch_kind), static_cast<int>(BranchKind::kManyToMany));
  REFUEL_EXPECT_EQ(ctx.mission.refuel_mode.safe_zone.zones_vertices_lla.size(), std::size_t{2});
  REFUEL_EXPECT_EQ(ctx.mission.operation_area.no_fly_zones.zones_vertices_lla.size(), std::size_t{2});
  REFUEL_EXPECT_EQ(ctx.tankers.size(), std::size_t{2});
  REFUEL_EXPECT_EQ(ctx.receivers.size(), std::size_t{3});
  REFUEL_EXPECT_EQ(ctx.mission.assignments.size(), std::size_t{2});

  REFUEL_EXPECT_EQ(ctx.mission.assignments[0].tanker_id, std::string("TK001"));
  REFUEL_EXPECT_EQ(ctx.mission.assignments[1].tanker_id, std::string("TK002"));
  REFUEL_EXPECT_EQ(ctx.mission.assignments[0].receiver_ids.size(), std::size_t{2});
  REFUEL_EXPECT_EQ(ctx.mission.assignments[1].receiver_ids.size(), std::size_t{1});

  REFUEL_EXPECT_NEAR(ctx.tankers[0].fuel_transfer_speed_kgps, 25.0, 1e-9);
  REFUEL_EXPECT_NEAR(ctx.tankers[1].fuel_transfer_speed_kgps, 25.0, 1e-9);

  const auto* re001 = FindReceiver(ctx, "RE001");
  const auto* re002 = FindReceiver(ctx, "RE002");
  REFUEL_EXPECT_TRUE(re001 != nullptr);
  REFUEL_EXPECT_TRUE(re002 != nullptr);
  REFUEL_EXPECT_NEAR(re001->max_fuel_kg, 7000.0, 1e-9);
  REFUEL_EXPECT_EQ(re001->priority, 2);
  REFUEL_EXPECT_EQ(re002->priority, 1);
  return true;
}

bool Test_ManyToMany_SubContextAssembly() {
  const PlanningContext base = refuel::io::DemoIO::LoadContext(DemoInputDir("0306_many_to_many"));

  PlanningContext sub1 = MakeOneToManySubContext(base, base.mission.assignments[0], {});
  REFUEL_EXPECT_EQ(static_cast<int>(sub1.mission.branch_kind), static_cast<int>(BranchKind::kOneToManyFixed));
  REFUEL_EXPECT_EQ(sub1.tanker.id, std::string("TK001"));
  REFUEL_EXPECT_EQ(sub1.mission.tanker_ids.size(), std::size_t{1});
  REFUEL_EXPECT_EQ(sub1.receivers.size(), std::size_t{2});
  REFUEL_EXPECT_EQ(sub1.receivers[0].id, std::string("RE002"));
  REFUEL_EXPECT_EQ(sub1.receivers[1].id, std::string("RE003"));
  REFUEL_EXPECT_TRUE(sub1.excluded_safezone_ids.empty());

  PlanningContext sub2 = MakeOneToManySubContext(base, base.mission.assignments[1], {1});
  REFUEL_EXPECT_EQ(sub2.tanker.id, std::string("TK002"));
  REFUEL_EXPECT_EQ(sub2.receivers.size(), std::size_t{1});
  REFUEL_EXPECT_EQ(sub2.receivers[0].id, std::string("RE001"));
  REFUEL_EXPECT_EQ(sub2.excluded_safezone_ids.size(), std::size_t{1});
  REFUEL_EXPECT_EQ(sub2.excluded_safezone_ids[0], 1);
  return true;
}

bool Test_ManyToMany_SafeZoneExclusionAcrossSubtasks() {
  const PlanningContext base = refuel::io::DemoIO::LoadContext(DemoInputDir("0306_many_to_many"));
  refuel::Pipeline pipe;

  PlanningContext sub1 = MakeOneToManySubContext(base, base.mission.assignments[0], {});
  pipe.Run(sub1);
  REFUEL_EXPECT_EQ(sub1.safe_zone_selected.chosen_safezone_id, 1);
  REFUEL_EXPECT_EQ(sub1.sequence.sequence.size(), std::size_t{2});
  REFUEL_EXPECT_EQ(sub1.rendezvous.receivers.size(), std::size_t{2});

  PlanningContext sub2 = MakeOneToManySubContext(base, base.mission.assignments[1], {sub1.safe_zone_selected.chosen_safezone_id});
  pipe.Run(sub2);
  REFUEL_EXPECT_EQ(sub2.safe_zone_selected.chosen_safezone_id, 2);
  REFUEL_EXPECT_TRUE(sub2.safe_zone_selected.chosen_safezone_id != sub1.safe_zone_selected.chosen_safezone_id);
  REFUEL_EXPECT_EQ(sub2.sequence.sequence.size(), std::size_t{1});
  REFUEL_EXPECT_EQ(sub2.rendezvous.receivers.size(), std::size_t{1});
  return true;
}

bool Test_ManyToMany_SubtasksProduceTrajectoriesAndCosts() {
  const PlanningContext base = refuel::io::DemoIO::LoadContext(DemoInputDir("0306_many_to_many"));
  refuel::Pipeline pipe;

  std::vector<int> excluded;
  for (const auto& asg : base.mission.assignments) {
    PlanningContext sub = MakeOneToManySubContext(base, asg, excluded);
    pipe.Run(sub);

    REFUEL_EXPECT_TRUE(!sub.trajectories.empty());
    REFUEL_EXPECT_TRUE(IsFinite(sub.cost.tanker_arrival_time_s));
    REFUEL_EXPECT_TRUE(IsFinite(sub.cost.tanker_fuel_consumed));
    REFUEL_EXPECT_EQ(sub.sequence.sequence.size(), sub.receivers.size());
    REFUEL_EXPECT_EQ(sub.rendezvous.receivers.size(), sub.receivers.size());

    const std::size_t expected_traj_count = sub.receivers.size() + std::size_t{1};
    REFUEL_EXPECT_EQ(sub.trajectories.size(), expected_traj_count);

    if (sub.safe_zone_selected.chosen_safezone_id >= 0) {
      excluded.push_back(sub.safe_zone_selected.chosen_safezone_id);
    }
  }

  REFUEL_EXPECT_EQ(excluded.size(), std::size_t{2});
  REFUEL_EXPECT_EQ(excluded[0], 1);
  REFUEL_EXPECT_EQ(excluded[1], 2);
  return true;
}

}  // namespace

int main() {
  using refuel::test::RunAll;
  using refuel::test::TestCase;

  std::vector<TestCase> cases = {
      {"ManyToMany_ParseExtendedInputs", Test_ManyToMany_ParseExtendedInputs},
      {"ManyToMany_SubContextAssembly", Test_ManyToMany_SubContextAssembly},
      {"ManyToMany_SafeZoneExclusionAcrossSubtasks", Test_ManyToMany_SafeZoneExclusionAcrossSubtasks},
      {"ManyToMany_SubtasksProduceTrajectoriesAndCosts", Test_ManyToMany_SubtasksProduceTrajectoriesAndCosts},
  };
  return RunAll(cases);
}
