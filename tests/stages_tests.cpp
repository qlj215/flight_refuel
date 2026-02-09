#include "tests/test_framework.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

// =========================
// Stage-by-stage tests
// =========================
//
// 目标：对 include/stages 下每个 Stage 的“输入/输出契约”做尽可能严格的单元/小集成测试。
//
// 说明（你后续维护/扩展时，建议一直遵守）：
// - 每个 Stage 都应把输出写回 PlanningContext（ctx）里对应字段。
// - Stage::Run 必须是“幂等/可重复调用”的：本次 Run 应该覆盖或清理旧输出，避免残留。
// - 测试检查的重点：
//     1) 是否写了正确的 ctx 字段
//     2) 写出的结构/格式是否满足后续 Stage 的输入要求
//     3) 关键字段取值范围（非空、非 NaN、合理的边界）
//

#include "pipeline/pipeline.hpp"

#include "stages/coordinate_transform_stage.hpp"
#include "stages/preprocess_stage.hpp"
#include "stages/safe_zone_select_stage.hpp"
#include "stages/racetrack_build_stage.hpp"
#include "stages/sequence_stage.hpp"
#include "stages/tanker_racetrack_decision_stage.hpp"
#include "stages/rendezvous_stage.hpp"
#include "stages/path_planning_stage.hpp"
#include "stages/cost_stage.hpp"

namespace {

static constexpr double kPi = 3.14159265358979323846;

inline double Deg2Rad(double deg) { return deg * kPi / 180.0; }

inline bool IsFinite(double v) { return std::isfinite(v); }

inline bool NearlyEqual(double a, double b, double eps = 1e-6) {
  return std::fabs(a - b) <= eps;
}

refuel::AircraftConfig MakeAircraft(std::string id, double x, double y, double alt_m, double speed_mps, double heading_deg) {
  refuel::AircraftConfig ac;
  ac.id = std::move(id);
  ac.initial_position_xy = {x, y, 0.0};
  ac.initial_position_lla = {0.0, 0.0, alt_m};
  ac.current_status.speed_mps = speed_mps;
  ac.current_status.heading_deg = heading_deg;
  // limits: give non-zero to avoid planner clamp issues
  ac.limits.climb_rate = 5.0;
  ac.limits.descent_rate = 5.0;
  return ac;
}

refuel::Polygon MakeSquarePolygon(double half_size_m) {
  refuel::Polygon p;
  // Counter-clockwise
  p.vertices_xy = {
      {-half_size_m, -half_size_m, 0.0},
      {+half_size_m, -half_size_m, 0.0},
      {+half_size_m, +half_size_m, 0.0},
      {-half_size_m, +half_size_m, 0.0},
  };
  return p;
}

// =========================
// Pretty printers
// =========================
// 你希望在测试输出里“看见具体数值”：
// - 进入某个 Stage 的输入是什么（关键字段 + 数值）
// - 该 Stage 跑完后生成了哪些输出（关键字段 + 数值）
// 因此这里提供一组轻量 dump 工具，所有 stage 测试都会调用。

void PrintBanner(const std::string& title) {
  std::cout << "\n";
  std::cout << "============================================================\n";
  std::cout << title << "\n";
  std::cout << "============================================================\n";
  std::cout << std::fixed << std::setprecision(6);
}

void PrintSub(const std::string& title) {
  std::cout << "\n-- " << title << " --\n";
}

void DumpLLA(const std::string& name, const refuel::LLA& p) {
  std::cout << name << " { lat_deg=" << p.lat_deg << ", lon_deg=" << p.lon_deg << ", alt_m=" << p.alt_m << " }\n";
}

void DumpVec3(const std::string& name, const refuel::Vec3& p) {
  std::cout << name << " { x=" << p.x << ", y=" << p.y << ", z=" << p.z << " }\n";
}

void DumpSpeedBounds(const std::string& name, const refuel::SpeedBounds& b) {
  std::cout << name << " { min=" << b.min_speed_mps
            << ", cruise=" << b.cruise_speed_mps
            << ", max=" << b.max_speed_mps << " } (m/s)\n";
}

void DumpPolygonXY(const std::string& name, const refuel::Polygon& poly, std::size_t max_v = 8) {
  std::cout << name << " { vertices_xy.size=" << poly.vertices_xy.size() << " }\n";
  const std::size_t n = std::min<std::size_t>(poly.vertices_xy.size(), max_v);
  for (std::size_t i = 0; i < n; ++i) {
    std::cout << "  [" << i << "] x=" << poly.vertices_xy[i].x
              << ", y=" << poly.vertices_xy[i].y
              << ", z=" << poly.vertices_xy[i].z << "\n";
  }
  if (poly.vertices_xy.size() > max_v) {
    std::cout << "  ... (" << (poly.vertices_xy.size() - max_v) << " more)\n";
  }
}

void DumpRacetrack(const refuel::RacetrackPlan& rt, std::size_t max_ep = 8) {
  std::cout << "RacetrackPlan { length_m=" << rt.length_m
            << ", radius_m=" << rt.radius_m
            << ", orientation_deg=" << rt.orientation_deg
            << ", altitude_m=" << rt.altitude_m
            << ", speed_mps=" << rt.speed_mps << " }\n";
  DumpVec3("  center_xy", rt.center_xy);
  std::cout << "  entrypoints_xy.size=" << rt.entrypoints_xy.size() << "\n";
  const std::size_t n = std::min<std::size_t>(rt.entrypoints_xy.size(), max_ep);
  for (std::size_t i = 0; i < n; ++i) {
    DumpVec3("    ep[" + std::to_string(i) + "]", rt.entrypoints_xy[i]);
  }
  if (rt.entrypoints_xy.size() > max_ep) {
    std::cout << "    ... (" << (rt.entrypoints_xy.size() - max_ep) << " more)\n";
  }
}

void DumpSequence(const refuel::SequencePlan& s) {
  std::cout << "SequencePlan.sequence (0-based receiver index order): [";
  for (std::size_t i = 0; i < s.sequence.size(); ++i) {
    if (i) std::cout << ", ";
    std::cout << s.sequence[i];
  }
  std::cout << "]\n";
  std::cout << "SequencePlan.receiver_to_order (receiver_id -> 1-based order):\n";
  for (const auto& kv : s.receiver_to_order) {
    std::cout << "  " << kv.first << " -> " << kv.second << "\n";
  }
}

void DumpRendezvous(const refuel::RendezvousPlan& rv, std::size_t max_recv = 16) {
  std::cout << "RendezvousPlan.tanker { meeting_altitude_m=" << rv.tanker_meeting_altitude_m
            << ", meeting_speed_mps=" << rv.tanker_meeting_speed_mps
            << ", entry_direction_deg=" << rv.tanker_entry_direction_deg << " }\n";
  DumpVec3("  tanker_entry_point_xy", rv.tanker_entry_point_xy);

  std::cout << "RendezvousPlan.receivers.size=" << rv.receivers.size() << "\n";
  std::size_t count = 0;
  for (const auto& kv : rv.receivers) {
    if (count++ >= max_recv) {
      std::cout << "  ... (" << (rv.receivers.size() - max_recv) << " more)\n";
      break;
    }
    const auto& rr = kv.second;
    std::cout << "  ReceiverRendezvous { id=" << rr.receiver_id
              << ", order=" << rr.sequence_order
              << ", meeting_altitude_m=" << rr.meeting_altitude_m
              << ", meeting_speed_mps=" << rr.meeting_speed_mps
              << ", cycling_number=" << rr.cycling_number << " }\n";
    DumpVec3("    coordinate_xy", rr.coordinate_xy);
    DumpVec3("    waiting_point_xy", rr.waiting_point_xy);
  }
}

void DumpTrajSummary(const refuel::Trajectory& t, std::size_t sample_n = 2) {
  std::cout << "Trajectory { aircraft_id=" << t.aircraft_id
            << ", points.size=" << t.points.size() << " }\n";
  if (t.points.empty()) return;

  auto dump_pt = [](const refuel::WaypointXYZ& p) {
    std::cout << "(x=" << p.x << ", y=" << p.y << ", z=" << p.z << ")";
  };

  std::cout << "  first=";
  dump_pt(t.points.front());
  std::cout << "  last=";
  dump_pt(t.points.back());
  std::cout << "\n";

  const std::size_t n = std::min<std::size_t>(t.points.size(), sample_n);
  for (std::size_t i = 0; i < n; ++i) {
    std::cout << "  sample[" << i << "]=";
    dump_pt(t.points[i]);
    std::cout << "\n";
  }
}

void DumpCost(const refuel::CostReport& c) {
  std::cout << "CostReport { tanker_arrival_time_s=" << c.tanker_arrival_time_s
            << ", tanker_fuel_consumed=" << c.tanker_fuel_consumed
            << ", receiver_total_arrival_time_s=" << c.receiver_total_arrival_time_s
            << ", receiver_total_fuel_consumed=" << c.receiver_total_fuel_consumed
            << " }\n";
  std::cout << "  receiver_arrival_time_s:\n";
  for (const auto& kv : c.receiver_arrival_time_s) {
    std::cout << "    " << kv.first << " -> " << kv.second << "\n";
  }
  std::cout << "  receiver_fuel_consumed:\n";
  for (const auto& kv : c.receiver_fuel_consumed) {
    std::cout << "    " << kv.first << " -> " << kv.second << "\n";
  }
}

bool Test_CoordinateTransformStage_Basic() {
  // 输入：ctx.tanker.initial_position_lla / ctx.receivers[i].initial_position_lla
  // 输出：对应的 ctx.*.initial_position_xy（Mercator XY, 单位 m）
  refuel::PlanningContext ctx;

  ctx.tanker.id = "T";
  ctx.tanker.initial_position_lla = {0.0, 0.0, 5000.0};
  ctx.tanker.current_status.heading_deg = 0.0;
  ctx.tanker.current_status.speed_mps = 200.0;

  refuel::AircraftConfig r1;
  r1.id = "R1";
  r1.initial_position_lla = {0.0, 0.01, 3000.0}; // lon=0.01deg

  refuel::AircraftConfig r2;
  r2.id = "R2";
  r2.initial_position_lla = {0.01, 0.0, 3000.0}; // lat=0.01deg

  ctx.receivers = {r1, r2};

  PrintBanner("Stage: CoordinateTransformStage");
  PrintSub("INPUT");
  DumpLLA("tanker.initial_position_lla", ctx.tanker.initial_position_lla);
  DumpLLA("receiver[0].initial_position_lla", ctx.receivers[0].initial_position_lla);
  DumpLLA("receiver[1].initial_position_lla", ctx.receivers[1].initial_position_lla);

  refuel::CoordinateTransformStage stage;
  stage.Run(ctx);

  PrintSub("OUTPUT");
  DumpVec3("tanker.initial_position_xy", ctx.tanker.initial_position_xy);
  DumpVec3("receiver[0].initial_position_xy", ctx.receivers[0].initial_position_xy);
  DumpVec3("receiver[1].initial_position_xy", ctx.receivers[1].initial_position_xy);

  // tanker at (0,0)
  REFUEL_EXPECT_NEAR(ctx.tanker.initial_position_xy.x, 0.0, 1e-9);
  REFUEL_EXPECT_NEAR(ctx.tanker.initial_position_xy.y, 0.0, 1e-9);
  // CoordinateTransformStage 会把输入的高度写入 xy.z（便于后续阶段直接引用高度）。
  REFUEL_EXPECT_NEAR(ctx.tanker.initial_position_xy.z, 5000.0, 1e-9);

  // Stage uses R = 6371000.0
  const double R = 6371000.0;
  const double expect_r1_x = R * Deg2Rad(0.01);
  std::cout << "Expected receiver[0].x ~= R*deg2rad(0.01) = " << expect_r1_x << "\n";
  REFUEL_EXPECT_NEAR(ctx.receivers[0].initial_position_xy.x, expect_r1_x, 1e-3);
  REFUEL_EXPECT_NEAR(ctx.receivers[0].initial_position_xy.y, 0.0, 1e-6);
  REFUEL_EXPECT_NEAR(ctx.receivers[0].initial_position_xy.z, 3000.0, 1e-9);

  // lat=0.01deg => y = R*log(tan(pi/4+lat/2))
  const double lat_rad = Deg2Rad(0.01);
  const double expect_r2_y = R * std::log(std::tan(kPi / 4.0 + lat_rad / 2.0));
  std::cout << "Expected receiver[1].y ~= R*log(tan(pi/4+lat/2)) = " << expect_r2_y << "\n";
  REFUEL_EXPECT_NEAR(ctx.receivers[1].initial_position_xy.x, 0.0, 1e-6);
  REFUEL_EXPECT_NEAR(ctx.receivers[1].initial_position_xy.y, expect_r2_y, 1e-3);

  return true;
}

bool Test_PreprocessStage_SpeedBoundsFilled() {
  // 输入：ctx.receivers（至少需要 id 列表）
  // 输出：ctx.tanker_speed_bounds 与 ctx.receiver_speed_bounds（map: receiver_id -> SpeedBounds）
  refuel::PlanningContext ctx;
  ctx.tanker.id = "T";
  ctx.receivers = {refuel::AircraftConfig{"R1"}, refuel::AircraftConfig{"R2"}};

  PrintBanner("Stage: PreprocessStage");
  PrintSub("INPUT");
  std::cout << "tanker.id=" << ctx.tanker.id << "\n";
  std::cout << "receivers.size=" << ctx.receivers.size() << "\n";
  for (std::size_t i = 0; i < ctx.receivers.size(); ++i) {
    std::cout << "  receiver[" << i << "].id=" << ctx.receivers[i].id << "\n";
  }

  refuel::PreprocessStage stage;
  stage.Run(ctx);

  PrintSub("OUTPUT");
  DumpSpeedBounds("ctx.tanker_speed_bounds", ctx.tanker_speed_bounds);
  for (const auto& kv : ctx.receiver_speed_bounds) {
    DumpSpeedBounds("ctx.receiver_speed_bounds['" + kv.first + "']", kv.second);
  }

  REFUEL_EXPECT_NEAR(ctx.tanker_speed_bounds.min_speed_mps, 120.0, 1e-9);
  REFUEL_EXPECT_NEAR(ctx.tanker_speed_bounds.cruise_speed_mps, 150.0, 1e-9);
  REFUEL_EXPECT_NEAR(ctx.tanker_speed_bounds.max_speed_mps, 300.0, 1e-9);

  REFUEL_EXPECT_EQ(static_cast<int>(ctx.receiver_speed_bounds.size()), 2);
  REFUEL_EXPECT_TRUE(ctx.receiver_speed_bounds.count("R1") == 1);
  REFUEL_EXPECT_TRUE(ctx.receiver_speed_bounds.count("R2") == 1);
  REFUEL_EXPECT_NEAR(ctx.receiver_speed_bounds["R1"].cruise_speed_mps, 170.0, 1e-9);
  return true;
}

bool Test_SafeZoneSelectStage_SelectAndConvert() {
  // 输入：ctx.mission.refuel_mode.safe_zone（SafeZoneInput: is_defined + zones_vertices_lla）
  // 输出：ctx.safe_zone_selected.{chosen_safezone_id, safe_zone_polygon_xy.vertices_xy}
  refuel::PlanningContext ctx;

  PrintBanner("Stage: SafeZoneSelectStage");

  // Case A: no defined safe zone
  {
    refuel::SafeZoneInput z;
    z.is_defined = false;
    PrintSub("CASE A INPUT");
    std::cout << "safe_zone.is_defined=" << (z.is_defined ? "true" : "false") << "\n";
    ctx.mission.refuel_mode.safe_zone = z;
    refuel::SafeZoneSelectStage s;
    s.Run(ctx);
    PrintSub("CASE A OUTPUT");
    std::cout << "chosen_safezone_id=" << ctx.safe_zone_selected.chosen_safezone_id << "\n";
    DumpPolygonXY("safe_zone_polygon_xy", ctx.safe_zone_selected.safe_zone_polygon_xy);
    REFUEL_EXPECT_EQ(ctx.safe_zone_selected.chosen_safezone_id, -1);
    REFUEL_EXPECT_TRUE(ctx.safe_zone_selected.safe_zone_polygon_xy.vertices_xy.empty());
  }

  // Case B: first defined safe zone selected (id=1), and LLA->XY conversion works
  {
    refuel::SafeZoneInput z;
    z.is_defined = true;
    // A small rectangle near the equator to keep expectations stable.
    z.zones_vertices_lla = {{
        {0.00, 0.00, 0.0},
        {0.00, 0.01, 0.0},
        {0.01, 0.01, 0.0},
        {0.01, 0.00, 0.0},
    }};
    PrintSub("CASE B INPUT");
    std::cout << "safe_zone.is_defined=" << (z.is_defined ? "true" : "false") << "\n";
    std::cout << "zones_vertices_lla.size=" << z.zones_vertices_lla.size() << "\n";
    for (std::size_t i = 0; i < z.zones_vertices_lla[0].size(); ++i) {
      DumpLLA("  vlla[0][" + std::to_string(i) + "]", z.zones_vertices_lla[0][i]);
    }
    ctx.mission.refuel_mode.safe_zone = z;

    refuel::SafeZoneSelectStage s;
    s.Run(ctx);

    PrintSub("CASE B OUTPUT");
    std::cout << "chosen_safezone_id=" << ctx.safe_zone_selected.chosen_safezone_id << "\n";
    DumpPolygonXY("safe_zone_polygon_xy", ctx.safe_zone_selected.safe_zone_polygon_xy);

    REFUEL_EXPECT_EQ(ctx.safe_zone_selected.chosen_safezone_id, 1);
    const auto& v = ctx.safe_zone_selected.safe_zone_polygon_xy.vertices_xy;
    REFUEL_EXPECT_EQ(static_cast<int>(v.size()), 4);

    // Stage uses R = 6378137.0
    const double R = 6378137.0;
    const double expect_x01 = R * Deg2Rad(0.01);
    REFUEL_EXPECT_NEAR(v[0].x, 0.0, 1e-6);
    REFUEL_EXPECT_NEAR(v[0].y, 0.0, 1e-6);
    REFUEL_EXPECT_NEAR(v[1].x, expect_x01, 1e-2);
    REFUEL_EXPECT_NEAR(v[1].y, 0.0, 1e-6);

    // Just sanity for the lat=0.01 vertex: must be finite and y>0.
    REFUEL_EXPECT_TRUE(IsFinite(v[2].y));
    REFUEL_EXPECT_TRUE(v[2].y > 0.0);
  }

  return true;
}

bool Test_RacetrackBuildStage_GeneratesEntrypoints() {
  // 输入：ctx.safe_zone_selected.safe_zone_polygon_xy + ctx.tanker / ctx.mission
  // 输出：ctx.racetrack（center/length/radius/orientation/alt/speed + entrypoints_xy）
  refuel::PlanningContext ctx;

  ctx.safe_zone_selected.chosen_safezone_id = 1;
  ctx.safe_zone_selected.safe_zone_polygon_xy = MakeSquarePolygon(25000.0);

  ctx.tanker = MakeAircraft("T", 0.0, -50000.0, /*alt*/6000.0, /*speed*/200.0, /*heading*/0.0);

  // Mission-side inputs used by RacetrackBuildStage (via detection idioms):
  ctx.mission.racecourse.orbit_radius = 8000.0;
  ctx.mission.racecourse.entry_angle_deg = 45.0;
  ctx.mission.racecourse.turn_direction = "clockwise";

  PrintBanner("Stage: RacetrackBuildStage");
  PrintSub("INPUT");
  DumpPolygonXY("safe_zone_polygon_xy", ctx.safe_zone_selected.safe_zone_polygon_xy);
  DumpVec3("tanker.initial_position_xy", ctx.tanker.initial_position_xy);
  DumpLLA("tanker.initial_position_lla", ctx.tanker.initial_position_lla);
  std::cout << "mission.racecourse.orbit_radius=" << ctx.mission.racecourse.orbit_radius << " (m)\n";
  std::cout << "mission.racecourse.entry_angle_deg=" << ctx.mission.racecourse.entry_angle_deg << " (deg)\n";
  std::cout << "mission.racecourse.turn_direction=" << ctx.mission.racecourse.turn_direction << "\n";

  refuel::RacetrackBuildStage stage;
  stage.Run(ctx);

  PrintSub("OUTPUT");
  DumpRacetrack(ctx.racetrack);

  REFUEL_EXPECT_TRUE(ctx.racetrack.radius_m > 0.0);
  REFUEL_EXPECT_TRUE(ctx.racetrack.length_m >= 0.0);
  REFUEL_EXPECT_TRUE(IsFinite(ctx.racetrack.orientation_deg));
  REFUEL_EXPECT_TRUE(ctx.racetrack.altitude_m > 0.0);
  REFUEL_EXPECT_TRUE(ctx.racetrack.speed_mps > 0.0);

  // Alt/speed should follow tanker defaults in the current skeleton implementation.
  REFUEL_EXPECT_NEAR(ctx.racetrack.altitude_m, 6000.0, 1e-9);
  REFUEL_EXPECT_NEAR(ctx.racetrack.speed_mps, 200.0, 1e-9);

  // 约束更稳健：跑马场中心至少应落在安全区 polygon 的包围盒内（或非常接近）。
  // 这里安全区是 [-25000, +25000] 的正方形。
  REFUEL_EXPECT_TRUE(std::fabs(ctx.racetrack.center_xy.x) <= 25000.0 + 1.0);
  REFUEL_EXPECT_TRUE(std::fabs(ctx.racetrack.center_xy.y) <= 25000.0 + 1.0);

  // Entrypoints must be non-empty; each must use meeting altitude on z.
  REFUEL_EXPECT_TRUE(!ctx.racetrack.entrypoints_xy.empty());
  for (const auto& ep : ctx.racetrack.entrypoints_xy) {
    REFUEL_EXPECT_NEAR(ep.z, ctx.racetrack.altitude_m, 1e-9);
  }
  return true;
}

bool Test_SequenceStage_OrdersByDistance() {
  // 输入：ctx.receivers[*].initial_position_xy + ctx.racetrack.center_xy
  // 输出：ctx.sequence.sequence(0-based index list) + ctx.sequence.receiver_to_order(map id->rank)
  refuel::PlanningContext ctx;

  ctx.racetrack.center_xy = {0.0, 0.0, 0.0};
  ctx.receivers = {
      MakeAircraft("R0", 1000.0, 0.0, 0.0, 0.0, 0.0),
      MakeAircraft("R1", 3000.0, 0.0, 0.0, 0.0, 0.0),
      MakeAircraft("R2", 2000.0, 0.0, 0.0, 0.0, 0.0),
  };

  PrintBanner("Stage: SequenceStage");
  PrintSub("INPUT");
  DumpVec3("racetrack.center_xy", ctx.racetrack.center_xy);
  for (std::size_t i = 0; i < ctx.receivers.size(); ++i) {
    DumpVec3("receiver[" + std::to_string(i) + "].initial_position_xy", ctx.receivers[i].initial_position_xy);
  }

  refuel::SequenceStage s;
  s.Run(ctx);

  PrintSub("OUTPUT");
  DumpSequence(ctx.sequence);

  REFUEL_EXPECT_EQ(static_cast<int>(ctx.sequence.sequence.size()), 3);
  // Distances are unique so order should be deterministic: 1000 -> 2000 -> 3000
  REFUEL_EXPECT_EQ(ctx.sequence.sequence[0], 0);
  REFUEL_EXPECT_EQ(ctx.sequence.sequence[1], 2);
  REFUEL_EXPECT_EQ(ctx.sequence.sequence[2], 1);

  REFUEL_EXPECT_EQ(static_cast<int>(ctx.sequence.receiver_to_order.size()), 3);
  REFUEL_EXPECT_EQ(ctx.sequence.receiver_to_order["R0"], 1);
  REFUEL_EXPECT_EQ(ctx.sequence.receiver_to_order["R2"], 2);
  REFUEL_EXPECT_EQ(ctx.sequence.receiver_to_order["R1"], 3);
  return true;
}

bool Test_TankerRacetrackDecisionStage_DefaultOffTrack() {
  // 由于 SequenceStage::IsTankerOnRacetrack 目前是 TODO 并默认返回 false，
  // 因此该 Stage 当前“应当”输出：tanker_on_racetrack=false, need_transfer=true, reduce_rendezvous=false。
  refuel::PlanningContext ctx;
  ctx.tanker = MakeAircraft("T", 0.0, 0.0, 6000.0, 200.0, 10.0);
  ctx.racetrack.center_xy = {0.0, 0.0, 0.0};

  PrintBanner("Stage: TankerRacetrackDecisionStage");
  PrintSub("INPUT");
  DumpVec3("tanker.initial_position_xy", ctx.tanker.initial_position_xy);
  std::cout << "tanker.heading_deg=" << ctx.tanker.current_status.heading_deg << "\n";
  DumpVec3("racetrack.center_xy", ctx.racetrack.center_xy);

  refuel::TankerRacetrackDecisionStage s;
  s.Run(ctx);

  PrintSub("OUTPUT");
  std::cout << "tanker_on_racetrack=" << (ctx.tanker_rt_decision.tanker_on_racetrack ? "true" : "false") << "\n";
  std::cout << "need_transfer=" << (ctx.tanker_rt_decision.need_transfer ? "true" : "false") << "\n";
  std::cout << "reduce_rendezvous=" << (ctx.tanker_rt_decision.reduce_rendezvous ? "true" : "false") << "\n";

  REFUEL_EXPECT_TRUE(ctx.tanker_rt_decision.tanker_on_racetrack == false);
  REFUEL_EXPECT_TRUE(ctx.tanker_rt_decision.need_transfer == true);
  REFUEL_EXPECT_TRUE(ctx.tanker_rt_decision.reduce_rendezvous == false);
  return true;
}

bool Test_RendezvousStage_Mode0_FillsPlans() {
  // 输入（最小可跑集合）：
  // - ctx.tanker / ctx.receivers: initial_position_xy, alt, heading
  // - ctx.tanker_speed_bounds / ctx.receiver_speed_bounds
  // - ctx.racetrack.entrypoints_xy + altitude_m + radius_m + length_m
  // - ctx.mission.refuel_mode.mode_type
  // 输出：
  // - ctx.rendezvous.tanker_entry_point_xy / tanker_entry_direction_deg
  // - ctx.rendezvous.tanker_meeting_altitude_m / tanker_meeting_speed_mps
  // - ctx.rendezvous.receivers[rid] (coordinate_xy, waiting_point_xy, meeting_altitude_m, meeting_speed_mps, cycling_number)
  refuel::PlanningContext ctx;

  ctx.mission.refuel_mode.mode_type = 0;
  ctx.mission.prefs.primary_pref = 0;
  ctx.mission.racecourse.turn_direction = "clockwise";
  ctx.mission.racecourse.orbit_radius = 5000.0;

  ctx.tanker = MakeAircraft("T", -20000.0, 0.0, 7000.0, 150.0, 90.0);
  ctx.tanker_speed_bounds = {100.0, 150.0, 200.0};

  ctx.receivers = {
      MakeAircraft("R1", 1000.0, 0.0, 5000.0, 0.0, 0.0),
      MakeAircraft("R2", 0.0, 1000.0, 5000.0, 0.0, 0.0),
  };
  ctx.receiver_speed_bounds["R1"] = {80.0, 200.0, 250.0};
  ctx.receiver_speed_bounds["R2"] = {80.0, 200.0, 250.0};

  ctx.racetrack.center_xy = {0.0, 0.0, 0.0};
  ctx.racetrack.radius_m = 5000.0;
  ctx.racetrack.length_m = 10000.0;
  ctx.racetrack.orientation_deg = 0.0;
  ctx.racetrack.altitude_m = 7000.0;
  ctx.racetrack.speed_mps = 150.0;
  ctx.racetrack.entrypoints_xy = {
      {5000.0, 0.0, 7000.0},
      {0.0, 5000.0, 7000.0},
      {-5000.0, 0.0, 7000.0},
      {0.0, -5000.0, 7000.0},
  };

  // Provide a stable sequence order map (used only for rr.sequence_order bookkeeping here)
  ctx.sequence.receiver_to_order["R1"] = 1;
  ctx.sequence.receiver_to_order["R2"] = 2;

  // Default decision branch
  ctx.tanker_rt_decision.tanker_on_racetrack = false;
  ctx.tanker_rt_decision.need_transfer = true;
  ctx.tanker_rt_decision.reduce_rendezvous = false;

  PrintBanner("Stage: RendezvousStage (mode0)");
  PrintSub("INPUT");
  std::cout << "mission.refuel_mode.mode_type=" << ctx.mission.refuel_mode.mode_type << "\n";
  std::cout << "mission.prefs.primary_pref=" << ctx.mission.prefs.primary_pref << "\n";
  std::cout << "mission.racecourse.turn_direction=" << ctx.mission.racecourse.turn_direction << "\n";
  std::cout << "mission.racecourse.orbit_radius=" << ctx.mission.racecourse.orbit_radius << " (m)\n";
  DumpVec3("tanker.initial_position_xy", ctx.tanker.initial_position_xy);
  DumpSpeedBounds("tanker_speed_bounds", ctx.tanker_speed_bounds);
  for (std::size_t i = 0; i < ctx.receivers.size(); ++i) {
    DumpVec3("receiver[" + std::to_string(i) + "].initial_position_xy", ctx.receivers[i].initial_position_xy);
  }
  for (const auto& kv : ctx.receiver_speed_bounds) {
    DumpSpeedBounds("receiver_speed_bounds['" + kv.first + "']", kv.second);
  }
  DumpRacetrack(ctx.racetrack);
  std::cout << "sequence.receiver_to_order (input):\n";
  for (const auto& kv : ctx.sequence.receiver_to_order) {
    std::cout << "  " << kv.first << " -> " << kv.second << "\n";
  }
  std::cout << "tanker_rt_decision: tanker_on_racetrack=" << (ctx.tanker_rt_decision.tanker_on_racetrack ? "true" : "false")
            << ", need_transfer=" << (ctx.tanker_rt_decision.need_transfer ? "true" : "false")
            << ", reduce_rendezvous=" << (ctx.tanker_rt_decision.reduce_rendezvous ? "true" : "false") << "\n";

  refuel::RendezvousStage s;
  s.Run(ctx);

  PrintSub("OUTPUT");
  DumpRendezvous(ctx.rendezvous);

  // Tanker rendezvous fields
  REFUEL_EXPECT_TRUE(IsFinite(ctx.rendezvous.tanker_entry_direction_deg));
  REFUEL_EXPECT_TRUE(ctx.rendezvous.tanker_entry_direction_deg >= 0.0);
  REFUEL_EXPECT_TRUE(ctx.rendezvous.tanker_entry_direction_deg < 360.0);
  REFUEL_EXPECT_TRUE(std::fabs(ctx.rendezvous.tanker_entry_point_xy.x) > 1e-6 ||
                     std::fabs(ctx.rendezvous.tanker_entry_point_xy.y) > 1e-6);

  REFUEL_EXPECT_TRUE(ctx.rendezvous.tanker_meeting_altitude_m > 0.0);
  REFUEL_EXPECT_TRUE(ctx.rendezvous.tanker_meeting_speed_mps > 0.0);
  REFUEL_EXPECT_TRUE(ctx.rendezvous.tanker_meeting_speed_mps >= ctx.tanker_speed_bounds.min_speed_mps - 1e-9);
  REFUEL_EXPECT_TRUE(ctx.rendezvous.tanker_meeting_speed_mps <= ctx.tanker_speed_bounds.max_speed_mps + 1e-9);

  // Per-receiver rendezvous map should include both receivers
  REFUEL_EXPECT_EQ(static_cast<int>(ctx.rendezvous.receivers.size()), 2);
  for (const auto& rid : {std::string("R1"), std::string("R2")}) {
    REFUEL_EXPECT_TRUE(ctx.rendezvous.receivers.count(rid) == 1);
    const auto& rr = ctx.rendezvous.receivers.at(rid);

    REFUEL_EXPECT_EQ(rr.receiver_id, rid);
    REFUEL_EXPECT_TRUE(rr.meeting_altitude_m > 0.0);
    REFUEL_EXPECT_TRUE(rr.meeting_speed_mps > 0.0);
    // meeting_speed should be within receiver bounds
    const auto& b = ctx.receiver_speed_bounds.at(rid);
    REFUEL_EXPECT_TRUE(rr.meeting_speed_mps >= b.min_speed_mps - 1e-9);
    REFUEL_EXPECT_TRUE(rr.meeting_speed_mps <= b.max_speed_mps + 1e-9);
    // coordinate/waiting points are XY (z ignored in this skeleton) but should be finite.
    REFUEL_EXPECT_TRUE(IsFinite(rr.coordinate_xy.x) && IsFinite(rr.coordinate_xy.y));
    REFUEL_EXPECT_TRUE(IsFinite(rr.waiting_point_xy.x) && IsFinite(rr.waiting_point_xy.y));
    REFUEL_EXPECT_TRUE(rr.cycling_number >= 0);
  }
  return true;
}

bool Test_RendezvousStage_ReduceRendezvous_OverridesEntry() {
  // reduce_rendezvous=true: 期望 entry_point/entry_direction 被覆盖为“加油机当前位置/当前航向”。
  refuel::PlanningContext ctx;

  ctx.mission.refuel_mode.mode_type = 0;
  ctx.mission.prefs.primary_pref = 0;
  ctx.mission.racecourse.turn_direction = "clockwise";
  ctx.mission.racecourse.orbit_radius = 5000.0;

  ctx.tanker = MakeAircraft("T", -1234.0, 5678.0, 7000.0, 150.0, 33.0);
  ctx.tanker_speed_bounds = {100.0, 150.0, 200.0};

  ctx.receivers = {MakeAircraft("R1", 1000.0, 0.0, 5000.0, 0.0, 0.0)};
  ctx.receiver_speed_bounds["R1"] = {80.0, 200.0, 250.0};

  ctx.racetrack.center_xy = {0.0, 0.0, 0.0};
  ctx.racetrack.radius_m = 5000.0;
  ctx.racetrack.length_m = 10000.0;
  ctx.racetrack.orientation_deg = 0.0;
  ctx.racetrack.altitude_m = 7000.0;
  ctx.racetrack.speed_mps = 150.0;
  ctx.racetrack.entrypoints_xy = {
      {5000.0, 0.0, 7000.0},
      {0.0, 5000.0, 7000.0},
      {-5000.0, 0.0, 7000.0},
      {0.0, -5000.0, 7000.0},
  };

  ctx.sequence.receiver_to_order["R1"] = 1;

  ctx.tanker_rt_decision.reduce_rendezvous = true;
  ctx.tanker_rt_decision.tanker_on_racetrack = true;
  ctx.tanker_rt_decision.need_transfer = false;

  PrintBanner("Stage: RendezvousStage (reduce_rendezvous)");
  PrintSub("INPUT");
  DumpVec3("tanker.initial_position_xy", ctx.tanker.initial_position_xy);
  std::cout << "tanker.heading_deg=" << ctx.tanker.current_status.heading_deg << "\n";
  std::cout << "reduce_rendezvous=true (expect entry_point/entry_direction overridden by tanker current state)\n";
  DumpRacetrack(ctx.racetrack);

  refuel::RendezvousStage s;
  s.Run(ctx);

  PrintSub("OUTPUT");
  DumpRendezvous(ctx.rendezvous);

  REFUEL_EXPECT_NEAR(ctx.rendezvous.tanker_entry_point_xy.x, ctx.tanker.initial_position_xy.x, 1e-9);
  REFUEL_EXPECT_NEAR(ctx.rendezvous.tanker_entry_point_xy.y, ctx.tanker.initial_position_xy.y, 1e-9);
  REFUEL_EXPECT_NEAR(ctx.rendezvous.tanker_entry_direction_deg, ctx.tanker.current_status.heading_deg, 1e-9);
  return true;
}

bool Test_PathPlanningStage_GeneratesTrajectories() {
  // PathPlanningStage 主要消费：
  // - ctx.racetrack (center/length/radius/orientation)
  // - ctx.rendezvous (tanker_entry_point/dir, tanker meet alt/speed, per-receiver waiting/coordinate/cycling)
  // 输出：ctx.trajectories (vector<Trajectory>)
  refuel::PlanningContext ctx;

  ctx.mission.racecourse.turn_direction = "clockwise";
  ctx.tanker = MakeAircraft("T", -20000.0, 0.0, 7000.0, 150.0, 90.0);
  ctx.tanker_rt_decision.tanker_on_racetrack = false;
  ctx.tanker_rt_decision.need_transfer = true;
  ctx.tanker_rt_decision.reduce_rendezvous = false;

  ctx.receivers = {
      MakeAircraft("R1", 1000.0, 0.0, 5000.0, 0.0, 0.0),
      MakeAircraft("R2", 0.0, 1000.0, 5000.0, 0.0, 0.0),
  };

  ctx.racetrack.center_xy = {0.0, 0.0, 0.0};
  ctx.racetrack.radius_m = 5000.0;
  ctx.racetrack.length_m = 10000.0;
  ctx.racetrack.orientation_deg = 0.0;
  ctx.racetrack.altitude_m = 7000.0;
  ctx.racetrack.speed_mps = 150.0;

  ctx.rendezvous.tanker_entry_point_xy = {5000.0, 0.0, 0.0};
  ctx.rendezvous.tanker_entry_direction_deg = 0.0;
  ctx.rendezvous.tanker_meeting_altitude_m = 7000.0;
  ctx.rendezvous.tanker_meeting_speed_mps = 150.0;

  {
    refuel::RendezvousPlan::ReceiverRendezvous rr;
    rr.receiver_id = "R1";
    rr.sequence_order = 1;
    rr.meeting_altitude_m = 7000.0;
    rr.meeting_speed_mps = 150.0;
    rr.waiting_point_xy = {0.0, 5000.0, 0.0};
    rr.coordinate_xy = {-5000.0, 0.0, 0.0};
    rr.cycling_number = 2;
    ctx.rendezvous.receivers["R1"] = rr;
  }
  {
    refuel::RendezvousPlan::ReceiverRendezvous rr;
    rr.receiver_id = "R2";
    rr.sequence_order = 2;
    rr.meeting_altitude_m = 7000.0;
    rr.meeting_speed_mps = 150.0;
    rr.waiting_point_xy = {0.0, -5000.0, 0.0};
    rr.coordinate_xy = {0.0, 5000.0, 0.0};
    rr.cycling_number = 3; // max
    ctx.rendezvous.receivers["R2"] = rr;
  }

  PrintBanner("Stage: PathPlanningStage");
  PrintSub("INPUT");
  std::cout << "mission.racecourse.turn_direction=" << ctx.mission.racecourse.turn_direction << "\n";
  std::cout << "tanker_rt_decision: tanker_on_racetrack=" << (ctx.tanker_rt_decision.tanker_on_racetrack ? "true" : "false")
            << ", need_transfer=" << (ctx.tanker_rt_decision.need_transfer ? "true" : "false")
            << ", reduce_rendezvous=" << (ctx.tanker_rt_decision.reduce_rendezvous ? "true" : "false") << "\n";
  DumpRacetrack(ctx.racetrack);
  DumpRendezvous(ctx.rendezvous);

  refuel::PathPlanningStage s;
  s.Run(ctx);

  PrintSub("OUTPUT");
  std::cout << "ctx.trajectories.size=" << ctx.trajectories.size() << "\n";
  for (const auto& t : ctx.trajectories) {
    DumpTrajSummary(t);
  }

  // Expect 1 tanker + 2 receivers
  REFUEL_EXPECT_EQ(static_cast<int>(ctx.trajectories.size()), 3);

  // Find tanker trajectory and sanity-check length (loops should make it long).
  const refuel::Trajectory* tanker_traj = nullptr;
  for (const auto& t : ctx.trajectories) {
    if (t.aircraft_id == "T") tanker_traj = &t;
    REFUEL_EXPECT_TRUE(!t.points.empty());
    for (const auto& p : t.points) {
      REFUEL_EXPECT_TRUE(IsFinite(p.x) && IsFinite(p.y) && IsFinite(p.z));
    }
  }
  REFUEL_EXPECT_TRUE(tanker_traj != nullptr);
  // With radius=5000,length=10000 and 3 loops, points should be plenty.
  REFUEL_EXPECT_TRUE(static_cast<int>(tanker_traj->points.size()) > 200);
  // First point is near tanker start (transfer begins at start)
  REFUEL_EXPECT_TRUE(NearlyEqual(tanker_traj->points.front().x, ctx.tanker.initial_position_xy.x, 1e-6));
  REFUEL_EXPECT_TRUE(NearlyEqual(tanker_traj->points.front().y, ctx.tanker.initial_position_xy.y, 1e-6));
  return true;
}

bool Test_CostStage_PopulatesMapsAndTotals() {
  // 当前 CostStage 是占位逻辑：
  // - 为每个 receiver 写入 arrival_time_s/fuel_consumed 的 map entry
  // - total_arrival_time_s = max(arrival_time_s)
  // - total_fuel_consumed  = sum(fuel)
  refuel::PlanningContext ctx;
  ctx.tanker.id = "T";
  ctx.receivers = {refuel::AircraftConfig{"R1"}, refuel::AircraftConfig{"R2"}};

  PrintBanner("Stage: CostStage");
  PrintSub("INPUT");
  std::cout << "tanker.id=" << ctx.tanker.id << "\n";
  std::cout << "receivers.size=" << ctx.receivers.size() << "\n";
  for (std::size_t i = 0; i < ctx.receivers.size(); ++i) {
    std::cout << "  receiver[" << i << "].id=" << ctx.receivers[i].id << "\n";
  }

  refuel::CostStage s;
  s.Run(ctx);

  PrintSub("OUTPUT");
  DumpCost(ctx.cost);

  REFUEL_EXPECT_EQ(static_cast<int>(ctx.cost.receiver_arrival_time_s.size()), 2);
  REFUEL_EXPECT_EQ(static_cast<int>(ctx.cost.receiver_fuel_consumed.size()), 2);
  REFUEL_EXPECT_TRUE(ctx.cost.receiver_arrival_time_s.count("R1") == 1);
  REFUEL_EXPECT_TRUE(ctx.cost.receiver_arrival_time_s.count("R2") == 1);
  REFUEL_EXPECT_NEAR(ctx.cost.receiver_total_arrival_time_s, 0.0, 1e-9);
  REFUEL_EXPECT_NEAR(ctx.cost.receiver_total_fuel_consumed, 0.0, 1e-9);
  return true;
}

bool Test_Pipeline_EndToEnd_Smoke() {
  // 端到端冒烟测试：确保 Pipeline 能把最小输入跑完并产生关键输出。
  //
  // 注意：当前 skeleton 的 CoordinateTransformStage 使用 R=6371000，
  // SafeZoneSelectStage 使用 R=6378137。二者略有不一致会带来小尺度误差。
  // 本测试只检查“结构性输出”（非空、有限、尺寸合理），避免对数值过拟合。
  refuel::PlanningContext ctx;

  // Mission: one safe zone (defined)
  refuel::SafeZoneInput z;
  z.is_defined = true;
  z.zones_vertices_lla = {{
      {0.00, 0.00, 0.0},
      {0.00, 0.20, 0.0},
      {0.20, 0.20, 0.0},
      {0.20, 0.00, 0.0},
  }};
  ctx.mission.refuel_mode.safe_zone = z;

  ctx.mission.refuel_mode.mode_type = 0;
  ctx.mission.prefs.primary_pref = 0;
  ctx.mission.racecourse.orbit_radius = 8000.0;
  ctx.mission.racecourse.turn_direction = "clockwise";
  ctx.mission.racecourse.entry_angle_deg = 30.0;

  // Tanker/receivers in LLA; CoordinateTransformStage will convert.
  ctx.tanker.id = "T";
  ctx.tanker.initial_position_lla = {0.05, -0.30, 7000.0};
  ctx.tanker.current_status.heading_deg = 45.0;
  ctx.tanker.current_status.speed_mps = 160.0;
  ctx.tanker.limits.climb_rate = 5.0;
  ctx.tanker.limits.descent_rate = 5.0;

  refuel::AircraftConfig r1;
  r1.id = "R1";
  r1.initial_position_lla = {0.05, 0.05, 5000.0};
  r1.limits.climb_rate = 5.0;
  r1.limits.descent_rate = 5.0;

  refuel::AircraftConfig r2;
  r2.id = "R2";
  r2.initial_position_lla = {0.10, 0.05, 5000.0};
  r2.limits.climb_rate = 5.0;
  r2.limits.descent_rate = 5.0;

  ctx.receivers = {r1, r2};

  PrintBanner("Pipeline: end-to-end smoke");
  PrintSub("INPUT");
  std::cout << "mission.refuel_mode.mode_type=" << ctx.mission.refuel_mode.mode_type << "\n";
  std::cout << "safe_zone.is_defined=" << (ctx.mission.refuel_mode.safe_zone.is_defined ? "true" : "false") << "\n";
  std::cout << "safe_zone.zones_vertices_lla.size=" << ctx.mission.refuel_mode.safe_zone.zones_vertices_lla.size() << "\n";
  if (!ctx.mission.refuel_mode.safe_zone.zones_vertices_lla.empty()) {
    for (std::size_t i = 0; i < ctx.mission.refuel_mode.safe_zone.zones_vertices_lla[0].size(); ++i) {
      DumpLLA("  safe_zone.vlla[0][" + std::to_string(i) + "]", ctx.mission.refuel_mode.safe_zone.zones_vertices_lla[0][i]);
    }
  }
  DumpLLA("tanker.initial_position_lla", ctx.tanker.initial_position_lla);
  std::cout << "tanker.heading_deg=" << ctx.tanker.current_status.heading_deg << ", speed_mps=" << ctx.tanker.current_status.speed_mps << "\n";
  for (std::size_t i = 0; i < ctx.receivers.size(); ++i) {
    DumpLLA("receiver[" + std::to_string(i) + "].initial_position_lla", ctx.receivers[i].initial_position_lla);
  }

  // Run
  refuel::Pipeline p;
  p.Run(ctx);

  PrintSub("OUTPUT (key fields)");
  std::cout << "safe_zone_selected.chosen_safezone_id=" << ctx.safe_zone_selected.chosen_safezone_id << "\n";
  DumpPolygonXY("safe_zone_selected.safe_zone_polygon_xy", ctx.safe_zone_selected.safe_zone_polygon_xy);
  DumpRacetrack(ctx.racetrack);
  DumpRendezvous(ctx.rendezvous);
  std::cout << "trajectories.size=" << ctx.trajectories.size() << "\n";
  for (const auto& t : ctx.trajectories) {
    DumpTrajSummary(t);
  }

  // Key outputs
  REFUEL_EXPECT_TRUE(ctx.safe_zone_selected.chosen_safezone_id == 1);
  REFUEL_EXPECT_TRUE(!ctx.racetrack.entrypoints_xy.empty());
  REFUEL_EXPECT_TRUE(!ctx.rendezvous.receivers.empty());
  REFUEL_EXPECT_TRUE(!ctx.trajectories.empty());

  // Trajectories should include tanker
  bool has_t = false;
  for (const auto& t : ctx.trajectories) {
    if (t.aircraft_id == "T") has_t = true;
    REFUEL_EXPECT_TRUE(!t.points.empty());
  }
  REFUEL_EXPECT_TRUE(has_t);
  return true;
}

} // namespace

int main() {
  using refuel::test::TestCase;

  std::vector<TestCase> cases = {
      {"CoordinateTransformStage: LLA->XY conversion", Test_CoordinateTransformStage_Basic},
      {"PreprocessStage: speed bounds populated", Test_PreprocessStage_SpeedBoundsFilled},
      {"SafeZoneSelectStage: selection + LLA->XY", Test_SafeZoneSelectStage_SelectAndConvert},
      {"RacetrackBuildStage: build racetrack + entrypoints", Test_RacetrackBuildStage_GeneratesEntrypoints},
      {"SequenceStage: ordering by distance", Test_SequenceStage_OrdersByDistance},
      {"TankerRacetrackDecisionStage: default off-track branch", Test_TankerRacetrackDecisionStage_DefaultOffTrack},
      {"RendezvousStage(mode0): fills rendezvous plan", Test_RendezvousStage_Mode0_FillsPlans},
      {"RendezvousStage: reduce_rendezvous overrides entry", Test_RendezvousStage_ReduceRendezvous_OverridesEntry},
      {"PathPlanningStage: trajectories generated", Test_PathPlanningStage_GeneratesTrajectories},
      {"CostStage: maps + totals populated", Test_CostStage_PopulatesMapsAndTotals},
      {"Pipeline: end-to-end smoke", Test_Pipeline_EndToEnd_Smoke},
  };

  return refuel::test::RunAll(cases);
}
