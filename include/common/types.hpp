#pragma once
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace refuel {

// ========================
// 1) 基础数学/几何类型
// ========================

struct LLA {
  double lat_deg{0.0};
  double lon_deg{0.0};
  double alt_m{0.0};
};

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Polygon {
  std::vector<Vec3> vertices_xy;
};

// ========================
// 2) 表格数据（输入）
// ========================

struct CruisePerformanceTable {
  std::vector<double> altitude_levels;
  std::vector<double> speed_levels;
  std::vector<std::vector<double>> true_airspeed_data;
};

struct FuelConsumptionTable {
  std::vector<double> altitude_levels;
  std::vector<double> speed_levels;
  std::vector<std::vector<double>> consumption_data;
};

// ========================
// 3) 飞机状态/限制（输入）
// ========================

struct PerformanceLimits {
  double climb_rate{0.0};
  double descent_rate{0.0};
  double max_altitude{0.0};
  double max_bank_angle_deg{0.0};
  double fuel_transfer_speed{0.0}; // kg/s，新增：加油机输油速率
};

struct CurrentStatus {
  double fuel_kg{0.0};
  double heading_deg{0.0};
  double speed_mps{0.0};
  double max_fuel_kg{0.0}; // 新增：受油机最大载油量（兼容挂在 current_status）
  int priority{0};         // 新增：受油机优先级（1 最高，0 表示未指定）
  std::string timestamp;
};

struct AircraftConfig {
  std::string id;
  std::string type;
  CruisePerformanceTable cruise_perf;
  FuelConsumptionTable fuel_table;
  LLA initial_position_lla;
  CurrentStatus current_status;
  PerformanceLimits limits;

  // 兼容不同 JSON 落点，便于后续 Stage 直接读取
  double max_fuel_kg{0.0};
  int priority{0};
  double fuel_transfer_speed{0.0};
  double fuel_transfer_speed_kgps{0.0};

  Vec3 initial_position_xy;
};

// ========================
// 4) 任务配置（输入 mission*.json / assignment.json）
// ========================

enum class BranchKind {
  kUnknown = 0,
  kFollow,
  kIntercept,
  kOneToManyFixed,
  kManyToMany,
};

struct SafeZoneInput {
  bool is_defined{false};
  std::vector<int> zone_ids;
  std::vector<std::vector<LLA>> zones_vertices_lla;
};

struct NoFlyZoneInput {
  bool is_defined{false};
  std::vector<int> zone_ids;
  std::vector<std::vector<LLA>> zones_vertices_lla;
};

struct OptimizationPrefs {
  int primary_pref{0};
  int secondary_pref{0};
  std::vector<double> weight_factors;
};

struct RacecourseInput {
  std::string turn_direction;
  double orbit_radius{0.0};
  double entry_angle_deg{0.0};
  double altitude_m{0.0};
  double speed_mps{0.0};
};

struct RefuelModeInput {
  int mode_type{0};
  std::string sub_mode;
  std::string designated_time;
  SafeZoneInput safe_zone;
};

struct OperationAreaInput {
  std::vector<LLA> boundary_lla;
  std::vector<double> altitude_limits;
  NoFlyZoneInput no_fly_zones;
};

struct RefuelAssignment {
  std::string tanker_id;
  std::vector<std::string> receiver_ids;
};

struct MissionInput {
  std::string mission_id;
  std::string creation_time;
  std::string tanker_id;
  std::vector<std::string> tanker_ids;
  std::vector<std::string> receiver_ids;
  RefuelModeInput refuel_mode;
  OptimizationPrefs prefs;
  RacecourseInput racecourse;
  OperationAreaInput operation_area;
  std::vector<RefuelAssignment> assignments;
  BranchKind branch_kind{BranchKind::kUnknown};
};

// ========================
// 5) 算法中间量/输出量
// ========================

struct SpeedBounds {
  double min_speed_mps{0.0};
  double cruise_speed_mps{0.0};
  double max_speed_mps{0.0};
};

struct SafeZoneSelected {
  Polygon safe_zone_polygon_xy;
  int chosen_safezone_id{-1};
};

struct RacetrackPlan {
  double length_m{0.0};
  double radius_m{0.0};
  double orientation_deg{0.0};
  double altitude_m{0.0};
  double speed_mps{0.0};

  Vec3 center_xy;
  std::vector<Vec3> entrypoints_xy;
};

struct TankerRacetrackDecision {
  bool tanker_on_racetrack{false};
  bool need_transfer{false};
  bool reduce_rendezvous{false};
};

struct SequencePlan {
  std::vector<int> sequence;
  std::map<std::string, int> receiver_to_order;
};

struct RendezvousPlan {
  double tanker_meeting_altitude_m{0.0};
  double tanker_meeting_speed_mps{0.0};
  Vec3 tanker_entry_point_xy;
  double tanker_entry_direction_deg{0.0};

  struct ReceiverRendezvous {
    std::string receiver_id;
    int sequence_order{0};
    double meeting_altitude_m{0.0};
    double meeting_speed_mps{0.0};
    Vec3 coordinate_xy;
    Vec3 waiting_point_xy;
    int cycling_number{0};
  };
  std::map<std::string, ReceiverRendezvous> receivers;
};

struct WaypointXYZ {
  double x{0.0}, y{0.0}, z{0.0};
};

struct Trajectory {
  std::string aircraft_id;
  std::vector<WaypointXYZ> points;
};

struct CostReport {
  double tanker_arrival_time_s{0.0};
  double tanker_fuel_consumed{0.0};

  std::map<std::string, double> receiver_arrival_time_s;
  std::map<std::string, double> receiver_fuel_consumed;

  double receiver_total_arrival_time_s{0.0};
  double receiver_total_fuel_consumed{0.0};
};

// ========================
// 6) 一次完整运行的上下文
// ========================

struct PlanningContext {
  MissionInput mission;

  // 当前子问题使用的 active tanker
  AircraftConfig tanker;
  // 全量 tankers，供多对多分解时挑选
  std::vector<AircraftConfig> tankers;
  std::vector<AircraftConfig> receivers;

  // 多对多串联时，用来排除已使用安全区
  std::vector<int> excluded_safezone_ids;

  SpeedBounds tanker_speed_bounds;
  std::map<std::string, SpeedBounds> receiver_speed_bounds;

  SafeZoneSelected safe_zone_selected;
  RacetrackPlan racetrack;
  TankerRacetrackDecision tanker_rt_decision;
  SequencePlan sequence;
  RendezvousPlan rendezvous;

  std::vector<Trajectory> trajectories;
  CostReport cost;
};

} // namespace refuel
