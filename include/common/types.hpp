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

// 经纬度/高度：来自 demo/input/*.json
struct LLA {
  double lat_deg{0.0};
  double lon_deg{0.0};
  double alt_m{0.0};
};

// 平面坐标：坐标转换后的结果（单位建议：meter）
// 说明：demo/output/*.csv 里是 x,y,z（看起来是某种投影后的平面坐标）
// 你需要保证最终输出的坐标系与既有工程一致。
struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// 安全区（多边形）：mission4.json 里 safe_zone.zones[i].vertices 是 [lat, lon] 列表
// 坐标转换后应落到 Vec3（z 可忽略）
struct Polygon {
  std::vector<Vec3> vertices_xy; // 仅使用 x,y
};

// ========================
// 2) 表格数据（输入）
// ========================

// cruise_performance_table：输入里给了 altitude_levels/speed_levels/true_airspeed_data
// 为了“只暴露接口”，这里不规定你要怎么插值；保留原始数据即可。
struct CruisePerformanceTable {
  std::vector<double> altitude_levels;
  std::vector<double> speed_levels;
  std::vector<std::vector<double>> true_airspeed_data;
};

// fuel_consumption_table：输入里给了 altitude_levels/consumption_data(二维)
// 具体含义/维度由你的算法决定（常见是 alt × speed 或 alt × index）。
struct FuelConsumptionTable {
  std::vector<double> altitude_levels;
  std::vector<double> speed_levels;
  std::vector<std::vector<double>> consumption_data;
};

// ========================
// 3) 飞机状态/限制（输入）
// ========================

struct PerformanceLimits {
  double climb_rate{0.0};      // m/s
  double descent_rate{0.0};    // m/s
  double max_altitude{0.0};    // m
  double max_bank_angle_deg{0.0};
};

struct CurrentStatus {
  double fuel_kg{0.0};
  double heading_deg{0.0};
  double speed_mps{0.0};
  std::string timestamp; // 原样保留
};

// Tanker/Receiver 公共字段
struct AircraftConfig {
  std::string id;              // TK001 / RE001 ...
  std::string type;            // KC-135 / F-16 ...
  CruisePerformanceTable cruise_perf;
  FuelConsumptionTable fuel_table;
  LLA initial_position_lla;    // 输入的经纬度
  CurrentStatus current_status;
  PerformanceLimits limits;

  // 坐标转换后的初始位置（算法计算变量之一）
  Vec3 initial_position_xy;
};

// ========================
// 4) 任务配置（输入 mission4.json）
// ========================

struct SafeZoneInput {
  bool is_defined{false};
  // mission4.json: safe_zone.zones[i].vertices = [[lat,lon], ...]
  // 这里先用 LLA 保存原始（alt 不提供则为 0）
  std::vector<std::vector<LLA>> zones_vertices_lla;
};

struct OptimizationPrefs {
  int primary_pref{0};          // mission4.json optimization_prefs.primary_pref
  int secondary_pref{0};
  std::vector<double> weight_factors; // [w_time, w_fuel, w_other] 之类
};

struct RacecourseInput {
  std::string turn_direction;  // nishizhen / shunzhen ...
  double orbit_radius{0.0};    // mission4.json racecourse.orbit_radius
  double entry_angle_deg{0.0}; // entry_angle
};

struct RefuelModeInput {
  int mode_type{0};            // 0/1/2：流程图里的“判断模式”
  std::string sub_mode;        // mission4.json timing_params.sub_mode
  std::string designated_time; // "14:30:00"
  SafeZoneInput safe_zone;
};

struct MissionInput {
  std::string mission_id;
  std::string creation_time;
  std::string tanker_id;                 // mission.units.tanker_id
  std::vector<std::string> receiver_ids; // mission.units.receiver_id（空格分隔）
  RefuelModeInput refuel_mode;
  OptimizationPrefs prefs;
  RacecourseInput racecourse;
};

// ========================
// 5) 流程图红字：算法计算出的关键变量（中间量/输出量）
// ========================

// 5.1 速度边界（来自“滤波、油耗预处理”输出）
struct SpeedBounds {
  double min_speed_mps{0.0};
  double cruise_speed_mps{0.0};
  double max_speed_mps{0.0};
};

// 5.2 安全区选择结果（来自“安全区选择”输出）
struct SafeZoneSelected {
  // 流程图红字：safe_zone_vertices（选择了哪个安全区后得到的顶点集合）
  Polygon safe_zone_polygon_xy;
  // 如果你的工程支持多个安全区，这里也可保留 index/id
  int chosen_safezone_id{-1};
};

// 5.3 跑马场（来自“跑马场建立”输出）
struct RacetrackPlan {
  double length_m{0.0};
  double radius_m{0.0};
  double orientation_deg{0.0};
  double altitude_m{0.0};
  double speed_mps{0.0};

  // 流程图红字：racetrack_center / racetrack_entrypoints
  Vec3 center_xy;
  std::vector<Vec3> entrypoints_xy;
};

// 5.3.1 加油机在跑马场上的状态判定（新增：对应你最新流程图中“加油机是否在跑马场上/是否需要转场”）
//
// 说明：
// - tanker_on_racetrack: 加油机初始位置是否已位于“当前规划跑马场”上
// - need_transfer:       若已在跑马场上，是否仍需要“转场/换跑马场/重新进场”等（由你的工程定义）
// - reduce_rendezvous:   若无需转场，可在“下方四个模式”的前提下，减少加油机会合相关计算
//                        （典型做法：tanker_entry_point 直接取 tanker 初始点；
//                         tanker_entry_direction 直接取当前航向；tanker_meeting_altitude/speed
//                         直接取跑马场或速度边界；不再做“从场外进场/选entrypoints”等计算）
struct TankerRacetrackDecision {
  bool tanker_on_racetrack{false};
  bool need_transfer{false};
  bool reduce_rendezvous{false};
};

// 5.4 加油顺序（来自“确定加油顺序”输出）
struct SequencePlan {
  // 流程图例子：[1,3,2] 表示 receiver1 -> 第1个加油，receiver3 -> 第2个加油，receiver2 -> 第3个加油
  // 这里定义：sequence[i] = receiver index（0-based）按加油顺序排列
  std::vector<int> sequence;
  // 方便输出：receiver_id -> 1/2/3...
  std::map<std::string, int> receiver_to_order;
};

// 5.5 会合/等待点规划（来自“0/1/2模式下的会合点规划”输出）
struct RendezvousPlan {
  // Tanker 的关键量（对应 demo/output/output.json.tanker）
  double tanker_meeting_altitude_m{0.0};
  double tanker_meeting_speed_mps{0.0};
  Vec3   tanker_entry_point_xy;      // entry_point [x,y]，z可不填
  double tanker_entry_direction_deg{0.0};

  // 每个 Receiver 的关键量（对应 demo/output/output.json.receivers.RE00X）
  struct ReceiverRendezvous {
    std::string receiver_id;
    int sequence_order{0}; // 1,2,3...
    double meeting_altitude_m{0.0};
    double meeting_speed_mps{0.0};
    Vec3 coordinate_xy;     // coordinate [x,y]
    Vec3 waiting_point_xy;  // waiting_point [x,y]
    int cycling_number{0};  // 盘旋圈数
  };
  std::map<std::string, ReceiverRendezvous> receivers;
};

// 5.6 航迹点（来自“航路规划”输出），写入 demo/output/*.csv
struct WaypointXYZ {
  double x{0.0}, y{0.0}, z{0.0};
};

struct Trajectory {
  std::string aircraft_id;           // tanker_id 或 receiver_id
  std::vector<WaypointXYZ> points;   // csv: x,y,z
};

// 5.7 代价（来自“代价策略”输出）
struct CostReport {
  double tanker_arrival_time_s{0.0};
  double tanker_fuel_consumed{0.0};

  std::map<std::string, double> receiver_arrival_time_s;
  std::map<std::string, double> receiver_fuel_consumed;

  double receiver_total_arrival_time_s{0.0};
  double receiver_total_fuel_consumed{0.0};
};

// ========================
// 6) 一次完整运行的上下文（各环节之间传递的“接口载体”）
// ========================

struct PlanningContext {
  // === 输入（可直接从 demo/input 读） ===
  MissionInput mission;
  AircraftConfig tanker;
  std::vector<AircraftConfig> receivers;

  // === 中间结果（红字变量） ===
  // 坐标转换后：tanker.initial_position_xy / receiver.initial_position_xy 已写回 AircraftConfig
  SpeedBounds tanker_speed_bounds;
  std::map<std::string, SpeedBounds> receiver_speed_bounds;

  SafeZoneSelected safe_zone_selected;
  RacetrackPlan racetrack;

  // 新增：跑马场状态判定结果（用于在会合规划/航路规划前进行轻量分支）
  TankerRacetrackDecision tanker_rt_decision;
  
  SequencePlan sequence;
  RendezvousPlan rendezvous;

  std::vector<Trajectory> trajectories;
  CostReport cost;
};

} // namespace refuel
