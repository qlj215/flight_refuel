#include "branches/chuying_branch.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "stages/coordinate_transform_stage.hpp"
#include "stages/preprocess_stage.hpp"
#include "stages/route_planner.h"

namespace fs = std::filesystem;

namespace refuel::branch {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSwitchDistanceM = 200000.0;
constexpr double kTimeStepS = 5.0;
constexpr double kEps = 1e-9;
constexpr double kEarthRadiusM = 6378137.0;

struct Vec2 {
  double x{0.0};
  double y{0.0};
};

struct Weights {
  double time{1.0 / 3.0};
  double tanker_fuel{1.0 / 3.0};
  double receiver_fuel{1.0 / 3.0};
};

struct FuelTable {
  const std::vector<double>* altitudes{nullptr};
  const std::vector<double>* speeds{nullptr};
  const std::vector<std::vector<double>>* rates{nullptr};
};

struct AircraftIn {
  Vec2 pos;
  double height_m{0.0};
  double heading_math_deg{0.0};
  double speed_cur{0.0};
  double speed_min{0.0};
  double speed_max{0.0};
  double fuel_now_kg{0.0};
  FuelTable fuel{};
};

struct Candidate {
  bool success{false};
  std::string mode;
  std::string detail;

  Vec2 meet{};
  double meet_time_s{0.0};
  double phase1_time_s{0.0};
  double phase2_time_s{0.0};

  double tanker_speed_mps{0.0};
  double receiver_speed_mps{0.0};
  double meet_heading_math_deg{0.0};

  Vec2 phase1_tanker_end{};
  Vec2 phase1_receiver_end{};

  double tanker_fuel_used_kg{0.0};
  double receiver_fuel_used_kg{0.0};

  double objective{std::numeric_limits<double>::infinity()};
};

struct MetricRange {
  double min_v{std::numeric_limits<double>::infinity()};
  double max_v{-std::numeric_limits<double>::infinity()};

  void Update(double v) {
    min_v = std::min(min_v, v);
    max_v = std::max(max_v, v);
  }

  double Normalize(double v) const {
    if (!(max_v > min_v + 1e-12)) return 0.0;
    return (v - min_v) / (max_v - min_v);
  }
};

struct Metrics {
  MetricRange time;
  MetricRange tanker_fuel;
  MetricRange receiver_fuel;

  void Update(const Candidate& c) {
    time.Update(c.meet_time_s);
    tanker_fuel.Update(c.tanker_fuel_used_kg);
    receiver_fuel.Update(c.receiver_fuel_used_kg);
  }
};

inline double Deg2Rad(double d) { return d * kPi / 180.0; }
inline double Rad2Deg(double r) { return r * 180.0 / kPi; }

double NormalizeDeg360(double d) {
  while (d < 0.0) d += 360.0;
  while (d >= 360.0) d -= 360.0;
  return d;
}

// 0324chuyin 模型使用“数学航向”约定：0°沿 +X，逆时针为正。
// 当前接入按“原值直通”处理（不做 nav<->math 自动转换），以保证与外部模型输入/输出一致。
inline double NavToMathDeg(double heading_deg_raw) { return NormalizeDeg360(heading_deg_raw); }
inline double MathToNavDeg(double heading_deg_math) { return NormalizeDeg360(heading_deg_math); }

Vec2 HeadingToUnitMath(double heading_math_deg) {
  const double r = Deg2Rad(heading_math_deg);
  return {std::cos(r), std::sin(r)};
}

double Norm2(const Vec2& v) { return std::sqrt(v.x * v.x + v.y * v.y); }

double Dist(const Vec2& a, const Vec2& b) {
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

Vec2 Add(const Vec2& a, const Vec2& b) { return {a.x + b.x, a.y + b.y}; }
Vec2 Sub(const Vec2& a, const Vec2& b) { return {a.x - b.x, a.y - b.y}; }
Vec2 Mul(const Vec2& a, double s) { return {a.x * s, a.y * s}; }

Vec2 Normalize(const Vec2& v) {
  const double n = Norm2(v);
  if (n < kEps) return {0.0, 0.0};
  return {v.x / n, v.y / n};
}

bool AlmostEqual(double a, double b, double eps = 1e-8) { return std::fabs(a - b) <= eps; }

std::pair<int, int> BracketIndex(const std::vector<double>& levels, double x) {
  if (levels.empty()) throw std::runtime_error("Fuel table levels are empty.");
  if (x <= levels.front()) return {0, 0};
  if (x >= levels.back()) {
    const int last = static_cast<int>(levels.size()) - 1;
    return {last, last};
  }
  for (int i = 0; i + 1 < static_cast<int>(levels.size()); ++i) {
    if (x >= levels[i] - 1e-9 && x <= levels[i + 1] + 1e-9) return {i, i + 1};
  }
  const int last = static_cast<int>(levels.size()) - 1;
  return {last, last};
}

double BilinearInterpolate(const FuelTable& tb, double altitude, double speed) {
  if (!tb.altitudes || !tb.speeds || !tb.rates) {
    throw std::runtime_error("Fuel table pointers are not initialized.");
  }
  const auto& alts = *tb.altitudes;
  const auto& spds = *tb.speeds;
  const auto& rates = *tb.rates;
  if (alts.empty() || spds.empty() || rates.empty()) {
    throw std::runtime_error("Fuel table data is empty.");
  }

  auto [a0, a1] = BracketIndex(alts, altitude);
  auto [s0, s1] = BracketIndex(spds, speed);

  auto at = [&](int ai, int si) -> double {
    if (ai < 0 || ai >= static_cast<int>(rates.size())) return rates.back().back();
    if (rates[ai].empty()) return 0.0;
    const int idx = std::clamp(si, 0, static_cast<int>(rates[ai].size()) - 1);
    return rates[ai][idx];
  };

  if (a0 == a1 && s0 == s1) return at(a0, s0);

  if (a0 == a1) {
    const double x0 = spds[s0], x1 = spds[s1];
    const double q0 = at(a0, s0), q1 = at(a0, s1);
    if (std::fabs(x1 - x0) < kEps) return q0;
    return q0 + (q1 - q0) * (speed - x0) / (x1 - x0);
  }

  if (s0 == s1) {
    const double y0 = alts[a0], y1 = alts[a1];
    const double q0 = at(a0, s0), q1 = at(a1, s0);
    if (std::fabs(y1 - y0) < kEps) return q0;
    return q0 + (q1 - q0) * (altitude - y0) / (y1 - y0);
  }

  const double x0 = spds[s0], x1 = spds[s1];
  const double y0 = alts[a0], y1 = alts[a1];
  const double q11 = at(a0, s0);
  const double q21 = at(a0, s1);
  const double q12 = at(a1, s0);
  const double q22 = at(a1, s1);

  const double tx = (std::fabs(x1 - x0) < kEps) ? 0.0 : (speed - x0) / (x1 - x0);
  const double ty = (std::fabs(y1 - y0) < kEps) ? 0.0 : (altitude - y0) / (y1 - y0);

  const double r1 = q11 * (1.0 - tx) + q21 * tx;
  const double r2 = q12 * (1.0 - tx) + q22 * tx;
  return r1 * (1.0 - ty) + r2 * ty;
}

Weights ExtractWeights(const MissionInput& mission) {
  Weights w;
  if (mission.prefs.weight_factors.size() >= 3) {
    w.time = mission.prefs.weight_factors[0];
    w.tanker_fuel = mission.prefs.weight_factors[1];
    w.receiver_fuel = mission.prefs.weight_factors[2];
    const double sum = w.time + w.tanker_fuel + w.receiver_fuel;
    if (sum > 1e-12) {
      w.time /= sum;
      w.tanker_fuel /= sum;
      w.receiver_fuel /= sum;
    } else {
      w = {};
    }
  }
  return w;
}

std::string ModeNameForState(int state, bool has_phase1) {
  if (state == 1) return "SEVERE_FIXED_RECEIVER";
  if (state == 2) {
    return has_phase1 ? "HEAD_ON_THEN_TANKER_CUT_INTO_RECEIVER_ROUTE"
                      : "DIRECT_TANKER_CUT_INTO_RECEIVER_ROUTE";
  }
  return has_phase1 ? "HEAD_ON_THEN_RECEIVER_CUT_INTO_TANKER_ROUTE"
                    : "DIRECT_RECEIVER_CUT_INTO_TANKER_ROUTE";
}

void FinalizeObjectives(std::vector<Candidate>& all, const Weights& w) {
  if (all.empty()) return;
  Metrics m;
  for (const auto& c : all) m.Update(c);
  for (auto& c : all) {
    const double nt = m.time.Normalize(c.meet_time_s);
    const double n1 = m.tanker_fuel.Normalize(c.tanker_fuel_used_kg);
    const double n2 = m.receiver_fuel.Normalize(c.receiver_fuel_used_kg);
    c.objective = w.time * nt + w.tanker_fuel * n1 + w.receiver_fuel * n2;
  }
}

Candidate ChooseBest(std::vector<Candidate>& all) {
  if (all.empty()) throw std::runtime_error("NO_FEASIBLE_RENDEZVOUS");
  std::sort(all.begin(), all.end(), [](const Candidate& a, const Candidate& b) {
    if (!AlmostEqual(a.objective, b.objective, 1e-12)) return a.objective < b.objective;
    if (!AlmostEqual(a.meet_time_s, b.meet_time_s, 1e-9)) return a.meet_time_s < b.meet_time_s;
    return (a.tanker_fuel_used_kg + a.receiver_fuel_used_kg) <
           (b.tanker_fuel_used_kg + b.receiver_fuel_used_kg);
  });
  return all.front();
}

std::vector<Candidate> SolveSevere(const AircraftIn& tanker,
                                   const AircraftIn& receiver,
                                   const Weights& weights) {
  std::vector<Candidate> out;
  const Vec2 receiver_dir = HeadingToUnitMath(receiver.heading_math_deg);
  const double receiver_speed = receiver.speed_cur;

  const double receiver_rate_kgph =
      BilinearInterpolate(receiver.fuel, receiver.height_m, receiver_speed);
  if (receiver_rate_kgph <= 1e-9) return out;

  const double max_time_s = receiver.fuel_now_kg / receiver_rate_kgph * 3600.0;
  for (double t = kTimeStepS; t <= max_time_s + 1e-9; t += kTimeStepS) {
    const Vec2 meet = Add(receiver.pos, Mul(receiver_dir, receiver_speed * t));

    const double tanker_dist = Dist(tanker.pos, meet);
    const double tanker_speed = tanker_dist / t;
    if (tanker_speed < tanker.speed_min - 1e-6 || tanker_speed > tanker.speed_max + 1e-6) continue;

    const double tanker_rate_kgph =
        BilinearInterpolate(tanker.fuel, receiver.height_m, tanker_speed);

    const double tanker_fuel = tanker_rate_kgph * t / 3600.0;
    const double receiver_fuel = receiver_rate_kgph * t / 3600.0;
    if (receiver_fuel > receiver.fuel_now_kg + 1e-6) continue;

    Candidate c;
    c.success = true;
    c.mode = ModeNameForState(1, false);
    c.detail = "Receiver keeps original heading/speed; tanker cuts into receiver route.";
    c.meet = meet;
    c.meet_time_s = t;
    c.phase1_time_s = 0.0;
    c.phase2_time_s = t;
    c.tanker_speed_mps = tanker_speed;
    c.receiver_speed_mps = receiver_speed;
    c.meet_heading_math_deg = receiver.heading_math_deg;
    c.phase1_tanker_end = tanker.pos;
    c.phase1_receiver_end = receiver.pos;
    c.tanker_fuel_used_kg = tanker_fuel;
    c.receiver_fuel_used_kg = receiver_fuel;
    out.push_back(c);
  }

  FinalizeObjectives(out, weights);
  return out;
}

std::vector<Candidate> SolveMovable(const AircraftIn& tanker,
                                    const AircraftIn& receiver,
                                    int damage_state,
                                    const Weights& weights) {
  std::vector<Candidate> out;

  const double initial_dist = Dist(tanker.pos, receiver.pos);
  const bool needs_phase1 = initial_dist > kSwitchDistanceM + 1e-9;

  const Vec2 line_tr = Normalize(Sub(receiver.pos, tanker.pos));
  const Vec2 line_rt = Mul(line_tr, -1.0);

  const Vec2 tanker_orig_dir = HeadingToUnitMath(tanker.heading_math_deg);
  const Vec2 receiver_orig_dir = HeadingToUnitMath(receiver.heading_math_deg);

  const bool receiver_has_priority = (damage_state == 2);

  const int tanker_vmin = static_cast<int>(std::ceil(tanker.speed_min));
  const int tanker_vmax = static_cast<int>(std::floor(tanker.speed_max));
  const int receiver_vmin = static_cast<int>(std::ceil(receiver.speed_min));
  const int receiver_vmax = static_cast<int>(std::floor(receiver.speed_max));

  for (int vt = tanker_vmin; vt <= tanker_vmax; ++vt) {
    const double tanker_speed = static_cast<double>(vt);
    const double tanker_rate_kgph =
        BilinearInterpolate(tanker.fuel, receiver.height_m, tanker_speed);

    for (int vr = receiver_vmin; vr <= receiver_vmax; ++vr) {
      const double receiver_speed = static_cast<double>(vr);
      const double receiver_rate_kgph =
          BilinearInterpolate(receiver.fuel, receiver.height_m, receiver_speed);
      if (receiver_rate_kgph <= 1e-9) continue;

      const double receiver_endurance_s = receiver.fuel_now_kg / receiver_rate_kgph * 3600.0;
      if (receiver_endurance_s <= 0.0) continue;

      double phase1_time = 0.0;
      Vec2 tanker_switch = tanker.pos;
      Vec2 receiver_switch = receiver.pos;

      if (needs_phase1) {
        const double closing_speed = tanker_speed + receiver_speed;
        if (closing_speed <= 1e-9) continue;
        phase1_time = (initial_dist - kSwitchDistanceM) / closing_speed;
        if (phase1_time < 0.0) phase1_time = 0.0;

        tanker_switch = Add(tanker.pos, Mul(line_tr, tanker_speed * phase1_time));
        receiver_switch = Add(receiver.pos, Mul(line_rt, receiver_speed * phase1_time));
      }

      if (phase1_time > receiver_endurance_s + 1e-6) continue;

      for (double tau = kTimeStepS; tau <= receiver_endurance_s - phase1_time + 1e-9; tau += kTimeStepS) {
        Vec2 meet;
        double required_cutter_speed = 0.0;
        double common_heading_math = 0.0;

        if (receiver_has_priority) {
          meet = Add(receiver_switch, Mul(receiver_orig_dir, receiver_speed * tau));
          required_cutter_speed = Dist(tanker_switch, meet) / tau;
          common_heading_math = receiver.heading_math_deg;
          if (std::fabs(required_cutter_speed - tanker_speed) > 0.75) continue;
        } else {
          meet = Add(tanker_switch, Mul(tanker_orig_dir, tanker_speed * tau));
          required_cutter_speed = Dist(receiver_switch, meet) / tau;
          common_heading_math = tanker.heading_math_deg;
          if (std::fabs(required_cutter_speed - receiver_speed) > 0.75) continue;
        }

        const double total_t = phase1_time + tau;
        const double tanker_fuel = tanker_rate_kgph * total_t / 3600.0;
        const double receiver_fuel = receiver_rate_kgph * total_t / 3600.0;
        if (receiver_fuel > receiver.fuel_now_kg + 1e-6) continue;

        Candidate c;
        c.success = true;
        c.mode = ModeNameForState(damage_state, needs_phase1);
        c.detail = receiver_has_priority
                       ? "Head-on if needed, then receiver keeps route and tanker cuts in."
                       : "Head-on if needed, then tanker keeps route and receiver cuts in.";
        c.meet = meet;
        c.meet_time_s = total_t;
        c.phase1_time_s = phase1_time;
        c.phase2_time_s = tau;
        c.tanker_speed_mps = tanker_speed;
        c.receiver_speed_mps = receiver_speed;
        c.meet_heading_math_deg = common_heading_math;
        c.phase1_tanker_end = tanker_switch;
        c.phase1_receiver_end = receiver_switch;
        c.tanker_fuel_used_kg = tanker_fuel;
        c.receiver_fuel_used_kg = receiver_fuel;
        out.push_back(c);
      }
    }
  }

  FinalizeObjectives(out, weights);
  return out;
}

std::vector<std::vector<routeplan::Vec2>> BuildNoFlyObstaclePolygons(const PlanningContext& ctx) {
  std::vector<std::vector<routeplan::Vec2>> out;
  const auto& nf = ctx.mission.operation_area.no_fly_zones;
  if (!nf.is_defined) return out;

  out.reserve(nf.zones_vertices_lla.size());
  for (const auto& poly_lla : nf.zones_vertices_lla) {
    if (poly_lla.size() < 3) continue;
    std::vector<routeplan::Vec2> poly_xy;
    poly_xy.reserve(poly_lla.size());
    for (const auto& p : poly_lla) {
      const double lat = p.lat_deg * kPi / 180.0;
      const double lon = p.lon_deg * kPi / 180.0;
      poly_xy.push_back({kEarthRadiusM * lon,
                         kEarthRadiusM * std::log(std::tan(kPi / 4.0 + lat / 2.0))});
    }
    if (poly_xy.size() >= 3) out.push_back(std::move(poly_xy));
  }
  return out;
}

std::vector<WaypointXYZ> SampleStraightLine(const WaypointXYZ& start,
                                            const WaypointXYZ& goal,
                                            double step_m) {
  std::vector<WaypointXYZ> out;
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double dz = goal.z - start.z;
  const double dist = std::sqrt(dx * dx + dy * dy);
  if (dist <= 1e-9) {
    out.push_back(start);
    if (!AlmostEqual(start.z, goal.z, 1e-6)) out.push_back(goal);
    return out;
  }
  const int n = std::max(1, static_cast<int>(std::ceil(dist / std::max(1.0, step_m))));
  out.reserve(static_cast<size_t>(n) + 1);
  for (int i = 0; i <= n; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(n);
    out.push_back({start.x + u * dx, start.y + u * dy, start.z + u * dz});
  }
  return out;
}

void AppendPoint(std::vector<WaypointXYZ>& pts, const WaypointXYZ& p) {
  if (!pts.empty()) {
    const auto& b = pts.back();
    if (std::fabs(b.x - p.x) < 1e-6 && std::fabs(b.y - p.y) < 1e-6 && std::fabs(b.z - p.z) < 1e-6) {
      return;
    }
  }
  pts.push_back(p);
}

void AppendPoints(std::vector<WaypointXYZ>& dst, const std::vector<WaypointXYZ>& src) {
  for (const auto& p : src) AppendPoint(dst, p);
}

std::vector<WaypointXYZ> PlanSegment(const AircraftConfig& ac,
                                     const Vec2& start_xy,
                                     double start_z,
                                     double start_heading_math_deg,
                                     const Vec2& goal_xy,
                                     double goal_z,
                                     double goal_heading_math_deg,
                                     double speed_mps,
                                     double turn_radius,
                                     const std::vector<std::vector<routeplan::Vec2>>& obstacles) {
  routeplan::Pose3D start{start_xy.x, start_xy.y, start_z, Deg2Rad(start_heading_math_deg)};
  routeplan::Pose3D goal{goal_xy.x, goal_xy.y, goal_z, Deg2Rad(goal_heading_math_deg)};

  routeplan::PlannerConfig cfg;
  cfg.turnRadius = std::max(10.0, turn_radius);
  cfg.step = std::clamp(turn_radius * 0.2, 200.0, 2000.0);
  cfg.finalStraightLen = std::max(0.0, cfg.step * 2.0);
  cfg.climbRate = std::max(0.1, ac.limits.climb_rate);
  cfg.descendRate = std::max(0.1, ac.limits.descent_rate);
  cfg.groundSpeed = std::max(1.0, speed_mps > 0.0 ? speed_mps : ac.current_status.speed_mps);
  cfg.z_mid = goal_z;
  cfg.allowSplineFallback = true;

  const auto res = routeplan::PlanRoute(start, goal, obstacles, cfg);
  if (!res.route.empty()) {
    std::vector<WaypointXYZ> out;
    out.reserve(res.route.size());
    for (const auto& p : res.route) out.push_back({p.x, p.y, p.z});
    return out;
  }

  return SampleStraightLine({start.x, start.y, start.z}, {goal.x, goal.y, goal.z}, cfg.step);
}

std::vector<WaypointXYZ> PlanRouteWithPhaseMidpoint(const AircraftConfig& ac,
                                                    const Vec2& phase_mid_xy,
                                                    bool has_phase,
                                                    const Vec2& meet_xy,
                                                    double meet_alt_m,
                                                    double meet_heading_math_deg,
                                                    double speed_mps,
                                                    double turn_radius,
                                                    const std::vector<std::vector<routeplan::Vec2>>& obstacles) {
  const Vec2 start_xy{ac.initial_position_xy.x, ac.initial_position_xy.y};
  const double start_z = ac.initial_position_lla.alt_m;
  const double start_heading_math = NavToMathDeg(ac.current_status.heading_deg);

  std::vector<WaypointXYZ> out;

  if (!has_phase) {
    return PlanSegment(ac,
                       start_xy,
                       start_z,
                       start_heading_math,
                       meet_xy,
                       meet_alt_m,
                       meet_heading_math_deg,
                       speed_mps,
                       turn_radius,
                       obstacles);
  }

  // Segment 1: start -> phase(mid)
  const double hdg_mid = Rad2Deg(std::atan2(meet_xy.y - phase_mid_xy.y, meet_xy.x - phase_mid_xy.x));
  auto seg1 = PlanSegment(ac,
                          start_xy,
                          start_z,
                          start_heading_math,
                          phase_mid_xy,
                          meet_alt_m,
                          hdg_mid,
                          speed_mps,
                          turn_radius,
                          obstacles);

  // Segment 2: phase(mid) -> meet
  auto seg2 = PlanSegment(ac,
                          phase_mid_xy,
                          meet_alt_m,
                          hdg_mid,
                          meet_xy,
                          meet_alt_m,
                          meet_heading_math_deg,
                          speed_mps,
                          turn_radius,
                          obstacles);

  AppendPoints(out, seg1);
  AppendPoints(out, seg2);
  return out;
}

void WriteCsv(const fs::path& file, const std::vector<WaypointXYZ>& pts) {
  std::ofstream ofs(file);
  if (!ofs) throw std::runtime_error("Failed to write CSV: " + file.string());
  ofs << "x,y,z\n";
  ofs << std::fixed << std::setprecision(2);
  for (const auto& p : pts) {
    ofs << p.x << "," << p.y << "," << p.z << "\n";
  }
}

double ClampPositive(double v, double fallback) {
  return (v > 1e-9) ? v : fallback;
}

} // namespace

bool RunChuyingBranch(const PlanningContext& base,
                      const std::string& outputDir,
                      std::string* errorMsg) {
  try {
    PlanningContext ctx = base;

    // 坐标与速度边界适配（沿用现有 Stage 结果，避免改动大量字段定义）
    CoordinateTransformStage coord_stage;
    coord_stage.Run(ctx);
    PreprocessStage preprocess_stage;
    preprocess_stage.Run(ctx);

    if (ctx.tankers.empty()) {
      throw std::runtime_error("No tanker found for chuying branch.");
    }
    if (ctx.receivers.empty()) {
      throw std::runtime_error("No receiver found for chuying branch.");
    }

    // 当前项目出迎分支先按“单加油机，多受油机逐个求解”接入
    const AircraftConfig& tanker_cfg = ctx.tanker;
    const Weights weights = ExtractWeights(ctx.mission);

    // 构造 tanker 输入
    const auto tanker_sb = ctx.tanker_speed_bounds;
    AircraftIn tanker;
    tanker.pos = {tanker_cfg.initial_position_xy.x, tanker_cfg.initial_position_xy.y};
    tanker.height_m = tanker_cfg.initial_position_lla.alt_m;
    tanker.heading_math_deg = NavToMathDeg(tanker_cfg.current_status.heading_deg);
    tanker.speed_cur = ClampPositive(tanker_cfg.current_status.speed_mps, tanker_sb.cruise_speed_mps);
    const double tanker_min_override = tanker_cfg.current_status.speed_min_mps;
    const double tanker_max_override = tanker_cfg.current_status.speed_max_mps;
    tanker.speed_min = (tanker_min_override > 1e-9)
                           ? tanker_min_override
                           : ClampPositive(tanker_sb.min_speed_mps, 120.0);
    tanker.speed_max = (tanker_max_override > 1e-9)
                           ? tanker_max_override
                           : std::max(tanker.speed_min + 1.0, ClampPositive(tanker_sb.max_speed_mps, 300.0));
    if (tanker.speed_max < tanker.speed_min + 1e-9) tanker.speed_max = tanker.speed_min + 1.0;
    tanker.fuel_now_kg = ClampPositive(tanker_cfg.current_status.fuel_kg, 1.0);
    tanker.fuel = {&tanker_cfg.fuel_table.altitude_levels,
                   &tanker_cfg.fuel_table.speed_levels,
                   &tanker_cfg.fuel_table.consumption_data};

    const auto obstacles = BuildNoFlyObstaclePolygons(ctx);
    const double turn_radius = (ctx.mission.racecourse.orbit_radius > 1e-6)
                                   ? ctx.mission.racecourse.orbit_radius
                                   : 5000.0;

    nlohmann::json root;
    root["branch"] = "chuying";
    root["status"] = "ok";
    root["model"] = "0324chuyin-integrated";
    root["phase_semantics"] = "phase in output means airway midpoint";
    root["weights"] = {
        {"time", weights.time},
        {"tanker_fuel", weights.tanker_fuel},
        {"receiver_fuel", weights.receiver_fuel},
    };
    root["results"] = nlohmann::json::array();
    int success_count = 0;

    fs::create_directories(outputDir);

    for (size_t i = 0; i < ctx.receivers.size(); ++i) {
      const auto& r = ctx.receivers[i];

      auto rb_it = ctx.receiver_speed_bounds.find(r.id);
      const SpeedBounds rb = (rb_it == ctx.receiver_speed_bounds.end()) ? SpeedBounds{120.0, 170.0, 300.0}
                                                                         : rb_it->second;

      AircraftIn receiver;
      receiver.pos = {r.initial_position_xy.x, r.initial_position_xy.y};
      receiver.height_m = r.initial_position_lla.alt_m;
      receiver.heading_math_deg = NavToMathDeg(r.current_status.heading_deg);
      receiver.speed_cur = ClampPositive(r.current_status.speed_mps, rb.cruise_speed_mps);
      const double receiver_min_override = r.current_status.speed_min_mps;
      const double receiver_max_override = r.current_status.speed_max_mps;
      receiver.speed_min = (receiver_min_override > 1e-9)
                               ? receiver_min_override
                               : ClampPositive(rb.min_speed_mps, 120.0);
      receiver.speed_max = (receiver_max_override > 1e-9)
                               ? receiver_max_override
                               : std::max(receiver.speed_min + 1.0, ClampPositive(rb.max_speed_mps, 300.0));
      if (receiver.speed_max < receiver.speed_min + 1e-9) receiver.speed_max = receiver.speed_min + 1.0;
      receiver.fuel_now_kg = ClampPositive(r.current_status.fuel_kg, 1.0);
      receiver.fuel = {&r.fuel_table.altitude_levels,
                       &r.fuel_table.speed_levels,
                       &r.fuel_table.consumption_data};

      // 约定：priority 映射到 damage_state，超界时截断到 [0,2]
      const int damage_state = std::clamp(r.priority, 0, 2);

      std::vector<Candidate> all;
      if (damage_state == 1) {
        all = SolveSevere(tanker, receiver, weights);
      } else {
        all = SolveMovable(tanker, receiver, damage_state, weights);
      }

      if (all.empty()) {
        nlohmann::json fail;
        fail["receiver_id"] = r.id;
        fail["success"] = 0;
        fail["error"] = "NO_FEASIBLE_RENDEZVOUS";
        root["results"].push_back(fail);
        continue;
      }

      Candidate best = ChooseBest(all);

      const Vec2 meet_xy = best.meet;
      const double meet_alt = receiver.height_m;

      const bool has_tanker_phase = best.phase1_time_s > 1e-6 &&
                                    Dist(best.phase1_tanker_end, tanker.pos) > 1.0;
      const bool has_receiver_phase = best.phase1_time_s > 1e-6 &&
                                      Dist(best.phase1_receiver_end, receiver.pos) > 1.0;

      auto tanker_traj = PlanRouteWithPhaseMidpoint(
          tanker_cfg,
          best.phase1_tanker_end,
          has_tanker_phase,
          meet_xy,
          meet_alt,
          best.meet_heading_math_deg,
          best.tanker_speed_mps,
          turn_radius,
          obstacles);

      auto receiver_traj = PlanRouteWithPhaseMidpoint(
          r,
          best.phase1_receiver_end,
          has_receiver_phase,
          meet_xy,
          meet_alt,
          best.meet_heading_math_deg,
          best.receiver_speed_mps,
          turn_radius,
          obstacles);

      // 写 CSV
      std::string tanker_csv_name;
      std::string receiver_csv_name;
      if (ctx.receivers.size() == 1) {
        tanker_csv_name = "tanker01.csv";
        receiver_csv_name = "receiver01.csv";
      } else {
        tanker_csv_name = "tanker_" + r.id + ".csv";
        receiver_csv_name = "receiver_" + r.id + ".csv";
      }

      WriteCsv(fs::path(outputDir) / tanker_csv_name, tanker_traj);
      WriteCsv(fs::path(outputDir) / receiver_csv_name, receiver_traj);

      const bool tanker_cuts_in =
          (best.mode.find("TANKER_CUT_INTO_RECEIVER_ROUTE") != std::string::npos) ||
          (best.mode == "SEVERE_FIXED_RECEIVER");

      nlohmann::json item;
      item["receiver_id"] = r.id;
      item["success"] = 1;

      // 兼容 0324chuyin/output.json 的核心结构
      item["meeting_point"] = {best.meet.x, best.meet.y};
      item["meeting_method"] = tanker_cuts_in ? "B" : "A";
      item["tanker_meeting_params"] = {
          {"speed", best.tanker_speed_mps},
          {"altitude", meet_alt},
      };
      item["meeting_time"] = best.meet_time_s;
      item["fuel_consumption"] = {
          {"tanker", best.tanker_fuel_used_kg},
          {"receiver", best.receiver_fuel_used_kg},
      };
      item["fueling_params"] = {
          {"speed", best.receiver_speed_mps},
          {"altitude", meet_alt},
      };

      // 扩展字段：phase 即航路中间点
      item["phase"] = {
          {"phase1_time_s", best.phase1_time_s},
          {"phase2_time_s", best.phase2_time_s},
          {"tanker_midpoint", {best.phase1_tanker_end.x, best.phase1_tanker_end.y}},
          {"receiver_midpoint", {best.phase1_receiver_end.x, best.phase1_receiver_end.y}},
      };

      // 调试/追踪字段
      item["mode"] = best.mode;
      item["detail"] = best.detail;
      item["objective"] = best.objective;
      item["meet_heading_deg_math"] = NormalizeDeg360(best.meet_heading_math_deg);
      item["meet_heading_deg_nav"] = MathToNavDeg(best.meet_heading_math_deg);
      item["tanker_csv"] = tanker_csv_name;
      item["receiver_csv"] = receiver_csv_name;
      item["receiver_damage_state"] = damage_state;

      root["results"].push_back(item);
      ++success_count;
    }

    // 单受油机时，主输出与 0324chuyin/output.json 同层级；
    // 多受油机时保留 results 数组承载多条结果。
    nlohmann::json final_out;
    if (ctx.receivers.size() == 1 && root["results"].is_array() && root["results"].size() == 1) {
      const auto& one = root["results"][0];
      if (one.value("success", 0) == 1) {
        final_out = {
            {"meeting_point", one["meeting_point"]},
            {"meeting_method", one["meeting_method"]},
            {"tanker_meeting_params", one["tanker_meeting_params"]},
            {"meeting_time", one["meeting_time"]},
            {"fuel_consumption", one["fuel_consumption"]},
            {"fueling_params", one["fueling_params"]},
            {"phase", one["phase"]},
        };
      } else {
        final_out = {
            {"success", 0},
            {"error", one.value("error", "NO_FEASIBLE_RENDEZVOUS")},
        };
      }
    } else {
      root["success_count"] = success_count;
      final_out = root;
    }

    std::ofstream ofs(fs::path(outputDir) / "output.json");
    if (!ofs) throw std::runtime_error("Failed to open output.json for writing.");
    ofs << final_out.dump(2) << "\n";

    return true;
  } catch (const std::exception& e) {
    if (errorMsg) *errorMsg = e.what();
    return false;
  }
}

} // namespace refuel::branch
