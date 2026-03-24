#include "branches/suidui_branch.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "stages/coordinate_transform_stage.hpp"
#include "stages/preprocess_stage.hpp"
#include "stages/racetrack_build_stage.hpp"
#include "stages/route_planner.h"
#include "stages/safe_zone_select_stage.hpp"

namespace fs = std::filesystem;

namespace refuel::branch {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;
constexpr double kReferenceAltitude = 7000.0;
constexpr double kTimeSearchStep = 5.0;
constexpr double kSearchRadius = 50000.0;
constexpr double kSearchStep = 1000.0;
constexpr double kDistWeight = 1.0;
constexpr double kDistDiffWeight = 2.0;
constexpr double kGoalDistWeight = 0.2;
constexpr double kTurnWeight = 500.0;
constexpr double kEconomicSpeed = 170.0;
constexpr double kOverspeedWeight = 0.3;
constexpr double kThreatBuffer = 20000.0;
constexpr double kRefuelThresholdRatio = 0.25;
constexpr double kRefuelFallbackToGoal = 200000.0;
constexpr double kRefuelLineSearchStep = 1000.0;
constexpr double kEarthRadiusM = 6378137.0;

struct V2 {
  double x{0.0};
  double y{0.0};
};

struct AircraftIn {
  V2 pos;
  double heading_deg{0.0};
  double altitude_m{7000.0};
  double v_min{120.0};
  double v_max{200.0};
  double v_cur{160.0};
  double fuel_now{0.0};
  double fuel_full{0.0};
};

struct Poly {
  std::vector<V2> pts;
};

struct FuelTable {
  const std::vector<double>* altitude_levels{nullptr};
  const std::vector<double>* speed_levels{nullptr};
  const std::vector<std::vector<double>>* consumption_data{nullptr};
};

struct Mission {
  AircraftIn tanker;
  AircraftIn receiver;
  V2 goal;
  double w_time{0.5};
  double w_fuel{0.5};
  double w_safe{0.0};
  double formation_speed{160.0};
  std::vector<Poly> threats;
};

struct MeetResult {
  bool feasible{false};
  V2 meet{};
  double heading_to_goal_deg{0.0};
  double tanker_dist{0.0};
  double receiver_dist{0.0};
  double goal_dist{0.0};
  double tanker_turn_deg{0.0};
  double receiver_turn_deg{0.0};
  double objective{std::numeric_limits<double>::infinity()};
};

struct SpeedResult {
  bool feasible{false};
  double meet_time_s{0.0};
  double tanker_speed{0.0};
  double receiver_speed{0.0};
  double tanker_fuel_to_meet{0.0};
  double receiver_fuel_to_meet{0.0};
  double total_fuel_to_meet{0.0};
  double overspeed_penalty{0.0};
  double objective{std::numeric_limits<double>::infinity()};
};

struct HeightResult {
  bool feasible{false};
  double meet_altitude_m{7000.0};
  double tanker_fuel_to_meet{0.0};
  double receiver_fuel_to_meet{0.0};
  double total_fuel_to_meet{0.0};
  double objective{std::numeric_limits<double>::infinity()};
};

struct RefuelResult {
  bool feasible{false};
  V2 refuel{};
  double dist_from_meet{0.0};
  double dist_to_goal{0.0};
  double receiver_fuel_at_refuel{0.0};
  std::string mode;
};

struct SolveResult {
  bool feasible{false};
  MeetResult meet;
  SpeedResult speed;
  HeightResult height;
  RefuelResult refuel;
};

inline double ClampPos(double v, double fallback) { return v > 1e-9 ? v : fallback; }

inline double NormDeg360(double d) {
  while (d < 0.0) d += 360.0;
  while (d >= 360.0) d -= 360.0;
  return d;
}

inline double Deg2Rad(double d) { return d * kPi / 180.0; }

V2 Add(const V2& a, const V2& b) { return {a.x + b.x, a.y + b.y}; }
V2 Sub(const V2& a, const V2& b) { return {a.x - b.x, a.y - b.y}; }
V2 Mul(const V2& a, double s) { return {a.x * s, a.y * s}; }

double Dot(const V2& a, const V2& b) { return a.x * b.x + a.y * b.y; }
double Cross(const V2& a, const V2& b) { return a.x * b.y - a.y * b.x; }

double Norm(const V2& a) { return std::sqrt(Dot(a, a)); }

double Dist(const V2& a, const V2& b) { return Norm(Sub(a, b)); }

V2 Normalize(const V2& a) {
  const double n = Norm(a);
  if (n < kEps) return {0.0, 0.0};
  return {a.x / n, a.y / n};
}

V2 HeadingToUnit(double heading_deg) {
  const double r = Deg2Rad(heading_deg);
  return {std::cos(r), std::sin(r)};
}

double HeadingToTargetDeg(const V2& from, const V2& to) {
  return NormDeg360(std::atan2(to.y - from.y, to.x - from.x) * 180.0 / kPi);
}

double AngleDiffDeg(double a, double b) {
  a = NormDeg360(a);
  b = NormDeg360(b);
  const double d = std::fabs(a - b);
  return std::min(d, 360.0 - d);
}

std::pair<int, int> Bracket(const std::vector<double>& xs, double x) {
  if (x <= xs.front()) return {0, 0};
  if (x >= xs.back()) {
    const int last = static_cast<int>(xs.size()) - 1;
    return {last, last};
  }
  for (int i = 0; i + 1 < static_cast<int>(xs.size()); ++i) {
    if (x >= xs[i] - 1e-9 && x <= xs[i + 1] + 1e-9) return {i, i + 1};
  }
  const int last = static_cast<int>(xs.size()) - 1;
  return {last, last};
}

std::vector<int> Choose3(const std::vector<double>& xs, double x) {
  const int n = static_cast<int>(xs.size());
  if (n < 3) throw std::runtime_error("Fuel table levels must have >=3 nodes.");
  int idx = 0;
  while (idx + 1 < n && xs[idx + 1] <= x) ++idx;
  int i0 = std::clamp(idx - 1, 0, n - 3);
  return {i0, i0 + 1, i0 + 2};
}

double QuadInterp(double x0,
                  double y0,
                  double x1,
                  double y1,
                  double x2,
                  double y2,
                  double x) {
  const double L0 = ((x - x1) * (x - x2)) / ((x0 - x1) * (x0 - x2));
  const double L1 = ((x - x0) * (x - x2)) / ((x1 - x0) * (x1 - x2));
  const double L2 = ((x - x0) * (x - x1)) / ((x2 - x0) * (x2 - x1));
  return y0 * L0 + y1 * L1 + y2 * L2;
}

double FuelRateKgPerHour(const FuelTable& tb, double altitude, double speed) {
  if (!tb.altitude_levels || !tb.speed_levels || !tb.consumption_data) {
    throw std::runtime_error("Fuel table pointer not initialized.");
  }
  const auto& alts = *tb.altitude_levels;
  const auto& spds = *tb.speed_levels;
  const auto& data = *tb.consumption_data;
  if (alts.empty() || spds.empty() || data.empty()) throw std::runtime_error("Empty fuel table.");

  altitude = std::clamp(altitude, alts.front(), alts.back());
  speed = std::clamp(speed, spds.front(), spds.back());

  const auto ai = Choose3(alts, altitude);
  const auto si = Choose3(spds, speed);

  double row_vals[3] = {0.0, 0.0, 0.0};
  for (int k = 0; k < 3; ++k) {
    const int a = ai[k];
    row_vals[k] = QuadInterp(spds[si[0]],
                             data[a][si[0]],
                             spds[si[1]],
                             data[a][si[1]],
                             spds[si[2]],
                             data[a][si[2]],
                             speed);
  }

  return QuadInterp(alts[ai[0]],
                    row_vals[0],
                    alts[ai[1]],
                    row_vals[1],
                    alts[ai[2]],
                    row_vals[2],
                    altitude);
}

double FuelUsedKg(double distance_m, double speed_mps, double altitude_m, const FuelTable& tb) {
  const double rate = FuelRateKgPerHour(tb, altitude_m, speed_mps);
  const double t_s = distance_m / std::max(speed_mps, 1e-6);
  return rate * (t_s / 3600.0);
}

double PointSegDistance(const V2& p, const V2& a, const V2& b) {
  const V2 ab = Sub(b, a);
  const double l2 = Dot(ab, ab);
  if (l2 < kEps) return Dist(p, a);
  double t = Dot(Sub(p, a), ab) / l2;
  t = std::clamp(t, 0.0, 1.0);
  const V2 proj = Add(a, Mul(ab, t));
  return Dist(p, proj);
}

bool PointInPoly(const V2& p, const Poly& poly) {
  bool inside = false;
  const int n = static_cast<int>(poly.pts.size());
  for (int i = 0, j = n - 1; i < n; j = i++) {
    const auto& a = poly.pts[i];
    const auto& b = poly.pts[j];
    const bool cross = ((a.y > p.y) != (b.y > p.y)) &&
                       (p.x < (b.x - a.x) * (p.y - a.y) / ((b.y - a.y) + 1e-30) + a.x);
    if (cross) inside = !inside;
  }
  return inside;
}

double PointPolyBoundaryDistance(const V2& p, const Poly& poly) {
  double best = std::numeric_limits<double>::infinity();
  const int n = static_cast<int>(poly.pts.size());
  for (int i = 0; i < n; ++i) {
    best = std::min(best, PointSegDistance(p, poly.pts[i], poly.pts[(i + 1) % n]));
  }
  return best;
}

bool IsThreatSafe(const V2& p, const std::vector<Poly>& threats, double buffer) {
  for (const auto& poly : threats) {
    if (poly.pts.size() < 3) continue;
    if (PointInPoly(p, poly)) return false;
    if (PointPolyBoundaryDistance(p, poly) < buffer - 1e-9) return false;
  }
  return true;
}

double MeetObjective(const Mission& in,
                     const V2& m,
                     double& d_t,
                     double& d_r,
                     double& d_g,
                     double& heading,
                     double& turn_t,
                     double& turn_r) {
  d_t = Dist(in.tanker.pos, m);
  d_r = Dist(in.receiver.pos, m);
  d_g = Dist(m, in.goal);
  heading = HeadingToTargetDeg(m, in.goal);
  turn_t = AngleDiffDeg(in.tanker.heading_deg, heading);
  turn_r = AngleDiffDeg(in.receiver.heading_deg, heading);

  return kDistWeight * (d_t + d_r) + kDistDiffWeight * std::fabs(d_t - d_r) +
         kGoalDistWeight * d_g + kTurnWeight * (turn_t + turn_r);
}

V2 FindCenterOnTR(const Mission& in) {
  V2 best = Add(in.tanker.pos, in.receiver.pos);
  best = Mul(best, 0.5);
  double best_obj = std::numeric_limits<double>::infinity();

  for (int i = 0; i <= 100; ++i) {
    const double a = static_cast<double>(i) / 100.0;
    const V2 m = Add(in.tanker.pos, Mul(Sub(in.receiver.pos, in.tanker.pos), a));
    double d_t, d_r, d_g, h, t1, t2;
    const double obj = MeetObjective(in, m, d_t, d_r, d_g, h, t1, t2);
    if (obj < best_obj) {
      best_obj = obj;
      best = m;
    }
  }
  return best;
}

MeetResult SearchMeetPoint(const Mission& in) {
  MeetResult best;
  const V2 center = FindCenterOnTR(in);
  for (double dx = -kSearchRadius; dx <= kSearchRadius + kEps; dx += kSearchStep) {
    for (double dy = -kSearchRadius; dy <= kSearchRadius + kEps; dy += kSearchStep) {
      const V2 m{center.x + dx, center.y + dy};
      double d_t, d_r, d_g, h, t1, t2;
      const double obj = MeetObjective(in, m, d_t, d_r, d_g, h, t1, t2);
      if (obj < best.objective) {
        best.feasible = true;
        best.meet = m;
        best.heading_to_goal_deg = h;
        best.tanker_dist = d_t;
        best.receiver_dist = d_r;
        best.goal_dist = d_g;
        best.tanker_turn_deg = t1;
        best.receiver_turn_deg = t2;
        best.objective = obj;
      }
    }
  }
  return best;
}

double OverSpeedPenalty(double v, double v_ref, double v_max) {
  if (v <= v_ref) return 0.0;
  const double x = (v - v_ref) / std::max(v_max - v_ref, 1e-6);
  return x * x;
}

SpeedResult OptimizeSpeed(const Mission& in,
                          const FuelTable& tanker_tb,
                          const FuelTable& receiver_tb,
                          const MeetResult& meet) {
  SpeedResult best;
  const double d_t = meet.tanker_dist;
  const double d_r = meet.receiver_dist;

  const double t_min = std::max(d_t / in.tanker.v_max, d_r / in.receiver.v_max);
  const double t_max = std::min(d_t / in.tanker.v_min, d_r / in.receiver.v_min);
  if (t_min > t_max + kEps) return best;

  struct Cand {
    double t{0.0}, vt{0.0}, vr{0.0}, ft{0.0}, fr{0.0}, fuel{0.0}, over{0.0};
  };
  std::vector<Cand> cands;

  auto push = [&](double t) {
    if (t <= 0.0) return;
    const double vt = d_t / t;
    const double vr = d_r / t;
    if (vt < in.tanker.v_min - 1e-6 || vt > in.tanker.v_max + 1e-6) return;
    if (vr < in.receiver.v_min - 1e-6 || vr > in.receiver.v_max + 1e-6) return;

    Cand c;
    c.t = t;
    c.vt = vt;
    c.vr = vr;
    c.ft = FuelUsedKg(d_t, vt, kReferenceAltitude, tanker_tb);
    c.fr = FuelUsedKg(d_r, vr, kReferenceAltitude, receiver_tb);
    c.fuel = c.ft + c.fr;
    c.over = OverSpeedPenalty(vt, kEconomicSpeed, in.tanker.v_max) +
             OverSpeedPenalty(vr, kEconomicSpeed, in.receiver.v_max);
    cands.push_back(c);
  };

  for (double t = t_min; t <= t_max + kEps; t += kTimeSearchStep) push(t);
  if (cands.empty() || std::fabs(cands.back().t - t_max) > 1e-8) push(t_max);
  if (cands.empty()) return best;

  double t_lo = cands.front().t, t_hi = cands.front().t;
  double f_lo = cands.front().fuel, f_hi = cands.front().fuel;
  double o_lo = cands.front().over, o_hi = cands.front().over;
  for (const auto& c : cands) {
    t_lo = std::min(t_lo, c.t);
    t_hi = std::max(t_hi, c.t);
    f_lo = std::min(f_lo, c.fuel);
    f_hi = std::max(f_hi, c.fuel);
    o_lo = std::min(o_lo, c.over);
    o_hi = std::max(o_hi, c.over);
  }
  auto norm = [](double x, double lo, double hi) {
    if (std::fabs(hi - lo) < 1e-12) return 0.0;
    return (x - lo) / (hi - lo);
  };

  for (const auto& c : cands) {
    const double obj = in.w_time * norm(c.t, t_lo, t_hi) + in.w_fuel * norm(c.fuel, f_lo, f_hi) +
                       in.w_safe * 0.0 + kOverspeedWeight * norm(c.over, o_lo, o_hi);
    if (obj < best.objective) {
      best.feasible = true;
      best.meet_time_s = c.t;
      best.tanker_speed = c.vt;
      best.receiver_speed = c.vr;
      best.tanker_fuel_to_meet = c.ft;
      best.receiver_fuel_to_meet = c.fr;
      best.total_fuel_to_meet = c.fuel;
      best.overspeed_penalty = c.over;
      best.objective = obj;
    }
  }

  return best;
}

HeightResult ChooseHeight(const Mission& in,
                          const FuelTable& tanker_tb,
                          const FuelTable& receiver_tb,
                          const MeetResult& meet,
                          const SpeedResult& speed) {
  HeightResult best;
  if (!speed.feasible) return best;

  struct C {
    double h{0.0}, ft{0.0}, fr{0.0}, fuel{0.0};
  };
  std::vector<C> cs;
  for (double h = 5000.0; h <= 10000.0 + kEps; h += 1000.0) {
    C c;
    c.h = h;
    c.ft = FuelUsedKg(meet.tanker_dist, speed.tanker_speed, h, tanker_tb);
    c.fr = FuelUsedKg(meet.receiver_dist, speed.receiver_speed, h, receiver_tb);
    c.fuel = c.ft + c.fr;
    cs.push_back(c);
  }
  if (cs.empty()) return best;

  double lo = cs.front().fuel, hi = cs.front().fuel;
  for (const auto& c : cs) {
    lo = std::min(lo, c.fuel);
    hi = std::max(hi, c.fuel);
  }
  auto norm = [&](double x) {
    if (std::fabs(hi - lo) < 1e-12) return 0.0;
    return (x - lo) / (hi - lo);
  };

  for (const auto& c : cs) {
    const double obj = in.w_fuel * norm(c.fuel) + in.w_safe * 0.0 + 1e-6 * std::fabs(c.h - kReferenceAltitude);
    if (obj < best.objective) {
      best.feasible = true;
      best.meet_altitude_m = c.h;
      best.tanker_fuel_to_meet = c.ft;
      best.receiver_fuel_to_meet = c.fr;
      best.total_fuel_to_meet = c.fuel;
      best.objective = obj;
    }
  }

  return best;
}

V2 PointOnSegByDist(const V2& a, const V2& b, double d_from_a) {
  const double L = Dist(a, b);
  if (L < kEps) return a;
  const double t = std::clamp(d_from_a / L, 0.0, 1.0);
  return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}

RefuelResult ChooseRefuel(const Mission& in,
                          const FuelTable& receiver_tb,
                          const MeetResult& meet,
                          const SpeedResult& speed,
                          const HeightResult& height) {
  RefuelResult out;

  const V2 M = meet.meet;
  const V2 G = in.goal;
  const double total_len = Dist(M, G);
  if (total_len < kEps) return out;

  const double fuel_at_meet = std::max(0.0, in.receiver.fuel_now - height.receiver_fuel_to_meet);
  const double threshold = kRefuelThresholdRatio * std::max(in.receiver.fuel_full, 1.0);

  const double rate_kgph = FuelRateKgPerHour(receiver_tb, height.meet_altitude_m, in.formation_speed);
  const double fuel_per_m = rate_kgph / 3600.0 / std::max(in.formation_speed, 1e-6);
  const double total_need = fuel_per_m * total_len;

  double ideal_s = 0.0;
  std::string mode;
  if (fuel_at_meet - total_need <= threshold + 1e-9) {
    const double burn_to_threshold = std::max(0.0, fuel_at_meet - threshold);
    ideal_s = std::clamp(burn_to_threshold / std::max(fuel_per_m, 1e-12), 0.0, total_len);
    mode = "threshold";
  } else {
    ideal_s = (total_len >= kRefuelFallbackToGoal) ? (total_len - kRefuelFallbackToGoal) : 0.0;
    mode = "fallback_200km";
  }

  const V2 ideal_p = PointOnSegByDist(M, G, ideal_s);
  auto make = [&](double s, const std::string& m) {
    RefuelResult r;
    r.feasible = true;
    r.dist_from_meet = s;
    r.dist_to_goal = total_len - s;
    r.refuel = PointOnSegByDist(M, G, s);
    r.receiver_fuel_at_refuel = fuel_at_meet - fuel_per_m * s;
    r.mode = m;
    return r;
  };

  if (IsThreatSafe(ideal_p, in.threats, kThreatBuffer)) return make(ideal_s, mode);

  bool found = false;
  double best_d = std::numeric_limits<double>::infinity();
  double best_s = 0.0;
  std::vector<double> samples;
  for (double s = 0.0; s <= total_len + kEps; s += kRefuelLineSearchStep) {
    samples.push_back(std::min(s, total_len));
  }
  samples.push_back(ideal_s);
  samples.push_back(0.0);
  samples.push_back(total_len);
  std::sort(samples.begin(), samples.end());
  samples.erase(std::unique(samples.begin(), samples.end(), [](double a, double b) { return std::fabs(a - b) < 1e-6; }),
                samples.end());

  for (double s : samples) {
    const V2 p = PointOnSegByDist(M, G, s);
    if (!IsThreatSafe(p, in.threats, kThreatBuffer)) continue;
    const double d = std::fabs(s - ideal_s);
    if (!found || d < best_d - 1e-9 || (std::fabs(d - best_d) < 1e-9 && s < best_s)) {
      found = true;
      best_d = d;
      best_s = s;
    }
  }
  if (!found) return out;

  const std::string adjusted = (mode == "threshold") ? "adjusted_from_threshold" : "adjusted_from_fallback";
  return make(best_s, adjusted);
}

SolveResult Solve(const Mission& in, const FuelTable& tanker_tb, const FuelTable& receiver_tb) {
  SolveResult s;
  s.meet = SearchMeetPoint(in);
  if (!s.meet.feasible) return s;

  s.speed = OptimizeSpeed(in, tanker_tb, receiver_tb, s.meet);
  if (!s.speed.feasible) return s;

  s.height = ChooseHeight(in, tanker_tb, receiver_tb, s.meet, s.speed);
  if (!s.height.feasible) return s;

  s.refuel = ChooseRefuel(in, receiver_tb, s.meet, s.speed, s.height);
  if (!s.refuel.feasible) return s;

  s.feasible = true;
  return s;
}

std::vector<std::vector<routeplan::Vec2>> BuildNoFly(const PlanningContext& ctx) {
  std::vector<std::vector<routeplan::Vec2>> out;
  const auto& nf = ctx.mission.operation_area.no_fly_zones;
  if (!nf.is_defined) return out;
  out.reserve(nf.zones_vertices_lla.size());
  for (const auto& poly_lla : nf.zones_vertices_lla) {
    if (poly_lla.size() < 3) continue;
    std::vector<routeplan::Vec2> poly;
    poly.reserve(poly_lla.size());
    for (const auto& p : poly_lla) {
      const double lat = p.lat_deg * kPi / 180.0;
      const double lon = p.lon_deg * kPi / 180.0;
      poly.push_back({kEarthRadiusM * lon, kEarthRadiusM * std::log(std::tan(kPi / 4.0 + lat / 2.0))});
    }
    out.push_back(std::move(poly));
  }
  return out;
}

std::vector<Poly> BuildThreats(const PlanningContext& ctx) {
  std::vector<Poly> out;
  const auto& nf = ctx.mission.operation_area.no_fly_zones;
  if (!nf.is_defined) return out;
  out.reserve(nf.zones_vertices_lla.size());
  for (const auto& poly_lla : nf.zones_vertices_lla) {
    if (poly_lla.size() < 3) continue;
    Poly poly;
    poly.pts.reserve(poly_lla.size());
    for (const auto& p : poly_lla) {
      const double lat = p.lat_deg * kPi / 180.0;
      const double lon = p.lon_deg * kPi / 180.0;
      poly.pts.push_back({kEarthRadiusM * lon, kEarthRadiusM * std::log(std::tan(kPi / 4.0 + lat / 2.0))});
    }
    out.push_back(std::move(poly));
  }
  return out;
}

V2 PolygonCentroid(const Polygon& poly) {
  V2 c{0.0, 0.0};
  if (poly.vertices_xy.empty()) return c;
  for (const auto& p : poly.vertices_xy) {
    c.x += p.x;
    c.y += p.y;
  }
  c.x /= static_cast<double>(poly.vertices_xy.size());
  c.y /= static_cast<double>(poly.vertices_xy.size());
  return c;
}

V2 PickGoalPoint(const PlanningContext& ctx, const AircraftConfig& receiver) {
  if (std::isfinite(ctx.racetrack.center_xy.x) && std::isfinite(ctx.racetrack.center_xy.y) &&
      (std::fabs(ctx.racetrack.center_xy.x) > 1.0 || std::fabs(ctx.racetrack.center_xy.y) > 1.0)) {
    return {ctx.racetrack.center_xy.x, ctx.racetrack.center_xy.y};
  }
  if (!ctx.safe_zone_selected.safe_zone_polygon_xy.vertices_xy.empty()) {
    return PolygonCentroid(ctx.safe_zone_selected.safe_zone_polygon_xy);
  }
  return {receiver.initial_position_xy.x, receiver.initial_position_xy.y};
}

std::vector<WaypointXYZ> SampleLine(const WaypointXYZ& a, const WaypointXYZ& b, double step) {
  std::vector<WaypointXYZ> out;
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double dz = b.z - a.z;
  const double d = std::sqrt(dx * dx + dy * dy);
  const int n = std::max(1, static_cast<int>(std::ceil(d / std::max(step, 1.0))));
  out.reserve(static_cast<size_t>(n) + 1);
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    out.push_back({a.x + t * dx, a.y + t * dy, a.z + t * dz});
  }
  return out;
}

void AppendPoint(std::vector<WaypointXYZ>& pts, const WaypointXYZ& p) {
  if (!pts.empty()) {
    const auto& b = pts.back();
    if (std::fabs(b.x - p.x) < 1e-6 && std::fabs(b.y - p.y) < 1e-6 && std::fabs(b.z - p.z) < 1e-6) return;
  }
  pts.push_back(p);
}

void AppendPoints(std::vector<WaypointXYZ>& dst, const std::vector<WaypointXYZ>& src) {
  for (const auto& p : src) AppendPoint(dst, p);
}

std::vector<WaypointXYZ> PlanSeg(const AircraftConfig& ac,
                                 const V2& start,
                                 double z0,
                                 double hdg0_deg,
                                 const V2& goal,
                                 double z1,
                                 double hdg1_deg,
                                 double speed,
                                 double turn_r,
                                 const std::vector<std::vector<routeplan::Vec2>>& obs) {
  routeplan::Pose3D s{start.x, start.y, z0, Deg2Rad(hdg0_deg)};
  routeplan::Pose3D g{goal.x, goal.y, z1, Deg2Rad(hdg1_deg)};

  routeplan::PlannerConfig cfg;
  cfg.turnRadius = std::max(10.0, turn_r);
  cfg.step = std::clamp(turn_r * 0.2, 200.0, 2000.0);
  cfg.finalStraightLen = std::max(0.0, cfg.step * 2.0);
  cfg.climbRate = std::max(0.1, ac.limits.climb_rate);
  cfg.descendRate = std::max(0.1, ac.limits.descent_rate);
  cfg.groundSpeed = std::max(1.0, speed);
  cfg.z_mid = z1;

  const auto r = routeplan::PlanRoute(s, g, obs, cfg);
  if (!r.route.empty()) {
    std::vector<WaypointXYZ> out;
    out.reserve(r.route.size());
    for (const auto& p : r.route) out.push_back({p.x, p.y, p.z});
    return out;
  }

  return SampleLine({s.x, s.y, s.z}, {g.x, g.y, g.z}, cfg.step);
}

std::vector<WaypointXYZ> PlanViaPoints(const AircraftConfig& ac,
                                       const std::vector<V2>& pts,
                                       double alt,
                                       double speed,
                                       double turn_r,
                                       const std::vector<std::vector<routeplan::Vec2>>& obs,
                                       double start_heading_deg,
                                       double end_heading_deg) {
  std::vector<WaypointXYZ> out;
  if (pts.size() < 2) return out;

  double h_in = start_heading_deg;
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    double h_out = end_heading_deg;
    if (i + 2 < pts.size()) h_out = HeadingToTargetDeg(pts[i + 1], pts[i + 2]);
    auto seg = PlanSeg(ac, pts[i], alt, h_in, pts[i + 1], alt, h_out, speed, turn_r, obs);
    AppendPoints(out, seg);
    h_in = h_out;
  }
  return out;
}

void WriteCsv(const fs::path& p, const std::vector<WaypointXYZ>& pts) {
  std::ofstream ofs(p);
  if (!ofs) throw std::runtime_error("Failed to write CSV: " + p.string());
  ofs << "x,y,z\n";
  ofs << std::fixed << std::setprecision(2);
  for (const auto& q : pts) ofs << q.x << ',' << q.y << ',' << q.z << '\n';
}

} // namespace

bool RunSuiduiBranch(const PlanningContext& base, const std::string& outputDir, std::string* errorMsg) {
  try {
    PlanningContext ctx = base;

    CoordinateTransformStage().Run(ctx);
    PreprocessStage().Run(ctx);
    SafeZoneSelectStage().Run(ctx);
    RacetrackBuildStage().Run(ctx);

    if (ctx.tankers.empty()) throw std::runtime_error("No tanker found for suidui branch.");
    if (ctx.receivers.empty()) throw std::runtime_error("No receiver found for suidui branch.");

    const AircraftConfig tanker_cfg = ctx.tanker;

    const auto tanker_tb = FuelTable{&tanker_cfg.fuel_table.altitude_levels,
                                     &tanker_cfg.fuel_table.speed_levels,
                                     &tanker_cfg.fuel_table.consumption_data};

    const auto obstacles = BuildNoFly(ctx);
    const auto threats = BuildThreats(ctx);
    const double turn_r = (ctx.mission.racecourse.orbit_radius > 1e-6) ? ctx.mission.racecourse.orbit_radius : 5000.0;

    double w_time = 0.5, w_fuel = 0.5, w_safe = 0.0;
    if (ctx.mission.prefs.weight_factors.size() >= 3) {
      w_time = ctx.mission.prefs.weight_factors[0];
      w_fuel = ctx.mission.prefs.weight_factors[1];
      w_safe = ctx.mission.prefs.weight_factors[2];
      const double s = w_time + w_fuel + w_safe;
      if (s > 1e-12) {
        w_time /= s;
        w_fuel /= s;
        w_safe /= s;
      }
    }

    const double formation_speed =
        (ctx.mission.racecourse.speed_mps > 1e-9) ? ctx.mission.racecourse.speed_mps : 160.0;

    nlohmann::json root;
    root["branch"] = "suidui";
    root["status"] = "ok";
    root["model"] = "0324suidui-integrated";
    root["results"] = nlohmann::json::array();

    fs::create_directories(outputDir);

    int success_count = 0;
    for (const auto& r_cfg : ctx.receivers) {
      auto rb_it = ctx.receiver_speed_bounds.find(r_cfg.id);
      const SpeedBounds rb = (rb_it == ctx.receiver_speed_bounds.end()) ? SpeedBounds{120.0, 170.0, 200.0} : rb_it->second;

      Mission in;
      in.tanker.pos = {tanker_cfg.initial_position_xy.x, tanker_cfg.initial_position_xy.y};
      in.tanker.heading_deg = NormDeg360(tanker_cfg.current_status.heading_deg);
      in.tanker.altitude_m = tanker_cfg.initial_position_lla.alt_m;
      in.tanker.v_cur = ClampPos(tanker_cfg.current_status.speed_mps, 160.0);
      in.tanker.v_min = (tanker_cfg.current_status.speed_min_mps > 1e-9)
                            ? tanker_cfg.current_status.speed_min_mps
                            : ClampPos(ctx.tanker_speed_bounds.min_speed_mps, 120.0);
      in.tanker.v_max = (tanker_cfg.current_status.speed_max_mps > 1e-9)
                            ? tanker_cfg.current_status.speed_max_mps
                            : ClampPos(ctx.tanker_speed_bounds.max_speed_mps, 200.0);
      if (in.tanker.v_max < in.tanker.v_min + 1e-9) in.tanker.v_max = in.tanker.v_min + 1.0;

      in.receiver.pos = {r_cfg.initial_position_xy.x, r_cfg.initial_position_xy.y};
      in.receiver.heading_deg = NormDeg360(r_cfg.current_status.heading_deg);
      in.receiver.altitude_m = r_cfg.initial_position_lla.alt_m;
      in.receiver.v_cur = ClampPos(r_cfg.current_status.speed_mps, 160.0);
      in.receiver.v_min = (r_cfg.current_status.speed_min_mps > 1e-9)
                              ? r_cfg.current_status.speed_min_mps
                              : ClampPos(rb.min_speed_mps, 120.0);
      in.receiver.v_max = (r_cfg.current_status.speed_max_mps > 1e-9)
                              ? r_cfg.current_status.speed_max_mps
                              : ClampPos(rb.max_speed_mps, 200.0);
      if (in.receiver.v_max < in.receiver.v_min + 1e-9) in.receiver.v_max = in.receiver.v_min + 1.0;

      in.receiver.fuel_now = ClampPos(r_cfg.current_status.fuel_kg, 1.0);
      in.receiver.fuel_full = ClampPos(r_cfg.max_fuel_kg > 1e-9 ? r_cfg.max_fuel_kg : r_cfg.current_status.max_fuel_kg,
                                       std::max(1.0, in.receiver.fuel_now));

      in.goal = PickGoalPoint(ctx, r_cfg);
      in.w_time = w_time;
      in.w_fuel = w_fuel;
      in.w_safe = w_safe;
      in.formation_speed = std::clamp(formation_speed, in.receiver.v_min, in.receiver.v_max);
      in.threats = threats;

      const auto receiver_tb = FuelTable{&r_cfg.fuel_table.altitude_levels,
                                         &r_cfg.fuel_table.speed_levels,
                                         &r_cfg.fuel_table.consumption_data};

      SolveResult ans = Solve(in, tanker_tb, receiver_tb);
      if (!ans.feasible) {
        nlohmann::json fail;
        fail["receiver_id"] = r_cfg.id;
        fail["success"] = 0;
        fail["error"] = "NO_FEASIBLE_SUIDUI_PLAN";
        root["results"].push_back(fail);
        continue;
      }

      const double tanker_rate_form = FuelRateKgPerHour(tanker_tb, ans.height.meet_altitude_m, in.formation_speed);
      const double receiver_rate_form = FuelRateKgPerHour(receiver_tb, ans.height.meet_altitude_m, in.formation_speed);
      const double t_form = ans.refuel.dist_from_meet / std::max(in.formation_speed, 1e-6);
      const double tanker_form = tanker_rate_form * t_form / 3600.0;
      const double receiver_form = receiver_rate_form * t_form / 3600.0;

      const double tanker_total = ans.height.tanker_fuel_to_meet + tanker_form;
      const double receiver_total = ans.height.receiver_fuel_to_meet + receiver_form;
      const double fueling_time = ans.speed.meet_time_s + ans.refuel.dist_from_meet / std::max(in.formation_speed, 1e-6);

      const V2 start_t{tanker_cfg.initial_position_xy.x, tanker_cfg.initial_position_xy.y};
      const V2 start_r{r_cfg.initial_position_xy.x, r_cfg.initial_position_xy.y};
      const std::vector<V2> tanker_path = {start_t, ans.meet.meet, ans.refuel.refuel, in.goal};
      const std::vector<V2> receiver_path = {start_r, ans.meet.meet, ans.refuel.refuel, in.goal};

      const auto tanker_traj = PlanViaPoints(tanker_cfg,
                                             tanker_path,
                                             ans.height.meet_altitude_m,
                                             ans.speed.tanker_speed,
                                             turn_r,
                                             obstacles,
                                             in.tanker.heading_deg,
                                             ans.meet.heading_to_goal_deg);
      const auto receiver_traj = PlanViaPoints(r_cfg,
                                               receiver_path,
                                               ans.height.meet_altitude_m,
                                               ans.speed.receiver_speed,
                                               turn_r,
                                               obstacles,
                                               in.receiver.heading_deg,
                                               ans.meet.heading_to_goal_deg);

      std::string tanker_csv;
      std::string receiver_csv;
      if (ctx.receivers.size() == 1) {
        tanker_csv = "tanker01.csv";
        receiver_csv = "receiver01.csv";
      } else {
        tanker_csv = "tanker_" + r_cfg.id + ".csv";
        receiver_csv = "receiver_" + r_cfg.id + ".csv";
      }
      WriteCsv(fs::path(outputDir) / tanker_csv, tanker_traj);
      WriteCsv(fs::path(outputDir) / receiver_csv, receiver_traj);

      nlohmann::json item;
      item["receiver_id"] = r_cfg.id;
      item["success"] = 1;
      item["meeting_point"] = {ans.meet.meet.x, ans.meet.meet.y};
      item["tanker_meeting_params"] = {{"speed", ans.speed.tanker_speed}, {"altitude", ans.height.meet_altitude_m}};
      item["receiver_meeting_params"] = {{"speed", ans.speed.receiver_speed}, {"altitude", ans.height.meet_altitude_m}};
      item["Receiver Cycling number"] = 0;
      item["team_fly_params"] = {{"speed", in.formation_speed}, {"altitude", ans.height.meet_altitude_m}};
      item["fueling_time"] = fueling_time;
      item["fuel_consumption"] = {{"tanker", tanker_total}, {"receiver", receiver_total}};
      item["fueling_params"] = {{"speed", in.formation_speed}, {"altitude", ans.height.meet_altitude_m}};

      item["debug"] = {
          {"goal_point", {in.goal.x, in.goal.y}},
          {"rendezvous_heading_deg", ans.meet.heading_to_goal_deg},
          {"rendezvous_mode", ans.refuel.mode},
          {"refuel_point", {ans.refuel.refuel.x, ans.refuel.refuel.y}},
          {"dist_meet_to_refuel_m", ans.refuel.dist_from_meet},
          {"dist_refuel_to_goal_m", ans.refuel.dist_to_goal},
          {"receiver_fuel_at_refuel_kg", ans.refuel.receiver_fuel_at_refuel},
          {"meet_time_s", ans.speed.meet_time_s},
          {"tanker_csv", tanker_csv},
          {"receiver_csv", receiver_csv},
      };

      root["results"].push_back(item);
      ++success_count;
    }

    nlohmann::json final_out;
    if (ctx.receivers.size() == 1 && root["results"].is_array() && root["results"].size() == 1) {
      const auto& one = root["results"][0];
      if (one.value("success", 0) == 1) {
        final_out = {
            {"meeting_point", one["meeting_point"]},
            {"tanker_meeting_params", one["tanker_meeting_params"]},
            {"receiver_meeting_params", one["receiver_meeting_params"]},
            {"Receiver Cycling number", one["Receiver Cycling number"]},
            {"team_fly_params", one["team_fly_params"]},
            {"fueling_time", one["fueling_time"]},
            {"fuel_consumption", one["fuel_consumption"]},
            {"fueling_params", one["fueling_params"]},
            {"debug", one["debug"]},
        };
      } else {
        final_out = {
            {"success", 0},
            {"error", one.value("error", "NO_FEASIBLE_SUIDUI_PLAN")},
        };
      }
    } else {
      root["success_count"] = success_count;
      final_out = root;
    }

    std::ofstream ofs(fs::path(outputDir) / "output.json");
    if (!ofs) throw std::runtime_error("Failed to write output.json");
    ofs << final_out.dump(2) << "\n";

    return true;
  } catch (const std::exception& e) {
    if (errorMsg) *errorMsg = e.what();
    return false;
  }
}

} // namespace refuel::branch
