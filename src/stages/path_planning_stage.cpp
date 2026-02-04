#include "stages/path_planning_stage.hpp"

// NOTE: Adjust this include path as needed in your repo layout.
#include "stages/route_planner.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace refuel {

namespace {

static constexpr double kPi = 3.14159265358979323846;

inline double Deg2Rad(double deg) { return deg * kPi / 180.0; }

inline double ClampMin(double v, double lo) { return (v < lo) ? lo : v; }

inline double Dist2(double ax, double ay, double bx, double by) {
  const double dx = ax - bx;
  const double dy = ay - by;
  return dx * dx + dy * dy;
}

inline bool NearlyEqual(double a, double b, double eps = 1e-6) {
  return std::fabs(a - b) <= eps;
}

inline bool NearlySameXYZ(const WaypointXYZ& a, const WaypointXYZ& b, double eps = 1e-6) {
  return NearlyEqual(a.x, b.x, eps) && NearlyEqual(a.y, b.y, eps) && NearlyEqual(a.z, b.z, eps);
}

void AppendPoint(Trajectory& t, const WaypointXYZ& p) {
  if (!t.points.empty() && NearlySameXYZ(t.points.back(), p)) return;
  t.points.push_back(p);
}

void AppendPoints(Trajectory& t, const std::vector<WaypointXYZ>& pts) {
  for (const auto& p : pts) AppendPoint(t, p);
}

std::vector<WaypointXYZ> SampleStraightLine(const WaypointXYZ& start, const WaypointXYZ& goal, double step_m) {
  std::vector<WaypointXYZ> out;
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double dz = goal.z - start.z;
  const double dist = std::sqrt(dx * dx + dy * dy);
  if (dist <= 1e-9) {
    out.push_back(start);
    if (!NearlySameXYZ(start, goal)) out.push_back(goal);
    return out;
  }

  const int n = std::max(1, static_cast<int>(std::ceil(dist / ClampMin(step_m, 1.0))));
  out.reserve(static_cast<std::size_t>(n) + 1);
  for (int i = 0; i <= n; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(n);
    out.push_back({start.x + u * dx, start.y + u * dy, start.z + u * dz});
  }
  return out;
}

// Generate one racetrack loop (clockwise by default), then reverse for counter-clockwise.
std::vector<WaypointXYZ> GenerateRacetrackLoop(const RacetrackPlan& rt,
                                               const std::string& turn_direction,
                                               double z,
                                               double step_m) {
  std::vector<WaypointXYZ> local;

  const double L = rt.length_m;
  const double R = rt.radius_m;
  if (L <= 1e-6 || R <= 1e-6) return local;

  step_m = ClampMin(step_m, 1.0);
  const double halfL = 0.5 * L;

  const int nStraight = std::max(2, static_cast<int>(std::ceil(L / step_m)));
  const double arcLen = kPi * R;
  const int nArc = std::max(6, static_cast<int>(std::ceil(arcLen / step_m)));

  // CW loop: top straight (left->right), right arc (top->bottom), bottom straight (right->left), left arc (bottom->top)
  local.reserve(static_cast<std::size_t>(2 * nStraight + 2 * nArc + 4));

  // Top straight
  for (int i = 0; i <= nStraight; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(nStraight);
    local.push_back({-halfL + u * L, R, z});
  }

  // Right arc: angle from +90 to -90 (CW)
  for (int i = 1; i <= nArc; ++i) {
    const double a = (kPi / 2.0) - (kPi * static_cast<double>(i) / static_cast<double>(nArc));
    local.push_back({halfL + R * std::cos(a), 0.0 + R * std::sin(a), z});
  }

  // Bottom straight
  for (int i = 1; i <= nStraight; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(nStraight);
    local.push_back({halfL - u * L, -R, z});
  }

  // Left arc: angle from -90 to +90
  for (int i = 1; i <= nArc; ++i) {
    const double a = (-kPi / 2.0) + (kPi * static_cast<double>(i) / static_cast<double>(nArc));
    local.push_back({-halfL + R * std::cos(a), 0.0 + R * std::sin(a), z});
  }

  // Remove closing duplicate if it exists.
  if (local.size() >= 2 && NearlySameXYZ(local.front(), local.back(), 1e-4)) {
    local.pop_back();
  }

  // Rotate + translate to world
  const double theta = Deg2Rad(rt.orientation_deg);
  const double c = std::cos(theta);
  const double s = std::sin(theta);

  std::vector<WaypointXYZ> world;
  world.reserve(local.size());
  for (const auto& p : local) {
    const double xr = c * p.x - s * p.y + rt.center_xy.x;
    const double yr = s * p.x + c * p.y + rt.center_xy.y;
    world.push_back({xr, yr, p.z});
  }

  // Decide direction.
  const bool is_clockwise = (turn_direction.find("shun") != std::string::npos) ||
                            (turn_direction.find("clock") != std::string::npos);
  if (!is_clockwise) {
    std::reverse(world.begin(), world.end());
  }

  return world;
}

std::size_t FindClosestIndex(const std::vector<WaypointXYZ>& pts, double x, double y) {
  if (pts.empty()) return 0;
  std::size_t best = 0;
  double bestD = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const double d = Dist2(pts[i].x, pts[i].y, x, y);
    if (d < bestD) {
      bestD = d;
      best = i;
    }
  }
  return best;
}

std::vector<WaypointXYZ> RotateLoopToStart(const std::vector<WaypointXYZ>& loop, std::size_t start_idx) {
  std::vector<WaypointXYZ> out;
  if (loop.empty()) return out;
  start_idx = std::min(start_idx, loop.size() - 1);
  out.reserve(loop.size());
  for (std::size_t i = 0; i < loop.size(); ++i) {
    out.push_back(loop[(start_idx + i) % loop.size()]);
  }
  return out;
}

// Segment along loop direction from start_idx to end_idx (inclusive), wrapping if needed.
std::vector<WaypointXYZ> SegmentForwardInclusive(const std::vector<WaypointXYZ>& loop,
                                                 std::size_t start_idx,
                                                 std::size_t end_idx) {
  std::vector<WaypointXYZ> seg;
  if (loop.empty()) return seg;
  start_idx = std::min(start_idx, loop.size() - 1);
  end_idx = std::min(end_idx, loop.size() - 1);

  seg.push_back(loop[start_idx]);
  std::size_t i = start_idx;
  while (i != end_idx) {
    i = (i + 1) % loop.size();
    seg.push_back(loop[i]);
    // Safety: should finish within one full loop.
    if (seg.size() > loop.size() + 1) break;
  }
  return seg;
}

double HeadingAtIndexRad(const std::vector<WaypointXYZ>& loop, std::size_t idx) {
  if (loop.size() < 2) return 0.0;
  idx = std::min(idx, loop.size() - 1);
  const std::size_t j = (idx + 1) % loop.size();
  const double dx = loop[j].x - loop[idx].x;
  const double dy = loop[j].y - loop[idx].y;
  return std::atan2(dy, dx);
}

std::vector<WaypointXYZ> PlanTransferWithRoutePlanner(const AircraftConfig& ac,
                                                     const Vec3& goal_xy,
                                                     double goal_z,
                                                     double goal_heading_rad,
                                                     double meeting_speed_mps,
                                                     double turn_radius_m,
                                                     double step_m) {
  const routeplan::Pose3D start{
      ac.initial_position_xy.x,
      ac.initial_position_xy.y,
      ac.initial_position_lla.alt_m,
      Deg2Rad(ac.current_status.heading_deg)};

  const routeplan::Pose3D goal{goal_xy.x, goal_xy.y, goal_z, goal_heading_rad};

  routeplan::PlannerConfig cfg;
  cfg.turnRadius = ClampMin(turn_radius_m, 10.0);
  cfg.step = ClampMin(step_m, 10.0);
  cfg.finalStraightLen = std::max(0.0, cfg.step * 2.0);

  cfg.climbRate = ClampMin(ac.limits.climb_rate, 0.1);
  cfg.descendRate = ClampMin(ac.limits.descent_rate, 0.1);

  const double gs = (meeting_speed_mps > 0.0) ? meeting_speed_mps : ac.current_status.speed_mps;
  cfg.groundSpeed = ClampMin(gs, 1.0);

  // Use the meeting altitude as mid altitude by default for a stable profile.
  cfg.z_mid = goal_z;

  cfg.allowSplineFallback = true;

  const std::vector<std::vector<routeplan::Vec2>> no_obstacles;
  const routeplan::PlanResult res = routeplan::PlanRoute(start, goal, no_obstacles, cfg);

  std::vector<WaypointXYZ> out;
  if (!res.route.empty()) {
    out.reserve(res.route.size());
    for (const auto& p : res.route) {
      out.push_back({p.x, p.y, p.z});
    }
    return out;
  }

  // Robust fallback: straight-line sampling.
  return SampleStraightLine({start.x, start.y, start.z}, {goal.x, goal.y, goal.z}, cfg.step);
}

void AppendLoopNTimes(Trajectory& t, const std::vector<WaypointXYZ>& loop, int loops) {
  if (loop.empty() || loops <= 0) return;
  for (int k = 0; k < loops; ++k) {
    // Avoid duplicating the first point of each loop after the first.
    const std::size_t start = (k == 0) ? 0 : 1;
    for (std::size_t i = start; i < loop.size(); ++i) {
      AppendPoint(t, loop[i]);
    }
  }
}

} // namespace

void PathPlanningStage::Run(PlanningContext& ctx) {
  ctx.trajectories.clear();

  // Loops count for tanker = max(receiver cycling_number)
  int max_cycling = 0;
  for (const auto& kv : ctx.rendezvous.receivers) {
    max_cycling = std::max(max_cycling, kv.second.cycling_number);
  }

  const std::string turn_dir = ctx.mission.racecourse.turn_direction;
  const double turn_radius = (ctx.racetrack.radius_m > 0.0) ? ctx.racetrack.radius_m : 5000.0;
  const double step_transfer = std::clamp(turn_radius * 0.20, 200.0, 2000.0);
  const double step_loop = std::clamp(turn_radius * 0.10, 100.0, 1500.0);

  // =========================
  // 1) Tanker trajectory
  // =========================
  {
    Trajectory t;
    t.aircraft_id = ctx.tanker.id;

    const double tanker_meet_z = (ctx.rendezvous.tanker_meeting_altitude_m > 0.0)
                                     ? ctx.rendezvous.tanker_meeting_altitude_m
                                     : ctx.racetrack.altitude_m;

    Vec3 entry = ctx.rendezvous.tanker_entry_point_xy;
    if (NearlyEqual(entry.x, 0.0) && NearlyEqual(entry.y, 0.0) && !ctx.racetrack.entrypoints_xy.empty()) {
      entry = ctx.racetrack.entrypoints_xy.front();
    }

    // Generate racetrack loop at tanker meeting altitude.
    const std::vector<WaypointXYZ> loop_raw = GenerateRacetrackLoop(ctx.racetrack, turn_dir, tanker_meet_z, step_loop);

    // Goal heading (radians) for transfer: use provided entry direction if available; otherwise infer from racetrack tangent.
    double goal_heading_rad = Deg2Rad(ctx.rendezvous.tanker_entry_direction_deg);
    if (std::fabs(ctx.rendezvous.tanker_entry_direction_deg) < 1e-9 && !loop_raw.empty()) {
      const std::size_t idx = FindClosestIndex(loop_raw, entry.x, entry.y);
      goal_heading_rad = HeadingAtIndexRad(loop_raw, idx);
    }

    const bool skip_transfer = ctx.tanker_rt_decision.reduce_rendezvous ||
                               (ctx.tanker_rt_decision.tanker_on_racetrack && !ctx.tanker_rt_decision.need_transfer);

    if (skip_transfer) {
      // If already on racetrack and no transfer is needed, start directly from current position.
      AppendPoint(t, {ctx.tanker.initial_position_xy.x, ctx.tanker.initial_position_xy.y, tanker_meet_z});
    } else {
      const auto transfer = PlanTransferWithRoutePlanner(
          ctx.tanker, entry, tanker_meet_z, goal_heading_rad, ctx.rendezvous.tanker_meeting_speed_mps, turn_radius, step_transfer);
      AppendPoints(t, transfer);
    }

    // Append racetrack loops for tanker (equal to max cycling_number).
    if (!loop_raw.empty() && max_cycling > 0) {
      const Vec3 start_on_track = skip_transfer ? ctx.tanker.initial_position_xy : entry;
      const std::size_t start_idx = FindClosestIndex(loop_raw, start_on_track.x, start_on_track.y);
      const auto loop = RotateLoopToStart(loop_raw, start_idx);
      AppendLoopNTimes(t, loop, max_cycling);
    }

    // Ensure non-empty.
    if (t.points.empty()) {
      AppendPoint(t, {ctx.tanker.initial_position_xy.x, ctx.tanker.initial_position_xy.y, ctx.tanker.initial_position_lla.alt_m});
    }

    ctx.trajectories.push_back(std::move(t));
  }

  // =========================
  // 2) Receiver trajectories
  // =========================
  for (const auto& r : ctx.receivers) {
    Trajectory t;
    t.aircraft_id = r.id;

    const auto it = ctx.rendezvous.receivers.find(r.id);
    if (it == ctx.rendezvous.receivers.end()) {
      // Fallback: at least one point.
      AppendPoint(t, {r.initial_position_xy.x, r.initial_position_xy.y, r.initial_position_lla.alt_m});
      ctx.trajectories.push_back(std::move(t));
      continue;
    }

    const auto& rr = it->second;
    const double meet_z = (rr.meeting_altitude_m > 0.0) ? rr.meeting_altitude_m : ctx.racetrack.altitude_m;

    const Vec3 waiting = rr.waiting_point_xy;
    const Vec3 coordinate = rr.coordinate_xy;

    // Racetrack loop at receiver meeting altitude.
    const std::vector<WaypointXYZ> loop_raw = GenerateRacetrackLoop(ctx.racetrack, turn_dir, meet_z, step_loop);

    // Infer waiting tangent as goal heading for smooth approach.
    double waiting_heading_rad = 0.0;
    if (!loop_raw.empty()) {
      const std::size_t idxW = FindClosestIndex(loop_raw, waiting.x, waiting.y);
      waiting_heading_rad = HeadingAtIndexRad(loop_raw, idxW);
    } else {
      waiting_heading_rad = std::atan2(waiting.y - r.initial_position_xy.y, waiting.x - r.initial_position_xy.x);
    }

    // Transfer: initial -> waiting
    const auto transfer = PlanTransferWithRoutePlanner(
        r, waiting, meet_z, waiting_heading_rad, rr.meeting_speed_mps, turn_radius, step_transfer);
    AppendPoints(t, transfer);

    // Waiting loops: cycling_number laps on racetrack (starting near waiting point).
    if (!loop_raw.empty() && rr.cycling_number > 0) {
      const std::size_t idxW = FindClosestIndex(loop_raw, waiting.x, waiting.y);
      const auto loop = RotateLoopToStart(loop_raw, idxW);
      AppendLoopNTimes(t, loop, rr.cycling_number);
    }

    // Final segment on racetrack from waiting to meeting coordinate (if they differ).
    if (!loop_raw.empty()) {
      const std::size_t idxW = FindClosestIndex(loop_raw, waiting.x, waiting.y);
      const std::size_t idxC = FindClosestIndex(loop_raw, coordinate.x, coordinate.y);
      const auto seg = SegmentForwardInclusive(loop_raw, idxW, idxC);
      AppendPoints(t, seg);
    } else {
      // No racetrack geometry: direct to coordinate.
      if (!t.points.empty()) {
        const WaypointXYZ last = t.points.back();
        const auto seg = SampleStraightLine(last, {coordinate.x, coordinate.y, meet_z}, step_transfer);
        AppendPoints(t, seg);
      } else {
        AppendPoint(t, {coordinate.x, coordinate.y, meet_z});
      }
    }

    if (t.points.empty()) {
      AppendPoint(t, {r.initial_position_xy.x, r.initial_position_xy.y, r.initial_position_lla.alt_m});
    }

    ctx.trajectories.push_back(std::move(t));
  }
}

} // namespace refuel
