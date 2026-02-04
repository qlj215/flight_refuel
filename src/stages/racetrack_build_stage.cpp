#include "stages/racetrack_build_stage.hpp"

// Integrate the real racetrack model (racetrack::PlanRacetrack)
// while keeping the external I/O contract unchanged.
//
// IMPORTANT ABOUT INCLUDES:
// - Put racetrack.h under your project's include path (recommended: include/stages/racetrack.h)
// - Then keep this include as "stages/racetrack.h".
#include "stages/racetrack.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iterator>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

// (removed anonymous namespace wrapper for MSYS2/MinGW parse robustness)

static constexpr double kPI = 3.14159265358979323846;

// ---------- detection idiom ----------
template <class T, template <class> class Op, class = void>
struct is_detected : std::false_type {};
template <class T, template <class> class Op>
struct is_detected<T, Op, std::void_t<Op<T>>> : std::true_type {};
template <class T, template <class> class Op>
static constexpr bool is_detected_v = is_detected<T, Op>::value;

// ---------- small math helpers ----------
static inline double sqr(double x) { return x * x; }

template <class V2Like>
static inline double dist2_xy(const V2Like& a, const V2Like& b) {
  return sqr(a.x - b.x) + sqr(a.y - b.y);
}

// ---------- conversions to racetrack::Vec2 ----------
template <class V2Like>
static inline racetrack::Vec2 ToRTVec2(const V2Like& p) {
  return racetrack::Vec2{p.x, p.y};
}

// ---------- robust Range2 reader (supports tuple-like, lo/hi, min/max, [0]/[1]) ----------
template <typename T, typename = void>
struct is_tuple_like : std::false_type {};
template <typename T>
struct is_tuple_like<T, std::void_t<decltype(std::tuple_size<T>::value)>> : std::true_type {};

template <class R>
using op_lo = decltype(std::declval<R>().lo);
template <class R>
using op_hi = decltype(std::declval<R>().hi);
template <class R>
using op_min = decltype(std::declval<R>().min);
template <class R>
using op_max = decltype(std::declval<R>().max);
template <class R>
using op_size = decltype(std::declval<R>().size());
template <class R>
using op_index0 = decltype(std::declval<R>()[0]);
template <class R>
using op_index1 = decltype(std::declval<R>()[1]);

template <typename Range>
static bool ReadRange2(const Range& r, double& lo, double& hi) {
  if constexpr (is_tuple_like<Range>::value) {
    lo = static_cast<double>(std::get<0>(r));
    hi = static_cast<double>(std::get<1>(r));
    return true;
  } else if constexpr (is_detected_v<Range, op_lo> && is_detected_v<Range, op_hi>) {
    lo = static_cast<double>(r.lo);
    hi = static_cast<double>(r.hi);
    return true;
  } else if constexpr (is_detected_v<Range, op_min> && is_detected_v<Range, op_max>) {
    lo = static_cast<double>(r.min);
    hi = static_cast<double>(r.max);
    return true;
  } else if constexpr (is_detected_v<Range, op_size> && is_detected_v<Range, op_index0> &&
                       is_detected_v<Range, op_index1>) {
    if (r.size() < 2) return false;
    lo = static_cast<double>(r[0]);
    hi = static_cast<double>(r[1]);
    return true;
  } else {
    return false;
  }
}

// ---------- Safe polygon extraction ----------
template <class Poly>
using op_points = decltype(std::declval<Poly>().points);
template <class Poly>
using op_vertices = decltype(std::declval<Poly>().vertices);
template <class Poly>
using op_vertices_xy = decltype(std::declval<Poly>().vertices_xy);
template <class Poly>
using op_pts = decltype(std::declval<Poly>().pts);

template <class Poly>
using op_begin = decltype(std::begin(std::declval<const Poly&>()));
template <class Poly>
using op_end = decltype(std::end(std::declval<const Poly&>()));

template <class Poly>
static void FillSafePolygon(const Poly& poly, std::vector<racetrack::Vec2>& out) {
  out.clear();

  // Common patterns seen in your codebase:
  // - refuel::Polygon { std::vector<refuel::Vec2> points; }
  // - refuel::Polygon { std::vector<refuel::Vec2> vertices; }
  // - refuel::Polygon { std::vector<refuel::Vec2> pts; }
  // - or refuel::Polygon itself is directly iterable
  if constexpr (is_detected_v<Poly, op_points>) {
    for (const auto& p : poly.points) out.push_back(ToRTVec2(p));
  } else if constexpr (is_detected_v<Poly, op_vertices>) {
    for (const auto& p : poly.vertices) out.push_back(ToRTVec2(p));
  } else if constexpr (is_detected_v<Poly, op_vertices_xy>) {
    // refuel::Polygon in types.hpp: std::vector<refuel::Vec3> vertices_xy
    for (const auto& p : poly.vertices_xy) out.push_back(ToRTVec2(p));
  } else if constexpr (is_detected_v<Poly, op_pts>) {
    for (const auto& p : poly.pts) out.push_back(ToRTVec2(p));
  } else if constexpr (is_detected_v<Poly, op_begin> && is_detected_v<Poly, op_end>) {
    for (const auto& p : poly) out.push_back(ToRTVec2(p));
  } else {
    // If you hit this, paste the definition of `refuel::Polygon` and I'll add the accessor.
    static_assert(sizeof(Poly) == 0, "Unsupported refuel::Polygon storage. Paste refuel::Polygon definition.");
  }
}

// ---------- Altitude/Speed extraction (compile-safe, falls back if fields differ) ----------
// ---------- altitude/speed extraction (based on ctx, not mission) ----------
// NOTE:
// In your current skeleton (types.hpp), refuel::MissionInput does NOT carry a "holding_zone" or explicit
// cruise altitude/speed constraints. Therefore we derive reasonable defaults from the tanker
// state/performance tables, while keeping the stage I/O contract unchanged.
//
// Strategy (edit later if you add mission fields):
// - speed_mps: tanker.current_status.speed_mps (if > 0) else median(cruise_perf.speed_levels) else 130
// - altitude_m: tanker.initial_position_lla.alt_m (if > 0) else median(cruise_perf.altitude_levels) else 7000
// - clamp altitude by tanker.limits.max_altitude when provided.
static double MedianOf(std::vector<double> v) {
  if (v.empty()) return 0.0;
  const std::size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
  return v[mid];
}

template <class Tanker>
using op_tk_current_speed = decltype(std::declval<const Tanker&>().current_status.speed_mps);
template <class Tanker>
using op_tk_cruise_speed_levels = decltype(std::declval<const Tanker&>().cruise_perf.speed_levels);
template <class Tanker>
using op_tk_alt_lla = decltype(std::declval<const Tanker&>().initial_position_lla.alt_m);
template <class Tanker>
using op_tk_cruise_alt_levels = decltype(std::declval<const Tanker&>().cruise_perf.altitude_levels);
template <class Tanker>
using op_tk_max_alt = decltype(std::declval<const Tanker&>().limits.max_altitude);

// ---------- mission.racetrack.{speed, altitude, angle} (new fields in mission.json) ----------
template <class Mission>
using op_m_racetrack_speed = decltype(std::declval<const Mission&>().racetrack.speed);
template <class Mission>
using op_m_racetrack_speed_mps = decltype(std::declval<const Mission&>().racetrack.speed_mps);
template <class Mission>
using op_m_racetrack_altitude = decltype(std::declval<const Mission&>().racetrack.altitude);
template <class Mission>
using op_m_racetrack_altitude_m = decltype(std::declval<const Mission&>().racetrack.altitude_m);
template <class Mission>
using op_m_racetrack_angle = decltype(std::declval<const Mission&>().racetrack.angle);
template <class Mission>
using op_m_racetrack_angle_deg = decltype(std::declval<const Mission&>().racetrack.angle_deg);

// Alternative flat naming patterns
template <class Mission>
using op_m_racetrack_speed_flat = decltype(std::declval<const Mission&>().racetrack_speed);
template <class Mission>
using op_m_altitude_flat = decltype(std::declval<const Mission&>().altitude);

template <class Mission>
using op_m_racecourse_entry_angle = decltype(std::declval<const Mission&>().racecourse.entry_angle);
template <class Mission>
using op_m_racecourse_entry_angle_deg = decltype(std::declval<const Mission&>().racecourse.entry_angle_deg);

// ---------- receiver max fuel & speed ----------
template <class Receiver>
using op_rcv_max_fuel = decltype(std::declval<const Receiver&>().max_fuel);
template <class Receiver>
using op_rcv_max_fuel_kg = decltype(std::declval<const Receiver&>().max_fuel_kg);
template <class Receiver>
using op_rcv_current_max_fuel_kg = decltype(std::declval<const Receiver&>().current_status.max_fuel_kg);
template <class Receiver>
using op_rcv_current_speed = decltype(std::declval<const Receiver&>().current_status.speed_mps);

// ---------- tanker fuel transfer speed ----------
template <class Tanker>
using op_tk_fuel_transfer_speed = decltype(std::declval<const Tanker&>().fuel_transfer_speed);
template <class Tanker>
using op_tk_fuel_transfer_speed_kgps = decltype(std::declval<const Tanker&>().fuel_transfer_speed_kgps);
template <class Tanker>
using op_tk_current_fuel_transfer_speed = decltype(std::declval<const Tanker&>().current_status.fuel_transfer_speed);
template <class Tanker>
using op_tk_current_fuel_transfer_speed_kgps = decltype(std::declval<const Tanker&>().performance_limits.fuel_transfer_speed);

template <class Ctx>
static void ExtractAltitudeSpeedFromCtx(const Ctx& ctx, double& altitude_m, double& speed_mps) {
  altitude_m = 7000.0;
  speed_mps  = 130.0;

// Prefer mission.json racetrack params if refuel::MissionInput stores them.
// mission1.json example: "racetrack": { "angle": 30, "altitude": 7000, "speed": 130 }
const auto& ms = ctx.mission;

if constexpr (is_detected_v<decltype(ms), op_m_racetrack_speed>) {
  if (ms.racetrack.speed > 1e-3) speed_mps = static_cast<double>(ms.racetrack.speed);
} else if constexpr (is_detected_v<decltype(ms), op_m_racetrack_speed_mps>) {
  if (ms.racetrack.speed_mps > 1e-3) speed_mps = static_cast<double>(ms.racetrack.speed_mps);
} else if constexpr (is_detected_v<decltype(ms), op_m_racetrack_speed_flat>) {
  if (ms.racetrack_speed > 1e-3) speed_mps = static_cast<double>(ms.racetrack_speed);
}

if constexpr (is_detected_v<decltype(ms), op_m_racetrack_altitude>) {
  if (ms.racetrack.altitude > 1.0) altitude_m = static_cast<double>(ms.racetrack.altitude);
} else if constexpr (is_detected_v<decltype(ms), op_m_racetrack_altitude_m>) {
  if (ms.racetrack.altitude_m > 1.0) altitude_m = static_cast<double>(ms.racetrack.altitude_m);
} else if constexpr (is_detected_v<decltype(ms), op_m_altitude_flat>) {
  if (ms.altitude > 1.0) altitude_m = static_cast<double>(ms.altitude);
}


  const auto& tk = ctx.tanker;

  // speed
  if constexpr (is_detected_v<decltype(tk), op_tk_current_speed>) {
    if (tk.current_status.speed_mps > 1e-3) {
      speed_mps = tk.current_status.speed_mps;
    }
  }
  if constexpr (is_detected_v<decltype(tk), op_tk_cruise_speed_levels>) {
    if (!(tk.cruise_perf.speed_levels.empty()) && speed_mps <= 1e-3) {
      speed_mps = MedianOf(tk.cruise_perf.speed_levels);
    }
  }

  // altitude
  if constexpr (is_detected_v<decltype(tk), op_tk_alt_lla>) {
    if (tk.initial_position_lla.alt_m > 1.0) {
      altitude_m = tk.initial_position_lla.alt_m;
    }
  }
  if constexpr (is_detected_v<decltype(tk), op_tk_cruise_alt_levels>) {
    if (!(tk.cruise_perf.altitude_levels.empty()) && altitude_m <= 1.0) {
      altitude_m = MedianOf(tk.cruise_perf.altitude_levels);
    }
  }

  // clamp by max altitude
  if constexpr (is_detected_v<decltype(tk), op_tk_max_alt>) {
    if (tk.limits.max_altitude > 1e-3) {
      altitude_m = std::min(altitude_m, tk.limits.max_altitude);
    }
  }
}

template <class Mission>
static double ExtractRacetrackAngleDegFromMission(const Mission& ms) {
  double angle_deg = 30.0;

  if constexpr (is_detected_v<decltype(ms), op_m_racetrack_angle>) {
    angle_deg = static_cast<double>(ms.racetrack.angle);
  } else if constexpr (is_detected_v<decltype(ms), op_m_racetrack_angle_deg>) {
    angle_deg = static_cast<double>(ms.racetrack.angle_deg);
  } else if constexpr (is_detected_v<decltype(ms), op_m_racecourse_entry_angle>) {
    angle_deg = static_cast<double>(ms.racecourse.entry_angle);
  } else if constexpr (is_detected_v<decltype(ms), op_m_racecourse_entry_angle_deg>) {
    angle_deg = static_cast<double>(ms.racecourse.entry_angle_deg);
  }

  return angle_deg;
}

template <class Receiver>
static std::optional<double> TryGetMaxFuelKg(const Receiver& r) {
  if constexpr (is_detected_v<Receiver, op_rcv_max_fuel>) {
    if (r.max_fuel > 0.0) return static_cast<double>(r.max_fuel);
  }
  if constexpr (is_detected_v<Receiver, op_rcv_max_fuel_kg>) {
    if (r.max_fuel_kg > 0.0) return static_cast<double>(r.max_fuel_kg);
  }
  if constexpr (is_detected_v<Receiver, op_rcv_current_max_fuel_kg>) {
    if (r.current_status.max_fuel_kg > 0.0) return static_cast<double>(r.current_status.max_fuel_kg);
  }
  return std::nullopt;
}

template <class Ctx>
static double ExtractMaxFuelKgFromCtx(const Ctx& ctx) {
  double max_fuel_kg = 0.0;
  for (const auto& r : ctx.receivers) {
    if (auto mf = TryGetMaxFuelKg(r)) {
      if (*mf > max_fuel_kg) max_fuel_kg = *mf;
    }
  }
  if (max_fuel_kg <= 0.0) max_fuel_kg = 7000.0; // fallback placeholder
  return max_fuel_kg;
}

template <class Tanker>
static std::optional<double> TryGetFuelTransferSpeedKgps(const Tanker& tk) {
  if constexpr (is_detected_v<Tanker, op_tk_fuel_transfer_speed_kgps>) {
    if (tk.fuel_transfer_speed_kgps > 1e-6) return static_cast<double>(tk.fuel_transfer_speed_kgps);
  }
  if constexpr (is_detected_v<Tanker, op_tk_fuel_transfer_speed>) {
    if (tk.fuel_transfer_speed > 1e-6) return static_cast<double>(tk.fuel_transfer_speed);
  }
  if constexpr (is_detected_v<Tanker, op_tk_current_fuel_transfer_speed_kgps>) {
    if (tk.current_status.fuel_transfer_speed_kgps > 1e-6)
      return static_cast<double>(tk.current_status.fuel_transfer_speed_kgps);
  }
  if constexpr (is_detected_v<Tanker, op_tk_current_fuel_transfer_speed>) {
    if (tk.current_status.fuel_transfer_speed > 1e-6)
      return static_cast<double>(tk.current_status.fuel_transfer_speed);
  }
  return std::nullopt;
}

template <class Ctx>
static double ExtractFuelTransferSpeedKgpsFromCtx(const Ctx& ctx) {
  // fuel_speed in your formula: tanker fuel transfer speed (kg/s).
  // If your project stores kg/min, convert it here.
  if (auto v = TryGetFuelTransferSpeedKgps(ctx.tanker)) return *v;
  return 20.0; // placeholder default (kg/s)
}

template <class Receiver>
static std::optional<double> TryGetCurrentSpeedMps(const Receiver& r) {
  if constexpr (is_detected_v<Receiver, op_rcv_current_speed>) {
    if (r.current_status.speed_mps > 1e-3) return static_cast<double>(r.current_status.speed_mps);
  }
  return std::nullopt;
}

template <class Ctx>
static double ExtractExitSpeedMpsFromCtx(const Ctx& ctx, double fallback_speed_mps) {
  double sum = 0.0;
  int cnt = 0;
  for (const auto& r : ctx.receivers) {
    if (auto s = TryGetCurrentSpeedMps(r)) {
      sum += *s;
      cnt++;
    }
  }
  if (cnt > 0) return sum / cnt;
  return fallback_speed_mps;
}

static racetrack::Vec2 ComputeReceiversCentroid(const std::vector<racetrack::Vec2>& receivers,
                                                const racetrack::Vec2& tanker) {
  if (receivers.empty()) return tanker;
  racetrack::Vec2 c{0.0, 0.0};
  for (const auto& p : receivers) {
    c.x += p.x;
    c.y += p.y;
  }
  c.x /= receivers.size();
  c.y /= receivers.size();
  return c;
}

// ---------- build candidate entrypoints on the racetrack boundary ----------
// entrypoints definition (user): "allowed candidate entry points to enter/merge into the racetrack".
static void BuildEntryPoints(const racetrack::Outputs& out,
                             double altitude_m,
                             std::vector<refuel::Vec3>& entrypoints_xy) {
  entrypoints_xy.clear();

  const double L = out.L;
  const double R = out.R;
  const double th = out.heading_deg * kPI / 180.0;

  const double c = std::cos(th);
  const double s = std::sin(th);

  // u: main axis direction (along AB)
  const double ux = c, uy = s;
  // v: left normal (perp to u)
  const double vx = -s, vy = c;

  const double cx = out.center.x;
  const double cy = out.center.y;

  auto push_unique = [&](double x, double y) {
    constexpr double EPS = 1e-3; // 1 mm
    for (const auto& q : entrypoints_xy) {
      if (std::fabs(q.x - x) < EPS && std::fabs(q.y - y) < EPS) return;
      if (std::fabs(q.y - y) < EPS && std::fabs(q.x - x) < EPS) return;
    }
    entrypoints_xy.push_back(refuel::Vec3{x, y, altitude_m});
  };

  if (R <= 0.0) return;

  if (L <= 1e-6) {
    // Circle case: sample 8 entrypoints evenly around the circle.
    for (int k = 0; k < 8; ++k) {
      const double ang = (2.0 * kPI * k) / 8.0;
      push_unique(cx + R * std::cos(ang), cy + R * std::sin(ang));
    }
    return;
  }

  // Segment endpoints on centerline
  const double ax = cx - ux * (L / 2.0);
  const double ay = cy - uy * (L / 2.0);
  const double bx = cx + ux * (L / 2.0);
  const double by = cy + uy * (L / 2.0);

  // 1) Tangency points where straights meet semicircles (common merge points)
  push_unique(ax + vx * R, ay + vy * R);
  push_unique(ax - vx * R, ay - vy * R);
  push_unique(bx + vx * R, by + vy * R);
  push_unique(bx - vx * R, by - vy * R);

  // 2) Midpoints of the two straight boundaries
  push_unique(cx + vx * R, cy + vy * R);
  push_unique(cx - vx * R, cy - vy * R);

  // 3) Quarters along both straights to offer more candidates
  const double t = 0.25 * L;
  push_unique(cx - ux * t + vx * R, cy - uy * t + vy * R);
  push_unique(cx + ux * t + vx * R, cy + uy * t + vy * R);
  push_unique(cx - ux * t - vx * R, cy - uy * t - vy * R);
  push_unique(cx + ux * t - vx * R, cy + uy * t - vy * R);

  // 4) Outer-most points on the two semicircles
  push_unique(ax - ux * R, ay - uy * R);
  push_unique(bx + ux * R, by + uy * R);
}


void refuel::RacetrackBuildStage::Run(refuel::PlanningContext& ctx) {
  ctx.racetrack = {}; // clear previous result

  // 1) altitude/speed: compile-safe extraction with fallbacks
  double altitude_m = 7000.0;
  double speed_mps  = 130.0;
  ExtractAltitudeSpeedFromCtx(ctx, altitude_m, speed_mps);

  ctx.racetrack.altitude_m = altitude_m;
  ctx.racetrack.speed_mps  = speed_mps;

  // 2) Prepare inputs for racetrack::PlanRacetrack
  racetrack::Inputs in;

  // Safe polygon (guaranteed non-empty per you)
  FillSafePolygon(ctx.safe_zone_selected.safe_zone_polygon_xy, in.safe_polygon);

  // Tanker & receivers positions
  in.tanker = ToRTVec2(ctx.tanker.initial_position_xy);
  in.receivers.reserve(ctx.receivers.size());
  for (const auto& r : ctx.receivers) {
    in.receivers.push_back(ToRTVec2(r.initial_position_xy));
  }

  // Mode preference: keep default Balanced0 for now.
  in.model_type = racetrack::ModelType::Balanced0;

  // Time (for sun bearing inside your model): use wall-clock for now
  in.unix_time_utc = static_cast<std::int64_t>(std::time(nullptr));

  // R0/L0 empirical initialization (your new formulas from the screenshot):
  //   R0 = racetrack_speed^2 / (g * tan(angle))
  //   L0 = racetrack_speed * ( 10000/(180 - racetrack_speed)
  //                            + 3*60
  //                            + max_fuel / fuel_speed
  //                            + distance_to_rendezvous / exit_speed ) * 3
  //
  // Units:
  // - racetrack_speed, exit_speed: m/s
  // - distance_to_rendezvous: m
  // - max_fuel: kg
  // - fuel_speed: kg/s
  const double g = 9.80665; // m/s^2
  const double angle_deg = ExtractRacetrackAngleDegFromMission(ctx.mission);
  const double angle_rad = angle_deg * kPI / 180.0;

  double tanv = std::tan(angle_rad);
  if (std::abs(tanv) < 1e-6) tanv = (tanv >= 0.0 ? 1e-6 : -1e-6);

  in.R0 = std::fabs(sqr(speed_mps) / (g * tanv));

  const double max_fuel_kg = ExtractMaxFuelKgFromCtx(ctx);
  const double fuel_speed_kgps = ExtractFuelTransferSpeedKgpsFromCtx(ctx);
  const double exit_speed_mps = ExtractExitSpeedMpsFromCtx(ctx, speed_mps);

  const racetrack::Vec2 recv_centroid = ComputeReceiversCentroid(in.receivers, in.tanker);
  const double distance_to_rendezvous = std::sqrt(dist2_xy(in.tanker, recv_centroid));

  double denom = 180.0 - speed_mps;
  if (std::abs(denom) < 1e-3) denom = (denom >= 0.0 ? 1e-3 : -1e-3);

  const double t1 = 10000.0 / denom;
  const double t2 = 3.0 * 60.0;
  const double t3 = (fuel_speed_kgps > 1e-6 ? (max_fuel_kg / fuel_speed_kgps) : 0.0);
  const double t4 = (exit_speed_mps > 1e-6 ? (distance_to_rendezvous / exit_speed_mps) : 0.0);

  in.L0 = speed_mps * (t1 + t2 + t3 + t4) * 3.0;

  // Guardrails: keep the stadium non-degenerate. Planner will still shrink to safe polygon.
  if (!std::isfinite(in.L0) || in.L0 < 1.0) in.L0 = 4.0 * in.R0;
  const double L_min = 2.0 * in.R0;
  if (in.L0 < L_min) in.L0 = L_min;

    // 3) Call real planner
    const racetrack::Result res = racetrack::PlanRacetrack(in);
    if (res.status != racetrack::Status::Ok) {
      // Keep racetrack cleared, but retain altitude/speed.
      ctx.racetrack.length_m = 0.0;
      ctx.racetrack.radius_m = in.R0;
      ctx.racetrack.orientation_deg = 0.0;
      ctx.racetrack.center_xy = refuel::Vec3{0.0, 0.0, altitude_m};
      ctx.racetrack.entrypoints_xy.clear();
      return;
    }

    // 4) Write outputs back to ctx.racetrack (interface unchanged)
    ctx.racetrack.length_m = res.out.L;
    ctx.racetrack.radius_m = res.out.R;
    ctx.racetrack.orientation_deg = res.out.heading_deg;
    ctx.racetrack.center_xy = refuel::Vec3{res.out.center.x, res.out.center.y, altitude_m};

    BuildEntryPoints(res.out, altitude_m, ctx.racetrack.entrypoints_xy);
}

