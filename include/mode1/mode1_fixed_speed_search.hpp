#pragma once
/**
 * @file mode1_fixed_speed_search.hpp
 *
 * 这个头文件实现（接口 + 可运行的基础实现）：
 *   “加油机定速 + 受油机定时”（mode_type = 1）的白框流程。
 *
 * 与 mode0 的差异（对齐你最新流程图）：
 *   - 不计算盘旋圈数（不需要 HoldingLoopCalculator 环节）
 *   - 对于每个 (tanker_speed, entrypoint_choice) 组合：
 *       直接解一个受油机“奔赴真速 vr”，使得受油机与加油机同时到达会合点。
 *       vr 允许为非整数（double 小数）。
 *   - 其余白框（速度离散、点位组合、会合高度反查、粗代价评估、全局选优）与 mode0 一致。
 *
 * 白框对应关系（与你最新图片一致）：
 *   [1] 离散遍历加油机的速度区间
 *   [2] 遍历受油机的点位选择（进入点位组合）
 *   [3] 计算受油机需要的“奔赴真速 vr”，使其与加油机同时到达点位，并做速度范围检查
 *   [5] 会合速度 ->（高度，表速）组合：反向查表取高度变化最小的会合高度
 *   [6] 粗计算相应的油耗与时间，找到代价最小的方案（此处决定：受油机真速 vr / 会合高度 / 会合表速）
 *   [9] 根据寻优偏好，取油耗最优 / 时间最优
 */

#include <string>
#include <vector>
#include <optional>
#include <limits>
#include <cmath>
#include <algorithm>
#include <functional>
#include <utility>

#include "common/types.hpp"

namespace refuel::mode1 {

// ============================================================
// 最小依赖工具函数（与 mode0 保持同口径）
// ============================================================

/// 2D 欧氏距离（只看 x/y，忽略 z）
static inline double Dist2D(const refuel::Vec3& a, const refuel::Vec3& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

/// 安全除法：避免除 0
static inline double SafeDiv(double num, double den, double fallback = 0.0) {
  if (std::fabs(den) < 1e-9) return fallback;
  return num / den;
}

/// 获取跑马场进入点列表。
/// - 正常：直接使用 ctx.racetrack.entrypoints_xy
/// - 若为空：围绕中心点构造 4 个简易进入点，保证流程可跑。
static inline std::vector<refuel::Vec3> GetEntryPointsFallback(const refuel::PlanningContext& ctx) {
  if (!ctx.racetrack.entrypoints_xy.empty()) {
    return ctx.racetrack.entrypoints_xy;
  }

  refuel::Vec3 c = ctx.racetrack.center_xy;
  if (std::fabs(c.x) < 1e-9 && std::fabs(c.y) < 1e-9) {
    c = ctx.tanker.initial_position_xy;
  }

  double r = ctx.racetrack.radius_m;
  if (r <= 1e-6) r = ctx.mission.racecourse.orbit_radius;
  if (r <= 1e-6) r = 5000.0;

  std::vector<refuel::Vec3> eps;
  eps.reserve(4);
  eps.push_back({c.x + r, c.y, 0.0});
  eps.push_back({c.x, c.y + r, 0.0});
  eps.push_back({c.x - r, c.y, 0.0});
  eps.push_back({c.x, c.y - r, 0.0});
  return eps;
}

/// 最近邻索引（2D）
static inline int NearestIndex2D(const std::vector<refuel::Vec3>& pts, const refuel::Vec3& p) {
  if (pts.empty()) return -1;
  int best = 0;
  double best_d = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
    const double d = Dist2D(pts[i], p);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

// ===========================
// 0) 搜索偏好（全局选优使用）
// ===========================
enum class Preference {
  kMinFuel,   ///< 油耗优先（cost = fuel）
  kMinTime    ///< 时间优先（cost = time）
};

// ===========================
// 1) 白框 1：加油机速度候选
// ===========================
struct TankerSpeedCandidates {
  std::vector<double> tanker_tas_mps;
};

// ===========================
// 2) 白框 2：进入点位组合
// ===========================
struct EntryPointChoice {
  std::string receiver_id;
  int entrypoint_idx = -1;
};

struct EntryPointCombo {
  std::vector<EntryPointChoice> choices;
};

struct EntryPointCombos {
  std::vector<EntryPointCombo> combos;
};

// ===========================
// 3) 白框 3：会合候选（可行性筛选）
// ===========================
struct MeetingCandidate {
  bool feasible = false;

  double receiver_tas_mps = 0.0;   ///< 受油机“奔赴真速 vr”（解出来的小数）
  double meet_tas_mps = 0.0;       ///< 会合速度：基础实现仍按“加油机定速=会合速度”

  double tanker_arrive_time_s = 0.0;
  double receiver_arrive_time_s = 0.0;
  double delta_t_s = 0.0;          ///< receiver - tanker（mode1 里应接近 0）

  // 为了与 mode0 的输出结构保持一致（便于 rendezvous_stage.cpp 按同一字段写回），
  // 这里保留盘旋相关字段，但 mode1 中固定为 0 / 不启用。
  int    n_loops = 0;
  double receiver_arrive_time_adjusted_s = 0.0;

  double meet_altitude_m = 0.0;
  double meet_ias_mps = 0.0;
  double altitude_change_m = 0.0;

  double total_fuel = std::numeric_limits<double>::infinity();
  double total_time = std::numeric_limits<double>::infinity();
  double cost = std::numeric_limits<double>::infinity();

  std::string infeasible_reason;
};

// ===========================
// 6) 白框 6/9 的结果封装
// ===========================
struct BestUnderTankerSpeed {
  bool feasible = false;
  double tanker_tas_mps = 0.0;
  EntryPointCombo combo;
  MeetingCandidate best;
};

struct BestGlobalResult {
  bool feasible = false;
  Preference preference = Preference::kMinFuel;
  BestUnderTankerSpeed best;
};

// ====================================================================
// 白框环节接口（每个类只做一件事）
// ====================================================================

class TankerSpeedDiscretizer {
public:
  virtual ~TankerSpeedDiscretizer() = default;

  virtual TankerSpeedCandidates Discretize(const PlanningContext& ctx) const {
    TankerSpeedCandidates out;

    const double vmin = ctx.tanker_speed_bounds.min_speed_mps;
    const double vmax = ctx.tanker_speed_bounds.max_speed_mps;
    const double vcruise = ctx.tanker_speed_bounds.cruise_speed_mps;

    if (vmax <= 1e-6 || vmin <= 1e-6 || vmax < vmin) {
      return out;
    }

    const double step = 10.0; // m/s：基础离散
    for (double v = vmin; v <= vmax + 1e-9; v += step) {
      out.tanker_tas_mps.push_back(v);
    }

    if (vcruise > vmin + 1e-9 && vcruise < vmax - 1e-9) {
      bool exists = false;
      for (double v : out.tanker_tas_mps) {
        if (std::fabs(v - vcruise) < 1e-6) { exists = true; break; }
      }
      if (!exists) out.tanker_tas_mps.push_back(vcruise);
    }

    std::sort(out.tanker_tas_mps.begin(), out.tanker_tas_mps.end());
    return out;
  }
};

class EntryPointComboGenerator {
public:
  virtual ~EntryPointComboGenerator() = default;

  virtual EntryPointCombos Generate(const PlanningContext& ctx) const {
    EntryPointCombos out;

    if (ctx.receivers.empty()) {
      return out;
    }

    const std::vector<refuel::Vec3> entrypoints = GetEntryPointsFallback(ctx);
    if (entrypoints.empty()) {
      return out;
    }

    // 每个 receiver 只保留距离最近的 topK 个进入点（防止组合爆炸）
    const size_t topK = std::min<size_t>(3, entrypoints.size());

    struct ReceiverOptions {
      std::string rid;
      std::vector<int> ep_indices;
    };
    std::vector<ReceiverOptions> all_opts;
    all_opts.reserve(ctx.receivers.size());

    for (const auto& r : ctx.receivers) {
      std::vector<std::pair<double, int>> dists;
      dists.reserve(entrypoints.size());
      for (size_t i = 0; i < entrypoints.size(); ++i) {
        dists.push_back({Dist2D(r.initial_position_xy, entrypoints[i]), static_cast<int>(i)});
      }
      std::sort(dists.begin(), dists.end(),
                [](const auto& a, const auto& b) { return a.first < b.first; });

      ReceiverOptions opt;
      opt.rid = r.id;
      for (size_t k = 0; k < topK; ++k) {
        opt.ep_indices.push_back(dists[k].second);
      }
      all_opts.push_back(opt);
    }

    EntryPointCombo current;
    current.choices.reserve(all_opts.size());

    std::function<void(size_t)> dfs = [&](size_t idx) {
      if (idx == all_opts.size()) {
        out.combos.push_back(current);
        return;
      }
      const auto& opt = all_opts[idx];
      for (int ep : opt.ep_indices) {
        current.choices.push_back({opt.rid, ep});
        dfs(idx + 1);
        current.choices.pop_back();
      }
    };

    dfs(0);
    return out;
  }
};

class FeasibleCandidateGenerator {
public:
  virtual ~FeasibleCandidateGenerator() = default;

  // mode1：不遍历离散 vr，而是直接解 vr，使得 t_receiver == t_tanker
  virtual std::vector<MeetingCandidate> Generate(
      const PlanningContext& ctx,
      double tanker_tas_mps,
      const EntryPointCombo& combo) const
  {
    std::vector<MeetingCandidate> out;

    if (combo.choices.empty()) return out;

    const std::string rid = combo.choices.front().receiver_id;
    const int ep_idx = combo.choices.front().entrypoint_idx;

    // receiver 配置
    const refuel::AircraftConfig* receiver = nullptr;
    for (const auto& r : ctx.receivers) {
      if (r.id == rid) { receiver = &r; break; }
    }
    if (!receiver) return out;

    const std::vector<refuel::Vec3> entrypoints = GetEntryPointsFallback(ctx);
    if (ep_idx < 0 || static_cast<size_t>(ep_idx) >= entrypoints.size()) return out;
    const refuel::Vec3 meet_point = entrypoints[static_cast<size_t>(ep_idx)];

    // tanker speed bounds 检查
    const double t_vmin = ctx.tanker_speed_bounds.min_speed_mps;
    const double t_vmax = ctx.tanker_speed_bounds.max_speed_mps;
    if (!(tanker_tas_mps >= t_vmin - 1e-9 && tanker_tas_mps <= t_vmax + 1e-9)) {
      return out;
    }

    // receiver speed bounds
    const auto itb = ctx.receiver_speed_bounds.find(rid);
    if (itb == ctx.receiver_speed_bounds.end()) return out;
    const double r_vmin = itb->second.min_speed_mps;
    const double r_vmax = itb->second.max_speed_mps;

    // 会合速度（基础实现按“加油机定速 = 会合速度”）
    const double meet_tas = tanker_tas_mps;
    // 仍要求：受油机具备跟随/匹配会合速度的能力
    if (meet_tas < r_vmin - 1e-9 || meet_tas > r_vmax + 1e-9) {
      return out;
    }

    // 到点时间：两点直线
    const double tanker_dist_m = Dist2D(ctx.tanker.initial_position_xy, meet_point);
    const double t_tanker_arrive = SafeDiv(tanker_dist_m, tanker_tas_mps, std::numeric_limits<double>::infinity());
    if (!std::isfinite(t_tanker_arrive) || t_tanker_arrive <= 1e-9) {
      return out;
    }

    // 解受油机“奔赴真速” vr，使得 t_receiver = t_tanker
    const double receiver_dist_m = Dist2D(receiver->initial_position_xy, meet_point);
    const double vr = SafeDiv(receiver_dist_m, t_tanker_arrive, std::numeric_limits<double>::infinity());

    MeetingCandidate c;
    c.meet_tas_mps = meet_tas;
    c.receiver_tas_mps = vr;
    c.tanker_arrive_time_s = t_tanker_arrive;
    c.receiver_arrive_time_s = t_tanker_arrive;
    c.delta_t_s = 0.0;
    c.n_loops = 0;
    c.receiver_arrive_time_adjusted_s = c.receiver_arrive_time_s;

    if (!std::isfinite(vr)) {
      c.feasible = false;
      c.infeasible_reason = "computed receiver dash speed is invalid";
      out.push_back(c);
      return out;
    }

    if (vr < r_vmin - 1e-9 || vr > r_vmax + 1e-9) {
      c.feasible = false;
      c.infeasible_reason = "computed receiver dash speed out of bounds";
      out.push_back(c);
      return out;
    }

    c.feasible = true;
    out.push_back(c);
    return out;
  }
};

class MeetAltitudeSelector {
public:
  virtual ~MeetAltitudeSelector() = default;

  // ---- (H, TAS)->fuel_rate_kgps 双线性插值；fuel_table.speed_levels 是 TAS ----
  static double FuelRateKgps_Bilinear(
      const decltype(refuel::AircraftConfig{}.fuel_table)& fuel_table,
      double alt_m,
      double tas_mps)
  {
    const auto& alts = fuel_table.altitude_levels;
    const auto& tass = fuel_table.speed_levels;       // TAS 网格
    const auto& data = fuel_table.consumption_data;   // alts.size x tass.size

    if (alts.empty() || tass.empty() || data.empty()) return std::numeric_limits<double>::infinity();
    if (data.size() != alts.size()) return std::numeric_limits<double>::infinity();
    for (const auto& row : data) {
      if (row.size() != tass.size()) return std::numeric_limits<double>::infinity();
    }

    auto bracket = [](const std::vector<double>& grid, double q, size_t* i0, size_t* i1, double* t) {
      const size_t n = grid.size();
      if (n == 1) { *i0 = *i1 = 0; *t = 0.0; return; }

      if (q <= grid.front()) { *i0 = 0; *i1 = 1; *t = 0.0; return; }
      if (q >= grid.back())  { *i0 = n - 2; *i1 = n - 1; *t = 1.0; return; }

      size_t lo = 0, hi = n - 1;
      while (hi - lo > 1) {
        const size_t mid = (lo + hi) / 2;
        if (grid[mid] <= q) lo = mid;
        else hi = mid;
      }
      *i0 = lo;
      *i1 = lo + 1;
      const double g0 = grid[*i0], g1 = grid[*i1];
      const double den = g1 - g0;
      *t = (std::fabs(den) < 1e-12) ? 0.0 : (q - g0) / den;
      if (*t < 0.0) *t = 0.0;
      if (*t > 1.0) *t = 1.0;
    };

    size_t a0 = 0, a1 = 0, v0 = 0, v1 = 0;
    double ta = 0.0, tv = 0.0;
    bracket(alts, alt_m, &a0, &a1, &ta);
    bracket(tass, tas_mps, &v0, &v1, &tv);

    const double f00 = data[a0][v0];
    const double f01 = data[a0][v1];
    const double f10 = data[a1][v0];
    const double f11 = data[a1][v1];

    const double f0 = f00 + (f01 - f00) * tv;
    const double f1 = f10 + (f11 - f10) * tv;
    return f0 + (f1 - f0) * ta;
  }

  // ---- 在固定高度行上：由 TAS 反插值求 IAS（perf.speed_levels 是 IAS，true_airspeed_data 是 TAS）----
  static bool InverseIASAtAltByTAS(
      const decltype(refuel::AircraftConfig{}.cruise_perf)& perf,
      int alt_idx,
      double target_tas,
      double* out_ias)
  {
    if (!out_ias) return false;
    if (alt_idx < 0 || alt_idx >= (int)perf.altitude_levels.size()) return false;
    if (alt_idx < 0 || alt_idx >= (int)perf.true_airspeed_data.size()) return false;

    const auto& ias_grid = perf.speed_levels;                // IAS
    const auto& tas_row  = perf.true_airspeed_data[alt_idx]; // TAS

    if (ias_grid.size() < 2 || tas_row.size() != ias_grid.size()) return false;

    if (target_tas < tas_row.front() - 1e-9 || target_tas > tas_row.back() + 1e-9) {
      return false;
    }

    for (size_t j = 0; j < tas_row.size(); ++j) {
      if (std::fabs(tas_row[j] - target_tas) < 1e-9) {
        *out_ias = ias_grid[j];
        return true;
      }
    }

    size_t j = 0;
    while (j + 1 < tas_row.size() && !(tas_row[j] <= target_tas && target_tas <= tas_row[j + 1])) {
      ++j;
    }
    if (j + 1 >= tas_row.size()) return false;

    const double tas0 = tas_row[j], tas1 = tas_row[j + 1];
    const double ias0 = ias_grid[j], ias1 = ias_grid[j + 1];
    const double den  = tas1 - tas0;
    if (std::fabs(den) < 1e-12) {
      *out_ias = ias0;
      return true;
    }
    const double t = (target_tas - tas0) / den;
    *out_ias = ias0 + t * (ias1 - ias0);
    return true;
  }

  virtual void Apply(std::vector<MeetingCandidate>& cands,
                     const PlanningContext& ctx,
                     const std::string& receiver_id) const
  {
    const refuel::AircraftConfig* r = nullptr;
    for (const auto& rr : ctx.receivers) {
      if (rr.id == receiver_id) { r = &rr; break; }
    }
    if (!r) return;

    double h_ref = r->initial_position_lla.alt_m;
    if (std::fabs(h_ref) < 1e-6 && ctx.racetrack.altitude_m > 1e-6) {
      h_ref = ctx.racetrack.altitude_m;
    }

    const auto& perf = r->cruise_perf;
    const bool has_perf = (!perf.altitude_levels.empty() &&
                           !perf.speed_levels.empty() &&
                           !perf.true_airspeed_data.empty());

    for (auto& c : cands) {
      if (!c.feasible) continue;

      const double target_tas = c.meet_tas_mps;

      if (!has_perf) {
        c.meet_altitude_m = h_ref;
        c.meet_ias_mps = target_tas; // 兜底：IAS≈TAS
        c.altitude_change_m = 0.0;
        continue;
      }

      bool found = false;
      double best_alt = 0.0;
      double best_ias = 0.0;
      double best_alt_change = std::numeric_limits<double>::infinity();
      double best_fuel_rate = std::numeric_limits<double>::infinity();

      for (int i = 0; i < (int)perf.altitude_levels.size(); ++i) {
        if (i >= (int)perf.true_airspeed_data.size()) break;
        if (perf.true_airspeed_data[i].size() != perf.speed_levels.size()) continue;

        double ias = 0.0;
        if (!InverseIASAtAltByTAS(perf, i, target_tas, &ias)) {
          continue;
        }

        const double alt = perf.altitude_levels[i];
        const double alt_change = std::fabs(alt - h_ref);
        const double fuel_rate = FuelRateKgps_Bilinear(r->fuel_table, alt, target_tas);

        if (!found ||
            alt_change < best_alt_change - 1e-9 ||
            (std::fabs(alt_change - best_alt_change) < 1e-9 && fuel_rate < best_fuel_rate - 1e-12)) {
          found = true;
          best_alt = alt;
          best_ias = ias;
          best_alt_change = alt_change;
          best_fuel_rate = fuel_rate;
        }
      }

      if (!found) {
        c.feasible = false;
        c.infeasible_reason = "meet_tas out of cruise table (no feasible altitude/IAS)";
        continue;
      }

      c.meet_altitude_m = best_alt;
      c.meet_ias_mps = best_ias;
      c.altitude_change_m = best_alt_change;
    }
  }
};

class CoarseCostEvaluator {
public:
  virtual ~CoarseCostEvaluator() = default;

  // (H, TAS) -> fuel_rate_kgps  双线性插值（fuel_table.speed_levels 是 TAS 网格）
  static double LookupFuelRateKgps_Bilinear(
      const decltype(refuel::AircraftConfig{}.fuel_table)& fuel_table,
      double alt_m,
      double tas_mps)
  {
    const auto& alts = fuel_table.altitude_levels;
    const auto& tass = fuel_table.speed_levels;
    const auto& data = fuel_table.consumption_data;

    if (alts.empty() || tass.empty() || data.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    if (data.size() != alts.size()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    for (const auto& row : data) {
      if (row.size() != tass.size()) {
        return std::numeric_limits<double>::quiet_NaN();
      }
    }

    auto bracket = [](const std::vector<double>& grid, double q, size_t* i0, size_t* i1, double* t) {
      const size_t n = grid.size();
      if (n == 1) {
        *i0 = *i1 = 0;
        *t = 0.0;
        return;
      }

      if (q <= grid.front()) { *i0 = 0; *i1 = 1; *t = 0.0; return; }
      if (q >= grid.back())  { *i0 = n - 2; *i1 = n - 1; *t = 1.0; return; }

      size_t lo = 0, hi = n - 1;
      while (hi - lo > 1) {
        const size_t mid = (lo + hi) / 2;
        if (grid[mid] <= q) lo = mid;
        else hi = mid;
      }
      *i0 = lo;
      *i1 = lo + 1;

      const double g0 = grid[*i0];
      const double g1 = grid[*i1];
      const double den = g1 - g0;
      *t = (std::fabs(den) < 1e-12) ? 0.0 : (q - g0) / den;
      if (*t < 0.0) *t = 0.0;
      if (*t > 1.0) *t = 1.0;
    };

    size_t a0 = 0, a1 = 0, v0 = 0, v1 = 0;
    double ta = 0.0, tv = 0.0;
    bracket(alts, alt_m, &a0, &a1, &ta);
    bracket(tass, tas_mps, &v0, &v1, &tv);

    const double f00 = data[a0][v0];
    const double f01 = data[a0][v1];
    const double f10 = data[a1][v0];
    const double f11 = data[a1][v1];

    const double f0 = f00 + (f01 - f00) * tv;
    const double f1 = f10 + (f11 - f10) * tv;
    return f0 + (f1 - f0) * ta;
  }

  virtual std::optional<MeetingCandidate> PickBest(
      std::vector<MeetingCandidate>& cands,
      const PlanningContext& ctx,
      Preference pref) const
  {
    double best_cost = std::numeric_limits<double>::infinity();
    std::optional<MeetingCandidate> best;

    const refuel::AircraftConfig* receiver = nullptr;
    if (!ctx.receivers.empty()) receiver = &ctx.receivers.front();

    for (auto& c : cands) {
      if (!c.feasible) continue;

      // 时间：mode1 要求同时到达；这里仍按 max(两者最终到达时间) 计算，保持稳健。
      if (c.receiver_arrive_time_adjusted_s <= 1e-9) {
        c.receiver_arrive_time_adjusted_s = c.receiver_arrive_time_s;
      }
      const double meeting_time_s = std::max(c.tanker_arrive_time_s, c.receiver_arrive_time_adjusted_s);
      c.total_time = meeting_time_s;

      // tanker fuel：按“到达点位耗时”计油耗（不额外计等待）
      double tanker_rate = LookupFuelRateKgps_Bilinear(ctx.tanker.fuel_table, c.meet_altitude_m, c.meet_tas_mps);
      if (!std::isfinite(tanker_rate) || tanker_rate <= 1e-12) {
        tanker_rate = 2.0; // 兜底
      }
      const double tanker_fuel = tanker_rate * c.tanker_arrive_time_s;

      // receiver fuel：按奔赴真速 vr 计油耗
      double receiver_rate = 1.0;
      if (receiver) {
        const double tas_for_fuel = (c.receiver_tas_mps > 1e-6) ? c.receiver_tas_mps : c.meet_tas_mps;
        receiver_rate = LookupFuelRateKgps_Bilinear(receiver->fuel_table, c.meet_altitude_m, tas_for_fuel);
        if (!std::isfinite(receiver_rate) || receiver_rate <= 1e-12) {
          receiver_rate = 1.0;
        }
      }
      const double receiver_fuel = receiver_rate * c.receiver_arrive_time_adjusted_s;

      c.total_fuel = tanker_fuel + receiver_fuel;
      c.cost = (pref == Preference::kMinFuel) ? c.total_fuel : c.total_time;

      if (c.cost < best_cost - 1e-12) {
        best_cost = c.cost;
        best = c;
      } else if (std::fabs(c.cost - best_cost) < 1e-12 && best.has_value()) {
        if (pref == Preference::kMinFuel) {
          if (c.total_time < best->total_time) best = c;
        } else {
          if (c.total_fuel < best->total_fuel) best = c;
        }
      }
    }

    return best;
  }
};

class GlobalSelector {
public:
  virtual ~GlobalSelector() = default;

  virtual BestGlobalResult Select(const std::vector<BestUnderTankerSpeed>& all,
                                  Preference pref) const
  {
    BestGlobalResult g;
    g.preference = pref;

    double best_cost = std::numeric_limits<double>::infinity();
    for (const auto& b : all) {
      if (!b.feasible) continue;
      if (b.best.cost < best_cost) {
        best_cost = b.best.cost;
        g.best = b;
        g.feasible = true;
      }
    }
    return g;
  }
};

// ====================================================================
// 组合器：把白框 1/2/3/5/6/9 串起来
// ====================================================================

class Mode1FixedSpeedSearcher {
public:
  struct Dependencies {
    TankerSpeedDiscretizer* tanker_speed_discretizer = nullptr;
    EntryPointComboGenerator* combo_generator = nullptr;
    FeasibleCandidateGenerator* candidate_generator = nullptr;
    MeetAltitudeSelector* altitude_selector = nullptr;
    CoarseCostEvaluator* cost_evaluator = nullptr;
    GlobalSelector* global_selector = nullptr;
  };

  explicit Mode1FixedSpeedSearcher(Dependencies deps) : deps_(deps) {}

  BestGlobalResult Solve(const PlanningContext& ctx, Preference pref) const {
    if (!deps_.tanker_speed_discretizer || !deps_.combo_generator || !deps_.candidate_generator ||
        !deps_.altitude_selector || !deps_.cost_evaluator || !deps_.global_selector) {
      return {};
    }

    const TankerSpeedCandidates vt_cands = deps_.tanker_speed_discretizer->Discretize(ctx);
    const EntryPointCombos combos = deps_.combo_generator->Generate(ctx);

    std::vector<BestUnderTankerSpeed> best_under_all_vt;

    for (double vt : vt_cands.tanker_tas_mps) {
      BestUnderTankerSpeed best_vt;
      best_vt.tanker_tas_mps = vt;
      best_vt.best.cost = std::numeric_limits<double>::infinity();

      for (const auto& combo : combos.combos) {
        std::vector<MeetingCandidate> cands = deps_.candidate_generator->Generate(ctx, vt, combo);
        if (cands.empty()) {
          continue;
        }

        if (!combo.choices.empty()) {
          deps_.altitude_selector->Apply(cands, ctx, combo.choices.front().receiver_id);
        }

        std::optional<MeetingCandidate> best_cand = deps_.cost_evaluator->PickBest(cands, ctx, pref);
        if (!best_cand.has_value()) {
          continue;
        }

        if (best_cand->cost < best_vt.best.cost) {
          best_vt.feasible = true;
          best_vt.combo = combo;
          best_vt.best = *best_cand;
        }
      }

      if (best_vt.feasible) {
        best_under_all_vt.push_back(best_vt);
      }
    }

    return deps_.global_selector->Select(best_under_all_vt, pref);
  }

private:
  Dependencies deps_;
};

} // namespace refuel::mode1
