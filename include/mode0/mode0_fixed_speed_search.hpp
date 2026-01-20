#pragma once
/**
 * @file mode0_fixed_speed_search.hpp
 *
 * 这个头文件实现（仅接口，不含算法实现）：
 *   “加油机定速 + 受油机定速”（mode_type = 0）的白框流程。
 *
 * 重要约定：白色方框 = 环节（接口边界），绿色框只是说明/提示，不作为环节边界。
 *
 * 白框对应关系（与你最新图片一致）：
 *   [1] 离散遍历加油机的速度区间
 *   [2] 遍历受油机的点位选择（进入点位组合）
 *   [3] 计算会合所需会合速度，并判断是否在速度范围内（输出“可行候选集”，不在此处选最优）
 *   [4] 计算盘旋的圈数（把时间差 delta_t 转为 n_loops）
 *   [5] 会合速度 ->（高度，表速）组合：反向查表取高度变化最小的会合高度
 *   [6] 粗计算相应的油耗与时间，找到代价最小的方案（此处才“决定受油机真速/高度/表速”）
 *   [9] 根据寻优偏好，取油耗最优 / 时间最优
 *
 * 你可以把具体算法实现逐步填到每个 class 的 TODO 中；
 * 只要保持这些接口不变，你的上层 Pipeline/RendezvousStage 就不需要改。
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

namespace refuel::mode0 {

// ============================================================
// 一些“最小依赖”的小工具函数（仅用于把 TODO 补全跑通）
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
/// - 正常情况：直接使用 ctx.racetrack.entrypoints_xy
/// - 若还没计算出来（为空）：在中心点周围按半径构造 4 个“简易进入点”，用于让流程跑通。
///   注意：这只是最小可运行实现；真实工程应该由 RacetrackBuildStage 输出 entrypoints。
static inline std::vector<refuel::Vec3> GetEntryPointsFallback(const refuel::PlanningContext& ctx) {
  if (!ctx.racetrack.entrypoints_xy.empty()) {
    return ctx.racetrack.entrypoints_xy;
  }

  // 选择一个“中心点”：优先 racetrack.center_xy，其次 tanker 初始位置。
  refuel::Vec3 c = ctx.racetrack.center_xy;
  if (std::fabs(c.x) < 1e-9 && std::fabs(c.y) < 1e-9) {
    c = ctx.tanker.initial_position_xy;
  }

  // 选择半径：优先 racetrack.radius_m，其次 mission 输入 orbit_radius。
  double r = ctx.racetrack.radius_m;
  if (r <= 1e-6) r = ctx.mission.racecourse.orbit_radius;
  if (r <= 1e-6) r = 5000.0; // 最后的兜底

  // 在 0/90/180/270 度方向各放一个点
  std::vector<refuel::Vec3> eps;
  eps.reserve(4);
  eps.push_back({c.x + r, c.y, 0.0});
  eps.push_back({c.x, c.y + r, 0.0});
  eps.push_back({c.x - r, c.y, 0.0});
  eps.push_back({c.x, c.y - r, 0.0});
  return eps;
}

/// 简易计算跑马场“一圈”长度（m）：直线段长度未知时，退化为圆。
static inline double RacetrackLoopLengthM(const refuel::PlanningContext& ctx) {
  const double r = (ctx.racetrack.radius_m > 1e-6) ? ctx.racetrack.radius_m
                                                   : (ctx.mission.racecourse.orbit_radius > 1e-6
                                                          ? ctx.mission.racecourse.orbit_radius
                                                          : 5000.0);
  const double L = (ctx.racetrack.length_m > 1e-6) ? ctx.racetrack.length_m : 0.0;
  // 直线两段 + 两个半圆
  constexpr double kPi = 3.14159265358979323846;
  return 2.0 * L + 2.0 * kPi * r;
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
  std::vector<double> tanker_tas_mps; ///< V_tanker_candidates（你也可以用 IAS，但要统一单位/定义）
};

// ===========================
// 2) 白框 2：进入点位组合
// ===========================
struct EntryPointChoice {
  std::string receiver_id; ///< 受油机 ID（例如 "RE001"）
  int entrypoint_idx = -1; ///< 选择 racetrack.entrypoints_xy 的哪个索引
};

/// 一个组合 = “所有受油机各自选了哪个进入点”的一次联合选择
struct EntryPointCombo {
  std::vector<EntryPointChoice> choices;
};

struct EntryPointCombos {
  std::vector<EntryPointCombo> combos;
};

// ===========================
// 3) 白框 3：会合候选（可行性筛选）
// ===========================
/**
 * MeetingCandidate 表示：在固定 (V_T, combo) 下，
 * 选定某个“受油机候选真速 V_R”后，是否能在点位实现会合/时间对齐的一个候选方案。
 *
 * 注意：这里还不是最终最优！白框 6 还会对多个 candidate 粗算代价并取最小。
 */
struct MeetingCandidate {
  bool feasible = false;

  // --- 决策变量之一：受油机候选真速（白框 6 需要在这里面选最优，所以必须保留） ---
  double receiver_tas_mps = 0.0;  ///< V_R（离散候选之一）

  // --- 会合真速（如你们定义会合时两机速度一致，则通常 meet_tas 与 tanker_tas/receiver_tas有关） ---
  double meet_tas_mps = 0.0;      ///< V_meet

  // --- 时间对齐相关（用于白框 4 计算盘旋圈数） ---
  double tanker_arrive_time_s = 0.0;    ///< t_tanker_arrive
  double receiver_arrive_time_s = 0.0;  ///< t_receiver_arrive(V_R)
  double delta_t_s = 0.0;              ///< receiver - tanker（>0 表示受油机更晚）

  // --- 白框 4 输出 ---
  int    n_loops = 0;                   ///< 盘旋圈数（最小满足“受油机不早到”）
  double receiver_arrive_time_adjusted_s = 0.0; ///< 盘旋后等效到达时间

  // --- 白框 5 输出 ---
  double meet_altitude_m = 0.0;  ///< H_meet
  double meet_ias_mps = 0.0;     ///< IAS_meet（表速；单位你可定义为 m/s）
  double altitude_change_m = 0.0;///< |H_meet - H_init|（越小越好）

  // --- 白框 6 输出（粗代价） ---
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
  EntryPointCombo combo;     ///< 该 vt 下最优 combo
  MeetingCandidate best;     ///< 该 vt 下最优 candidate（确定了 V_R/H/IAS/n_loops 等）
};

struct BestGlobalResult {
  bool feasible = false;
  Preference preference = Preference::kMinFuel;
  BestUnderTankerSpeed best;
};

// ====================================================================
// 白框环节接口（只定义接口，具体算法全部 TODO；每个类只做一件事）
// ====================================================================

/**
 * 白框 1：离散遍历加油机的速度区间
 * - 输入：ctx.tanker_speed_bounds（或你们自己的速度区间/集合定义）
 * - 输出：一组候选速度点
 */
class TankerSpeedDiscretizer {
public:
  virtual ~TankerSpeedDiscretizer() = default;

  /// 说明：标记为 virtual 便于在 tests 中写“假实现/桩函数”，从而单测每个白框环节或整流程。
  virtual TankerSpeedCandidates Discretize(const PlanningContext& ctx) const {
    // 最小实现目标：
    // - 把 ctx.tanker_speed_bounds 的 [min,max] 离散成若干速度点
    // - 若 bounds 还没准备好（都是 0），返回空（保持“空 ctx 不可行”，不破坏单测）
    TankerSpeedCandidates out;

    const double vmin = ctx.tanker_speed_bounds.min_speed_mps;
    const double vmax = ctx.tanker_speed_bounds.max_speed_mps;
    const double vcruise = ctx.tanker_speed_bounds.cruise_speed_mps;

    if (vmax <= 1e-6 || vmin <= 1e-6 || vmax < vmin) {
      return out; // 返回空
    }

    // 步长：越简单越好。你后续可换成“来自性能表网格”的离散。
    const double step = 10.0; // m/s

    for (double v = vmin; v <= vmax + 1e-9; v += step) {
      out.tanker_tas_mps.push_back(v);
    }

    // 把 cruise 速度也加入候选（如果不在列表中）
    if (vcruise > vmin + 1e-9 && vcruise < vmax - 1e-9) {
      bool exists = false;
      for (double v : out.tanker_tas_mps) {
        if (std::fabs(v - vcruise) < 1e-6) { exists = true; break; }
      }
      if (!exists) out.tanker_tas_mps.push_back(vcruise);
    }

    // 小到大排序，保证遍历行为稳定
    std::sort(out.tanker_tas_mps.begin(), out.tanker_tas_mps.end());
    return out;
  }
};

/**
 * 白框 2：遍历受油机点位选择（进入点组合）
 * - 输入：ctx.racetrack.entrypoints_xy + ctx.receivers（数量/ID）
 * - 输出：EntryPointCombos（每个 combo 对应一次联合选择）
 *
 * 注意：组合数可能指数爆炸。工程上通常会做剪枝：
 *   - 每个 receiver 只保留距离最近的 topK 个 entrypoint
 *   - 或按照优先级/扇区过滤 entrypoint
 * 这些剪枝都应写在 TODO 实现里，但接口不变。
 */
class EntryPointComboGenerator {
public:
  virtual ~EntryPointComboGenerator() = default;

  /// 同上：virtual 便于在单元测试里注入假实现。
  virtual EntryPointCombos Generate(const PlanningContext& ctx) const {
    EntryPointCombos out;

    // 没有受油机时，无法构造“组合”。
    if (ctx.receivers.empty()) {
      return out;
    }

    // 获取进入点列表（若 ctx.racetrack.entrypoints 还没填，会自动构造 4 个简易点）。
    const std::vector<refuel::Vec3> entrypoints = GetEntryPointsFallback(ctx);
    if (entrypoints.empty()) {
      return out;
    }

    // --- 1) 为每个 receiver 选一组“可选进入点” ---
    // 为避免组合指数爆炸：每个 receiver 只保留最近的 topK 个进入点。
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

    // --- 2) 对所有 receiver 的 options 做笛卡尔积，生成 combos ---
    // 递归构造：每深入一层，就给一个 receiver 选一个 entrypoint。
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

/**
 * 白框 3：计算会合所需会合速度，并判断是否在速度范围内
 * - 输入：固定的 tanker_tas + 一个 combo + 受油机速度候选集合
 * - 输出：一组“可行 MeetingCandidate”
 *
 * 关键：这里不做“最优选择”，只负责生成候选并做可行性判断。
 * 受油机真速的最终选择放到白框 6（由 cost 决定）。
 */
class FeasibleCandidateGenerator {
public:
  virtual ~FeasibleCandidateGenerator() = default;

  /// virtual：便于单测时构造“可控的候选集”。
  virtual std::vector<MeetingCandidate> Generate(
      const PlanningContext& ctx,
      double tanker_tas_mps,
      const EntryPointCombo& combo,
      const std::vector<double>& receiver_tas_candidates_mps) const
  {
    std::vector<MeetingCandidate> out;

    // 需要至少一个 receiver 的选择；若 combo 为空，直接不可行。
    if (combo.choices.empty()) {
      return out;
    }
    // 需要 receiver 速度候选集合。
    if (receiver_tas_candidates_mps.empty()) {
      return out;
    }

    // 本 skeleton 的 MeetingCandidate 结构是“单 receiver”的。
    // 因此我们用 combo.choices.front() 作为“当前评估的 receiver”。
    const std::string rid = combo.choices.front().receiver_id;
    const int ep_idx = combo.choices.front().entrypoint_idx;

    // 找到 receiver 配置。
    const refuel::AircraftConfig* receiver = nullptr;
    for (const auto& r : ctx.receivers) {
      if (r.id == rid) { receiver = &r; break; }
    }
    if (!receiver) {
      return out;
    }

    // 获取进入点坐标（可能是 fallback 生成的）。
    const std::vector<refuel::Vec3> entrypoints = GetEntryPointsFallback(ctx);
    if (ep_idx < 0 || static_cast<size_t>(ep_idx) >= entrypoints.size()) {
      return out;
    }
    const refuel::Vec3 meet_point = entrypoints[static_cast<size_t>(ep_idx)];

    // --- 1) tanker 到达时间 ---
    const double tanker_dist_m = Dist2D(ctx.tanker.initial_position_xy, meet_point);
    const double t_tanker_arrive = SafeDiv(tanker_dist_m, tanker_tas_mps, std::numeric_limits<double>::infinity());

    // --- 2) 速度范围判定需要用到 bounds（若 bounds 不存在，则按“有界但未知”处理：直接判不可行） ---
    const double t_vmin = ctx.tanker_speed_bounds.min_speed_mps;
    const double t_vmax = ctx.tanker_speed_bounds.max_speed_mps;
    if (!(tanker_tas_mps >= t_vmin - 1e-9 && tanker_tas_mps <= t_vmax + 1e-9)) {
      return out;
    }

    const auto itb = ctx.receiver_speed_bounds.find(rid);
    if (itb == ctx.receiver_speed_bounds.end()) {
      return out;
    }
    const double r_vmin = itb->second.min_speed_mps;
    const double r_vmax = itb->second.max_speed_mps;

    // --- 3) 遍历 receiver 候选速度：生成可行候选 ---
    out.reserve(receiver_tas_candidates_mps.size());
    for (double vr : receiver_tas_candidates_mps) {
      MeetingCandidate c;
      c.receiver_tas_mps = vr;

      // 会合速度最简单的定义：加油机速度=会合速度（加油机定速）
      c.meet_tas_mps = tanker_tas_mps;

      // receiver 到达时间
      const double receiver_dist_m = Dist2D(receiver->initial_position_xy, meet_point);
      const double t_receiver_arrive = SafeDiv(receiver_dist_m, vr, std::numeric_limits<double>::infinity());

      c.tanker_arrive_time_s = t_tanker_arrive;
      c.receiver_arrive_time_s = t_receiver_arrive;
      c.delta_t_s = c.receiver_arrive_time_s - c.tanker_arrive_time_s;

      // --- 可行性判定（速度是否在范围内） ---
      if (vr < r_vmin - 1e-9 || vr > r_vmax + 1e-9) {
        c.feasible = false;
        c.infeasible_reason = "receiver speed out of bounds";
        continue;
      }
      if (!(std::isfinite(t_receiver_arrive) && std::isfinite(t_tanker_arrive))) {
        c.feasible = false;
        c.infeasible_reason = "invalid arrival time";
        continue;
      }

      c.feasible = true;
      out.push_back(c);
    }

    return out;
  }
};

/**
 * 白框 4：计算盘旋圈数
 * - 输入：candidates（含 delta_t） + loop_period（盘旋一圈耗时）
 * - 输出：在 candidates 上原地写回 n_loops / adjusted_time
 *
 * 你图片里的定义：取最小圈数，使受油机“正好或晚于”加油机到达点位。
 */
class HoldingLoopCalculator {
public:
  virtual ~HoldingLoopCalculator() = default;

  /// virtual：便于单测时验证“圈数计算”的边界条件。
  virtual void Apply(std::vector<MeetingCandidate>& cands,
             double loop_period_s) const
  {
    // 说明：
    // - 这里严格按你图里的定义：取最小圈数，使受油机“正好或晚于”加油机到达。
    // - loop_period_s 是“一圈盘旋耗时（秒）”。如果传入值异常，则退化为 10s。
    if (loop_period_s <= 1e-6) loop_period_s = 10.0;

    for (auto& c : cands) {
      if (!c.feasible) continue;

      if (c.delta_t_s >= 0.0) {
        // 受油机本来就不早到：无需盘旋
        c.n_loops = 0;
        c.receiver_arrive_time_adjusted_s = c.receiver_arrive_time_s;
      } else {
        // 受油机早到：补齐等待圈数
        c.n_loops = static_cast<int>(std::ceil((-c.delta_t_s) / loop_period_s));
        if (c.n_loops < 0) c.n_loops = 0;
        c.receiver_arrive_time_adjusted_s = c.receiver_arrive_time_s + c.n_loops * loop_period_s;
      }
    }
  }
};

/**
 * 白框 5：反查表，取高度变化最小的会合高度
 * - 输入：candidates（含 meet_tas） + 性能表（ctx.receiver[i].cruise_perf 等）
 * - 输出：在 candidates 上原地写回 meet_altitude/meet_ias/altitude_change
 *
 * 你图片里明确：选“高度变化最小”的会合高度。
 */
class MeetAltitudeSelector {
public:
  virtual ~MeetAltitudeSelector() = default;

  /// virtual：便于单测时替换成“固定返回某高度/表速”的假实现。
  virtual void Apply(std::vector<MeetingCandidate>& cands,
             const PlanningContext& ctx,
             const std::string& receiver_id) const
  {
    // 最小实现目标：
    // - 不做复杂反查表，先用“初始高度”作为会合高度（高度变化=0）
    // - IAS 先近似为 TAS（真实工程可从性能表插值 TAS<->IAS）

    const refuel::AircraftConfig* r = nullptr;
    for (const auto& rr : ctx.receivers) {
      if (rr.id == receiver_id) { r = &rr; break; }
    }
    // 找不到 receiver：不改任何候选。
    if (!r) return;

    // 初始高度：优先使用输入 LLA.alt_m；若为 0，再退回跑马场高度。
    double h_init = r->initial_position_lla.alt_m;
    if (std::fabs(h_init) < 1e-6 && ctx.racetrack.altitude_m > 1e-6) {
      h_init = ctx.racetrack.altitude_m;
    }

    for (auto& c : cands) {
      if (!c.feasible) continue;
      c.meet_altitude_m = h_init;
      c.meet_ias_mps = c.meet_tas_mps; // 简化：IAS≈TAS
      c.altitude_change_m = 0.0;
    }
  }
};

/**
 * 白框 6：粗计算油耗与时间，找到代价最小的方案（受油机真速/高度/表速也在此确定）
 * - 输入：候选集 candidates（已经经过白框 3/4/5 处理，字段更完整）
 * - 输出：返回代价最小的 candidate（决定最终 receiver_tas / meet_altitude / meet_ias 等）
 */
class CoarseCostEvaluator {
public:
  virtual ~CoarseCostEvaluator() = default;

  /// virtual：便于单测整流程的“选优逻辑”。
  virtual std::optional<MeetingCandidate> PickBest(
      std::vector<MeetingCandidate>& cands,
      const PlanningContext& ctx,
      Preference pref) const
  {
    // 最小实现目标：
    // - 不做复杂航段拆分/插值
    // - 直接利用白框 3/4 已经算出的到达时间（tanker_arrive_time_s / receiver_arrive_time_adjusted_s）
    // - 用一个常数“耗油率(kg/s)”粗估 total_fuel
    // - 代价 cost 按偏好取 fuel 或 time

    double best_cost = std::numeric_limits<double>::infinity();
    std::optional<MeetingCandidate> best;

    // 兜底耗油率（真实工程请改成“从 fuel_table 插值”）
    const double tanker_burn_kgps = 2.0;   // kg/s
    const double receiver_burn_kgps = 1.0; // kg/s

    for (auto& c : cands) {
      if (!c.feasible) continue;

      // 受油机的“等效到达时间”必须由白框 4 写回；若还没写，退化为原到达时间。
      if (c.receiver_arrive_time_adjusted_s <= 1e-9) {
        c.receiver_arrive_time_adjusted_s = c.receiver_arrive_time_s;
      }

      // 会合发生在两者都到达之后。由于白框 4 确保“受油机不早到”，这里取 max 更稳妥。
      const double meeting_time_s = std::max(c.tanker_arrive_time_s, c.receiver_arrive_time_adjusted_s);

      // 这里把总时间定义为“完成会合所需的时间”。
      c.total_time = meeting_time_s;

      // 粗油耗：按各自飞行/等待时间乘以常数耗油率
      const double tanker_fuel = tanker_burn_kgps * c.tanker_arrive_time_s;
      const double receiver_fuel = receiver_burn_kgps * c.receiver_arrive_time_adjusted_s;
      c.total_fuel = tanker_fuel + receiver_fuel;

      c.cost = (pref == Preference::kMinFuel) ? c.total_fuel : c.total_time;

      if (c.cost < best_cost) {
        best_cost = c.cost;
        best = c;
      }
    }
    return best;
  }
};

/**
 * 白框 9：全局选优（在所有加油机速度 vt 下，选最终最优）
 * - 输入：每个 vt 对应的 best（白框 6 输出）
 * - 输出：BestGlobalResult
 */
class GlobalSelector {
public:
  virtual ~GlobalSelector() = default;

  /// virtual：便于在单测中替换全局选优策略（例如多目标/权重）。
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
// 组合器：把白框 1/2/3/4/5/6/9 串起来（仍然只提供“框架+注释”）
// ====================================================================

class Mode0FixedSpeedSearcher {
public:
  struct Dependencies {
    TankerSpeedDiscretizer* tanker_speed_discretizer = nullptr; ///< 白框 1
    EntryPointComboGenerator* combo_generator = nullptr;        ///< 白框 2
    FeasibleCandidateGenerator* candidate_generator = nullptr;  ///< 白框 3
    HoldingLoopCalculator* loop_calculator = nullptr;           ///< 白框 4
    MeetAltitudeSelector* altitude_selector = nullptr;          ///< 白框 5
    CoarseCostEvaluator* cost_evaluator = nullptr;              ///< 白框 6
    GlobalSelector* global_selector = nullptr;                  ///< 白框 9
  };

  explicit Mode0FixedSpeedSearcher(Dependencies deps) : deps_(deps) {}

  BestGlobalResult Solve(const PlanningContext& ctx,
                         Preference pref,
                         const std::vector<double>& receiver_tas_candidates_mps) const
  {
    // --- 参数/依赖检查（工程里最好用 assert 或抛异常，这里只做最轻量检查） ---
    if (!deps_.tanker_speed_discretizer || !deps_.combo_generator || !deps_.candidate_generator ||
        !deps_.loop_calculator || !deps_.altitude_selector || !deps_.cost_evaluator || !deps_.global_selector) {
      return {}; // 依赖未注入，直接返回 infeasible
    }

    // 白框 1：加油机速度离散
    const TankerSpeedCandidates vt_cands = deps_.tanker_speed_discretizer->Discretize(ctx);

    // 白框 2：进入点组合生成（遍历用）
    const EntryPointCombos combos = deps_.combo_generator->Generate(ctx);

    std::vector<BestUnderTankerSpeed> best_under_all_vt;

    // ========== 外层循环 L1：遍历 vt ==========
    for (double vt : vt_cands.tanker_tas_mps) {
      BestUnderTankerSpeed best_vt;
      best_vt.tanker_tas_mps = vt;
      best_vt.best.cost = std::numeric_limits<double>::infinity();

      // ========== 内层循环 L2：遍历 combo ==========
      for (const auto& combo : combos.combos) {
        // 白框 3：生成可行候选集（不选最优）
        std::vector<MeetingCandidate> cands =
            deps_.candidate_generator->Generate(ctx, vt, combo, receiver_tas_candidates_mps);
        if (cands.empty()) {
          // 这个 combo 完全不可行，直接换下一个 combo
          continue;
        }

        // 白框 4：计算盘旋圈数
        // 最小实现：loop_period ≈ (跑马场一圈长度) / (盘旋速度)
        // - 一圈长度：由 ctx.racetrack.length/radius 估计（未知则退化为圆）
        // - 盘旋速度：优先取该 receiver 的 cruise_speed；没有则用 ctx.racetrack.speed；再没有就用 vt
        const double loop_len_m = RacetrackLoopLengthM(ctx);

        double v_loop = 0.0;
        if (!combo.choices.empty()) {
          const std::string& rid = combo.choices.front().receiver_id;
          auto it = ctx.receiver_speed_bounds.find(rid);
          if (it != ctx.receiver_speed_bounds.end()) {
            v_loop = it->second.cruise_speed_mps;
          }
        }
        if (v_loop <= 1e-6) v_loop = ctx.racetrack.speed_mps;
        if (v_loop <= 1e-6) v_loop = vt;
        const double loop_period_s = SafeDiv(loop_len_m, v_loop, 10.0);

        deps_.loop_calculator->Apply(cands, loop_period_s);

        // 白框 5：反查表选会合高度（这里可能需要“逐 receiver”处理）
        // 由于 combo 包含多个 receiver 的选择，你们工程里可能要对每个 receiver 分别做一次流程。
        // 这里给的是“单 receiver”示例：receiver_id 从 combo 中取一个（你可扩展为循环每个 receiver）。
        if (!combo.choices.empty()) {
          deps_.altitude_selector->Apply(cands, ctx, combo.choices.front().receiver_id);
        }

        // 白框 6：粗算代价，取该 combo 下最优 candidate（此处才决定 receiver 真速/高度/表速）
        std::optional<MeetingCandidate> best_cand =
            deps_.cost_evaluator->PickBest(cands, ctx, pref);

        if (!best_cand.has_value()) {
          // 这个 combo 在 4/5/6 之后仍不可行
          continue;
        }

        // 组合内比较：Best(combo | vt)
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

    // 白框 9：全局选优
    return deps_.global_selector->Select(best_under_all_vt, pref);
  }

private:
  Dependencies deps_;
};

} // namespace refuel::mode0
